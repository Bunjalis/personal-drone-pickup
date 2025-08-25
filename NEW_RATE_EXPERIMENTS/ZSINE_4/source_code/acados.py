import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from .dynamics import QuadDynamics
import casadi as ca
from acados_template import AcadosSim, AcadosSimSolver


def generate_ocp_controller(dynamics=None):
    # Define the dynamics model
    if dynamics is None:
        quad_dynamics = QuadDynamics()
    else:
        quad_dynamics = dynamics

    dynamics_expr = quad_dynamics.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = quad_dynamics.x  # [p, q, v, r, u] (17 states)
    model.u = quad_dynamics.u_dot  # [u_dot] (control input)
    model.p = quad_dynamics.p_param  # [ kT] parameters


    # Explicit Dynamics
    model.f_expl_expr = dynamics_expr(quad_dynamics.x, quad_dynamics.u_dot, quad_dynamics.p_param)

    # Implicit Dynamics
    xdot = ca.MX.sym('xdot', model.x.size()[0])
    f_impl_expr = model.f_expl_expr - xdot
    model.xdot = xdot
    model.f_impl_expr = f_impl_expr



    # Create OCP object
    ocp = AcadosOcp()
    ocp.model = model

    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 2.0

    nu = 4  # Number of control inputs
    nx = 17  # New state dimension (no omega)
    ny = nx + nu

    # Cost matrices (tune as needed)
    Q_mat = 2 * np.diag([
        50.0, 50.0, 50.0,    # position
        2.0, 2.0, 2.0, 2.0,  # quaternion
        0.1, 0.1, 0.1,      # velocity
        0.1, 0.1, 0.1,      # angular rates
        0.0001, 0.0001, 0.0001, 0.0001,  # u
    ])
    R_mat = 2 * np.diag([1.0, 1.0, 15.0, 1.0])
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat  # Terminal cost only considers the state

    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x

    x0 = np.zeros(nx)
    # No initial quaternion constraint - let it be free
    ocp.constraints.x0 = x0

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = 1 * np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-nu:, -nu:] = 1 * np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((nx, ))
    ocp.cost.yref[3] = 1
    ocp.cost.yref_e[3] = 1

    ocp.parameter_values = np.array([38.0, 0.5, 0.07, 80.0, 250.0, 0.5])  # Default parameters: thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo


    # Set solver options (as before)
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.nlp_solver_max_iter = 500
    ocp.solver_options.qp_solver_iter_max = 300
    ocp.solver_options.qp_solver_tol_stat = 1e-4
    ocp.solver_options.qp_solver_tol_eq = 1e-4
    ocp.solver_options.qp_solver_tol_ineq = 1e-4
    ocp.solver_options.qp_solver_tol_comp = 1e-4
    ocp.solver_options.nlp_solver_tol_stat = 1e-4
    ocp.solver_options.nlp_solver_tol_eq = 1e-4
    ocp.solver_options.nlp_solver_tol_ineq = 1e-4
    ocp.solver_options.nlp_solver_tol_comp = 1e-4
    ocp.solver_options.levenberg_marquardt = 1e-3


    # Add constraints for throttle (x[2])
    # Throttle bound
    max_rate = 1.0

    ocp.constraints.lbx = np.array([0.0, -max_rate, -max_rate, -max_rate])   # Lower bounds: throttle and last 4 states
    ocp.constraints.ubx = np.array([0.8,  max_rate,  max_rate,  max_rate])   # Upper bounds: throttle and last 4 states
    ocp.constraints.idxbx = np.array([15, 13, 14, 16])          # Indices: throttle and last 4 states


    # Set input constraints (tune as needed)
    ocp.constraints.lbu = np.array([-15.0, -15.0, -5.0, -15.0])  # throttle, roll_rate, pitch_rate, yaw_rate
    ocp.constraints.ubu = np.array([15.0, 15.0, 5.0, 15.0])
    ocp.constraints.idxbu = np.arange(nu)

    # Create OCP solver
    ocp_solver = AcadosOcpSolver(ocp)

    # Create simulation configuration
    sim = AcadosSim()
    sim.model = ocp.model
    sim.solver_options.T = 1.0 / 30.0  # Set integrator to run at 30Hz
    sim.parameter_values = np.array([38.0, 0.5, 0.07, 80.0, 250.0, 670.0])  # Default parameters: thrust_ratio, drag_coeff_z, tau_rate, centre_rate_deg, max_rate_deg, rate_expo
    sim_solver = AcadosSimSolver(sim)

    return ocp_solver, sim_solver


def set_initial_guess(ocp_solver, N_horizon=20):

    u_init = np.array([0.0, 0.0, -1.0, 0.0])  # Convert to numpy array
    for i in range(N_horizon):
        ocp_solver.set(i, "u", u_init)


def warm_start_from_previous_solution(ocp_solver, N_horizon=20):
    """
    Warm start the MPC solver using the previous solution shifted by one time step
    """
    for i in range(N_horizon - 1):
        u_prev = ocp_solver.get(i + 1, "u")
        ocp_solver.set(i, "u", u_prev)
    u_last = ocp_solver.get(N_horizon - 1, "u")
    ocp_solver.set(N_horizon - 1, "u", u_last)
