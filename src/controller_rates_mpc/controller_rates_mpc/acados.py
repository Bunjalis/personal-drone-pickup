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
    model.x = quad_dynamics.x  # [p, q, v, r] (13 states)
    model.u = quad_dynamics.u  # [throttle, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd]
    model.f_expl_expr = dynamics_expr(quad_dynamics.x, quad_dynamics.u)

    # Create OCP object
    ocp = AcadosOcp()
    ocp.model = model

    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 2.0

    nu = 4  # Number of control inputs
    nx = 13  # New state dimension (no omega)
    ny = nx + nu

    # Cost matrices (tune as needed)
    Q_mat = 2 * np.diag([
        5.0, 5.0, 5.0,    # position
        5.0, 5.0, 5.0, 5.0,  # quaternion
        1.0, 1.0, 1.0,      # velocity
        1.0, 1.0, 1.0      # angular rates
    ])
    R_mat = 2 * np.diag([0.5, 0.5, 0.1, 0.5])
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat  # Terminal cost only considers the state

    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x

    x0 = np.zeros(nx)
    x0[3] = 1  # Initial quaternion w=1
    ocp.constraints.x0 = x0

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = 1 * np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-nu:, -nu:] = 1 * np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((nx, ))
    ocp.cost.yref[3] = 1
    ocp.cost.yref_e[3] = 1

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


    rate_limit = 0.4

    # Set input constraints (tune as needed)
    ocp.constraints.lbu = np.array([-rate_limit, -rate_limit, 0.05, -0.4])  # throttle, roll_rate, pitch_rate, yaw_rate
    ocp.constraints.ubu = np.array([rate_limit, rate_limit, 0.5, 0.4])
    ocp.constraints.idxbu = np.arange(nu)

    # Create OCP solver
    ocp_solver = AcadosOcpSolver(ocp)

    # Create simulation configuration
    sim = AcadosSim()
    sim.model = ocp.model
    sim.solver_options.T = 1.0 / 30.0  # Set integrator to run at 30Hz
    sim_solver = AcadosSimSolver(sim)

    return ocp_solver, sim_solver
