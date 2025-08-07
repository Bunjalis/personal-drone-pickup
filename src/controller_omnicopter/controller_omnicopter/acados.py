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

    nu = 8  # Number of control inputs
    nx = 13  # New state dimension (no omega)
    ny = nx + nu

    # Cost matrices (tune as needed)
    Q_mat = 2 * np.diag([
        2.1, 2.1, 2.1,    # position
        2.1, 2.1, 2.1, 2.1,  # quaternion
        0.1, 0.1, 0.1,      # velocity
        0.1, 0.1, 0.1      # angular rates
    ])
    # Remove input cost matrix R_mat and its usage
    ocp.cost.W = Q_mat
    ocp.cost.W_e = Q_mat  # Terminal cost only considers the state

    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x

    x0 = np.zeros(nx)
    x0[3] = 1  # Initial quaternion w=1
    ocp.constraints.x0 = x0

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.Vx = np.eye(nx)
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((nx, ))
    ocp.cost.yref_e = np.zeros((nx, ))
    ocp.cost.yref[3] = 1
    ocp.cost.yref_e[3] = 1

    # Set Vu to a zero matrix with dimensions (ny, nu)
    ocp.cost.Vu = np.zeros((nx, nu))

    # Set solver options - improved for quadratic thrust model
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    
    # Increase iterations for better convergence with nonlinear model
    ocp.solver_options.nlp_solver_max_iter = 3000
    ocp.solver_options.qp_solver_iter_max = 1500
    
    # Relax tolerances for better convergence
    ocp.solver_options.qp_solver_tol_stat = 1e-5
    ocp.solver_options.qp_solver_tol_eq = 1e-5
    ocp.solver_options.qp_solver_tol_ineq = 1e-5
    ocp.solver_options.qp_solver_tol_comp = 1e-5
    ocp.solver_options.nlp_solver_tol_stat = 1e-5
    ocp.solver_options.nlp_solver_tol_eq = 1e-5
    ocp.solver_options.nlp_solver_tol_ineq = 1e-5
    ocp.solver_options.nlp_solver_tol_comp = 1e-5
    
    # Increase regularization for numerical stability
    #ocp.solver_options.levenberg_marquardt = 1e-4
    
    # Add regularization for ill-conditioned problems
    #ocp.solver_options.regularize_method = 'CONVEXIFY'


    # Set input constraints for bidirectional control [-1, 1]
    # Omnicopter motors can run backwards for full 6-DOF control
    max = 0.45
    ocp.constraints.lbu = np.array([-max, -max, -max, -max, -max, -max, -max, -max])  # reverse thrust
    ocp.constraints.ubu = np.array([max, max, max, max, max, max, max, max])  # forward thrust
    ocp.constraints.idxbu = np.arange(nu)

    # Create OCP solver
    ocp_solver = AcadosOcpSolver(ocp)

    # Create simulation configuration
    sim = AcadosSim()
    sim.model = ocp.model
    sim.solver_options.T = 1.0 / 100.0  # Set integrator to run at 30Hz
    sim_solver = AcadosSimSolver(sim)

    return ocp_solver, sim_solver


def calculate_hover_initial_guess():
    """
    Simple hover initial guess based on known working pattern
    
    Returns:
        u_hover: Initial guess for control inputs to achieve hover
    """
    hover_pattern = np.array([-0.28, 0.28, -0.28, 0.28, 0.28, -0.28, 0.28, -0.28])
    return hover_pattern


def set_initial_guess(ocp_solver, N_horizon=20):
    """
    Set initial guess for the MPC solver based on hover solution
    
    Args:
        ocp_solver: Acados OCP solver
        N_horizon: Prediction horizon length
    """
    u_hover = calculate_hover_initial_guess()
    
    # Set control initial guess to hover solution for all time steps
    for i in range(N_horizon):
        ocp_solver.set(i, "u", u_hover)


def warm_start_from_previous_solution(ocp_solver, N_horizon=20):
    """
    Warm start the MPC solver using the previous solution shifted by one time step
    
    Args:
        ocp_solver: Acados OCP solver
        N_horizon: Prediction horizon length
    """
    # Shift the previous solution: u[0] becomes u[1], u[1] becomes u[2], etc.
    for i in range(N_horizon - 1):
        u_prev = ocp_solver.get(i + 1, "u")  # Get control from next time step
        ocp_solver.set(i, "u", u_prev)       # Set it to current time step
    
    # For the last time step, use the control from the previous last time step
    u_last = ocp_solver.get(N_horizon - 1, "u")
    ocp_solver.set(N_horizon - 1, "u", u_last)
