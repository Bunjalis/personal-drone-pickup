import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from .dynamics import QuadDynamics
import casadi as ca
from acados_template import AcadosSim, AcadosSimSolver


def generate_ocp_controller():
    # Define the dynamics model
    quad_dynamics = QuadDynamics()
    dynamics_function = quad_dynamics.quad_dynamics()

    # Wrap the dynamics function to round inputs
    def rounded_dynamics(x, u):
        x_rounded = np.round(x, 3)
        u_rounded = np.round(u, 3)
        return dynamics_function(x_rounded, u_rounded)

    # Use the rounded dynamics in the Acados model
    optimizer = QuadDynamics()
    
    # Get the dynamics expression
    dynamics_expr = optimizer.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = optimizer.x
    model.u = optimizer.u
    model.f_expl_expr = dynamics_expr(optimizer.x, optimizer.u)  # Set the expression, not the function

    # Create OCP object
    ocp = AcadosOcp()

    # Define the model using quadcopter dynamics
    ocp.model = model

    ocp.solver_options.N_horizon = 60
    ocp.solver_options.tf = 2.0

    # Define the cost function
    nx = 13  # Number of outputs
    nu = 4   # Number of inputs
    ny = nx + nu  # Number of outputs + inputs

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    Q_mat = 2 * np.diag([10, 10, 10, 5.0, 5.0, 5.0, 5.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    R_mat = 2 * np.diag([0.1, 0.1, 0.1, 0.1])

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = 1*np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-4:, -4:] = 1*np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)



    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref = np.zeros((nx + nu, ))
    ocp.cost.yref_e = np.zeros((nx, ))
    ocp.cost.yref[3] = 1
    ocp.cost.yref_e[3] = 1 

    # Set prediction horizon
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.qp_solver_tol_stat = 1e-3  # Stationarity tolerance
    ocp.solver_options.qp_solver_tol_eq = 1e-3    # Equality constraint tolerance
    ocp.solver_options.qp_solver_tol_ineq = 1e-3  # Inequality constraint tolerance
    ocp.solver_options.qp_solver_tol_comp = 1e-3  # Complementarity tolerance

    # Set initial conditionx0
    x0 = np.zeros(nx)
    x0[3] = 1
    ocp.constraints.x0 = x0

    # Set constraints on u[0]
    ocp.constraints.lbu = np.array([0.0, 0.0, 0.0, 0.0])
    ocp.constraints.ubu = np.array([0.7, 0.7, 0.7, 0.7])
    ocp.constraints.idxbu = np.arange(nu)

    # Create solver
    ocp_solver = AcadosOcpSolver(ocp)
    return ocp_solver
