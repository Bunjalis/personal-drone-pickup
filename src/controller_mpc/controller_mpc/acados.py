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

    # Use the rounded dynamics in the Acados model
    optimizer = QuadDynamics()
    
    # Get the dynamics expression
    dynamics_expr = optimizer.quad_dynamics()

    model = AcadosModel()
    model.name = 'quad_dynamics'
    model.x = optimizer.x  # Include omega in the state vector
    model.u = optimizer.u
    model.f_expl_expr = dynamics_expr(optimizer.x, optimizer.u)  # Use full state including omega internally

    # Create OCP object
    ocp = AcadosOcp()

    # Define the model using quadcopter dynamics
    ocp.model = model

    ocp.solver_options.N_horizon = 120
    ocp.solver_options.tf = 2.0

    # Define the cost function
    nx = 13  # Updated number of states
    nu = 4   # Number of inputs
    ny = nx + nu  # Number of outputs + inputs

    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    Q_mat = 2 * np.diag([5, 5, 10, 8, 8, 8, 8, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])
    R_mat = 2 * np.diag([0.1, 0.1, 0.1, 0.1])

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = 1 * np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-4:, -4:] = 3 * np.eye(nu)
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
    
    # Increase iterations and relax tolerances to improve convergence
    ocp.solver_options.nlp_solver_max_iter = 500
    ocp.solver_options.qp_solver_iter_max = 300
    
    # Relax QP solver tolerances
    #ocp.solver_options.qp_solver_tol_stat = 1e-1  # Stationarity tolerance (was 1e-3)
    #ocp.solver_options.qp_solver_tol_eq = 1e-1   # Equality constraint tolerance (was 1e-3)
    #ocp.solver_options.qp_solver_tol_ineq = 1e-1  # Inequality constraint tolerance (was 1e-3)
    #ocp.solver_options.qp_solver_tol_comp = 1e-1  # Complementarity tolerance (was 1e-3)
    
    # Relax NLP solver tolerances
    #ocp.solver_options.nlp_solver_tol_stat = 1e-1  # Optimality/stationarity
    #ocp.solver_options.nlp_solver_tol_eq = 1e-1    # Feasibility of equality constraints
    #ocp.solver_options.nlp_solver_tol_ineq = 1e-1  # Feasibility of inequality constraints
    #ocp.solver_options.nlp_solver_tol_comp = 1e-1  # Complementarity
    
    # Add Levenberg-Marquardt regularization to improve numerical stability
    #ocp.solver_options.levenberg_marquardt = 1e-1

    # Set initial conditionx0
    x0 = np.zeros(nx)
    x0[3] = 1
    ocp.constraints.x0 = x0

    # Set constraints on u[0]
    ocp.constraints.lbu = np.array([0.2, 0.2, 0.2, 0.2])
    ocp.constraints.ubu = np.array([0.7, 0.7, 0.7, 0.7])
    ocp.constraints.idxbu = np.arange(nu)

    # Create OCP solver
    ocp_solver = AcadosOcpSolver(ocp)
    
    # Create simulation configuration
    sim = AcadosSim()
    sim.model = ocp.model
    sim.solver_options.T = ocp.solver_options.tf / ocp.solver_options.N_horizon

    # Create simulation solver
    sim_solver = AcadosSimSolver(sim)

    # Return both the OCP solver and the explicitly created simulation solver
    return ocp_solver, sim_solver
