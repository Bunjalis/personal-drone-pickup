import numpy as np
import pandas as pd
import casadi as cs
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from dynamics import QuadDynamics
from acados_template import AcadosSim, AcadosSimSolver

def quat_to_euler_numpy(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

# Load the CSV data
data = pd.read_csv('log.csv')

# Extract relevant columns
# Control inputs and their derivatives
u0 = data['u0'].values
u1 = data['u1'].values
u2 = data['u2'].values
u3 = data['u3'].values
u0_rate = data['u0_rate'].values
u1_rate = data['u1_rate'].values
u2_rate = data['u2_rate'].values
u3_rate = data['u3_rate'].values

# Motion capture ground truth (mot_pose)
mot_x = data['mot_pose_x'].values
mot_y = data['mot_pose_y'].values
mot_z = data['mot_pose_z'].values
mot_qw = data['mot_pose_qw'].values
mot_qx = data['mot_pose_qx'].values
mot_qy = data['mot_pose_qy'].values
mot_qz = data['mot_pose_qz'].values
mot_vx = data['mot_pose_vx'].values
mot_vy = data['mot_pose_vy'].values
mot_vz = data['mot_pose_vz'].values
mot_avx = data['mot_pose_avx'].values
mot_avy = data['mot_pose_avy'].values
mot_avz = data['mot_pose_avz'].values

# Convert motion capture quaternions to Euler angles
mot_roll, mot_pitch, mot_yaw = quat_to_euler_numpy(mot_qw, mot_qx, mot_qy, mot_qz)

# Estimated pose (to use for frozen state)
est_x = data['est_pose_x'].values
est_y = data['est_pose_y'].values
est_z = data['est_pose_z'].values
est_qw = data['est_pose_qw'].values
est_qx = data['est_pose_qx'].values
est_qy = data['est_pose_qy'].values
est_qz = data['est_pose_qz'].values
est_vx = data['est_pose_vx'].values
est_vy = data['est_pose_vy'].values
est_vz = data['est_pose_vz'].values
est_avx = data['est_pose_avx'].values
est_avy = data['est_pose_avy'].values
est_avz = data['est_pose_avz'].values

# Convert estimated quaternions to Euler angles
est_roll, est_pitch, est_yaw = quat_to_euler_numpy(est_qw, est_qx, est_qy, est_qz)

# Timestamps
timestamps = data['timestamp'].values
dt_array = np.diff(timestamps)
dt_mean = np.mean(dt_array)

# Fixed parameters from the dataset
thrust_ratio = data['est_param_thrust_ratio'].iloc[-1]  # Use last value in case it varies
drag_coeff_z = data['est_param_drag_coeff_z'].iloc[-1]
centre_rate_deg = data['fixed_centre_rate_deg'].iloc[0]
max_rate_deg = data['fixed_max_rate_deg'].iloc[0]
rate_expo = data['fixed_rate_expo'].iloc[0]
angle_max_deg = data['fixed_angle_max_deg'].iloc[0]
tau_angle = data['est_param_tau_angle'].iloc[-1]
fc_roll_offset_deg = data['est_param_fc_roll_offset_deg'].iloc[-1]
fc_pitch_offset_deg = data['est_param_fc_pitch_offset_deg'].iloc[-1]

print(f"Dataset info:")
print(f"  Number of samples: {len(data)}")
print(f"  Mean dt: {dt_mean:.4f} seconds")
print(f"  Control frequency: {1.0/dt_mean:.2f} Hz")
print(f"  Fixed parameters:")
print(f"    thrust_ratio: {thrust_ratio:.4f}")
print(f"    drag_coeff_z: {drag_coeff_z:.4f}")
print(f"    tau_angle: {tau_angle:.4f}")
print(f"    angle_max_deg: {angle_max_deg:.4f}")
print(f"    fc_roll_offset_deg: {fc_roll_offset_deg:.6f}")
print(f"    fc_pitch_offset_deg: {fc_pitch_offset_deg:.6f}")


def create_acados_simulator(tau_rate_value, dt):
    """Create an Acados integrator with the given tau_rate."""
    from acados_template import AcadosModel
    
    # Initialize the dynamics model
    dyn = QuadDynamics()
    dynamics_expr = dyn.quad_dynamics()
    
    # Create Acados model
    model = AcadosModel()
    model.name = 'quad_dynamics_sim'
    model.x = dyn.x          # State: [p(3), q(4), v(3), r(3), u(4)] = 17
    model.u = dyn.u_dot      # Control: u_dot (4)
    model.p = dyn.p_param    # Parameters (10)
    
    # Explicit dynamics
    model.f_expl_expr = dynamics_expr(model.x, model.u, model.p)
    
    # Create simulator
    sim = AcadosSim()
    sim.model = model
    sim.solver_options.T = dt  # Integration time step
    sim.solver_options.num_stages = 4  # RK4
    sim.solver_options.num_steps = 1
    sim.solver_options.integrator_type = 'ERK'  # Explicit Runge-Kutta
    
    # Set parameter values
    p_param = np.array([
        thrust_ratio,
        drag_coeff_z,
        tau_rate_value,  # This is what we're optimizing
        centre_rate_deg,
        max_rate_deg,
        rate_expo,
        angle_max_deg,
        tau_angle,
        fc_roll_offset_deg,
        fc_pitch_offset_deg
    ])
    sim.parameter_values = p_param
    
    # Create solver
    sim_solver = AcadosSimSolver(sim)
    
    return sim_solver, p_param


def simulate_with_tau_rate(tau_rate_value):
    """
    Simulate the system using Acados integrator with the frozen estimated states.
    Returns the error between predicted and motion capture roll angle.
    """
    # Create Acados simulator with this tau_rate
    sim_solver, p_param = create_acados_simulator(tau_rate_value, dt_mean)
    
    # Storage for predicted angles
    pred_roll = []
    pred_pitch = []
    
    # Simulate through the dataset
    for i in range(len(data) - 1):
        # Current state (frozen from estimated values)
        # State vector: [p(3), q(4), v(3), r(3), u(4)] = 17 elements
        x_current = np.array([
            est_x[i], est_y[i], est_z[i],  # position (3)
            est_qw[i], est_qx[i], est_qy[i], est_qz[i],  # quaternion (4)
            est_vx[i], est_vy[i], est_vz[i],  # velocity (3)
            est_avx[i], est_avy[i], est_avz[i],  # angular velocity (3)
            u0[i], u1[i], u2[i], u3[i]  # control inputs (4)
        ])
        
        # Control input derivatives
        u_dot = np.array([u0_rate[i], u1_rate[i], u2_rate[i], u3_rate[i]])
        
        # Set state and control in simulator
        sim_solver.set("x", x_current)
        sim_solver.set("u", u_dot)
        sim_solver.set("p", p_param)
        
        # Integrate one step
        status = sim_solver.solve()
        if status != 0:
            print(f"Warning: Acados simulator failed at step {i} with status {status}")
        
        # Get predicted next state
        x_next = sim_solver.get("x")
        
        # Extract predicted quaternion
        q_pred = x_next[3:7]
        
        # Convert to Euler angles
        roll_pred, pitch_pred, _ = quat_to_euler_numpy(q_pred[0], q_pred[1], q_pred[2], q_pred[3])
        
        pred_roll.append(roll_pred)
        pred_pitch.append(pitch_pred)
    
    # Convert to arrays
    pred_roll = np.array(pred_roll)
    pred_pitch = np.array(pred_pitch)
    
    # Compute error against motion capture angles (skip first sample)
    # Focus ONLY on roll
    roll_error = mot_roll[1:] - pred_roll
    
    # Compute RMSE for roll only
    error = np.sqrt(np.mean(roll_error**2))
    
    return error, pred_roll, pred_pitch


def objective_function(tau_rate_value):
    """Objective function to minimize: RMSE of ROLL angle prediction."""
    error, _, _ = simulate_with_tau_rate(tau_rate_value[0])
    print(f"  tau_rate = {tau_rate_value[0]:.6f}, Roll RMSE = {error:.6f} rad ({np.rad2deg(error):.4f}°)")
    return error


# Initial guess for tau_rate
# Use 0.2 as the true initial parameter (as per your controller setup)
initial_tau_rate_true = 0.2
initial_tau_rate_from_csv = data['est_param_tau_rate'].iloc[-1]

print(f"\nInitial tau_rate (true parameter): {initial_tau_rate_true:.6f}")
print(f"Initial tau_rate (from CSV): {initial_tau_rate_from_csv:.6f}")

# Use the true initial value
initial_tau_rate = initial_tau_rate_true

# Optimize tau_rate
print("\nOptimizing tau_rate...")
result = minimize(
    objective_function,
    x0=[initial_tau_rate],
    method='Nelder-Mead',
    bounds=[(0.01, 1.0)],  # Reasonable bounds for tau_rate
    options={'maxiter': 100, 'disp': True}
)

optimal_tau_rate = result.x[0]
print(f"\n{'='*60}")
print(f"Optimization complete!")
print(f"{'='*60}")
print(f"Optimal tau_rate: {optimal_tau_rate:.6f}")
print(f"Initial tau_rate (true): {initial_tau_rate:.6f}")
print(f"Initial tau_rate (CSV):  {initial_tau_rate_from_csv:.6f}")
print(f"Change from true: {optimal_tau_rate - initial_tau_rate:.6f} ({100*(optimal_tau_rate - initial_tau_rate)/initial_tau_rate:.2f}%)")

# Simulate with optimal tau_rate and both initial values
_, pred_roll_opt, pred_pitch_opt = simulate_with_tau_rate(optimal_tau_rate)
_, pred_roll_init, pred_pitch_init = simulate_with_tau_rate(initial_tau_rate)
_, pred_roll_csv, pred_pitch_csv = simulate_with_tau_rate(initial_tau_rate_from_csv)

# Plot comparison
fig, axes = plt.subplots(2, 1, figsize=(14, 10))
time = timestamps[1:]  # Skip first sample

# Roll angle
axes[0].plot(time, np.rad2deg(mot_roll[1:]), 'k-', label='Motion Capture (Ground Truth)', linewidth=2.5, alpha=0.8)
axes[0].plot(time, np.rad2deg(pred_roll_init), 'b--', label=f'True Initial τ_rate={initial_tau_rate:.4f}', linewidth=1.5, alpha=0.7)
axes[0].plot(time, np.rad2deg(pred_roll_csv), 'r:', label=f'CSV τ_rate={initial_tau_rate_from_csv:.4f}', linewidth=1.5, alpha=0.7)
axes[0].plot(time, np.rad2deg(pred_roll_opt), 'g-', label=f'Optimal τ_rate={optimal_tau_rate:.4f}', linewidth=2)
axes[0].set_ylabel('Roll Angle (degrees)', fontsize=11)
axes[0].legend(fontsize=10)
axes[0].grid(True, alpha=0.3)
axes[0].set_title('Roll Angle Tracking with Optimized tau_rate', fontsize=12, fontweight='bold')

# Pitch angle
axes[1].plot(time, np.rad2deg(mot_pitch[1:]), 'k-', label='Motion Capture (Ground Truth)', linewidth=2.5, alpha=0.8)
axes[1].plot(time, np.rad2deg(pred_pitch_init), 'b--', label=f'True Initial τ_rate={initial_tau_rate:.4f}', linewidth=1.5, alpha=0.7)
axes[1].plot(time, np.rad2deg(pred_pitch_csv), 'r:', label=f'CSV τ_rate={initial_tau_rate_from_csv:.4f}', linewidth=1.5, alpha=0.7)
axes[1].plot(time, np.rad2deg(pred_pitch_opt), 'g-', label=f'Optimal τ_rate={optimal_tau_rate:.4f}', linewidth=2)
axes[1].set_ylabel('Pitch Angle (degrees)', fontsize=11)
axes[1].set_xlabel('Time (s)', fontsize=11)
axes[1].legend(fontsize=10)
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('tau_rate_optimization.png', dpi=150, bbox_inches='tight')
print(f"\nPlot saved to 'tau_rate_optimization.png'")
plt.show()

# Calculate and print error metrics
rmse_init_roll = np.sqrt(np.mean((mot_roll[1:] - pred_roll_init)**2))
rmse_init_pitch = np.sqrt(np.mean((mot_pitch[1:] - pred_pitch_init)**2))

rmse_csv_roll = np.sqrt(np.mean((mot_roll[1:] - pred_roll_csv)**2))
rmse_csv_pitch = np.sqrt(np.mean((mot_pitch[1:] - pred_pitch_csv)**2))

rmse_opt_roll = np.sqrt(np.mean((mot_roll[1:] - pred_roll_opt)**2))
rmse_opt_pitch = np.sqrt(np.mean((mot_pitch[1:] - pred_pitch_opt)**2))

print(f"\n{'='*60}")
print(f"RMSE Comparison (Roll Angle Only):")
print(f"{'='*60}")
print(f"  Roll angle:")
print(f"    True Initial (τ={initial_tau_rate:.4f}): {rmse_init_roll:.6f} rad ({np.rad2deg(rmse_init_roll):.4f}°)")
print(f"    CSV Value (τ={initial_tau_rate_from_csv:.4f}):    {rmse_csv_roll:.6f} rad ({np.rad2deg(rmse_csv_roll):.4f}°)")
print(f"    Optimal (τ={optimal_tau_rate:.4f}):     {rmse_opt_roll:.6f} rad ({np.rad2deg(rmse_opt_roll):.4f}°)")
print(f"    Improvement vs True: {100*(rmse_init_roll-rmse_opt_roll)/rmse_init_roll:.2f}%")
print(f"")
print(f"  Pitch angle (for reference, not optimized):")
print(f"    True Initial (τ={initial_tau_rate:.4f}): {rmse_init_pitch:.6f} rad ({np.rad2deg(rmse_init_pitch):.4f}°)")
print(f"    CSV Value (τ={initial_tau_rate_from_csv:.4f}):    {rmse_csv_pitch:.6f} rad ({np.rad2deg(rmse_csv_pitch):.4f}°)")
print(f"    Optimal (τ={optimal_tau_rate:.4f}):     {rmse_opt_pitch:.6f} rad ({np.rad2deg(rmse_opt_pitch):.4f}°)")
print(f"    Improvement vs True: {100*(rmse_init_pitch-rmse_opt_pitch)/rmse_init_pitch:.2f}%")
print(f"{'='*60}")

