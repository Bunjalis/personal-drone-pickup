import json
import time
import datetime
import torch
import numpy as np
import copy
import math
import matplotlib.pyplot as plt

# --- 1. Imports ---
start_time = time.time()
print("importing trajectories")
from trajectories import hover_trajectory, z_sin_trajectory, xyz_sine_trajectory, circle_trajectory, power_loop_trajectory, figure8_zsine_trajectory, fast_xyz_sine_trajectory
end_time = time.time()
print(f"finished importing trajectories, took {end_time-start_time}")

start_time = time.time()
print("Importing Acados")
from acados import generate_ocp_controller, set_initial_guess, warm_start_from_previous_solution, set_trajectory_reference_aligned, update_ocp_parameters
end_time = time.time()
print(f"finished importing acados, took {end_time-start_time}")

from wind import Wind

FREQUENCY_HZ = 30.0
DT = 1.0 / FREQUENCY_HZ
LOGGING_NAME = "mpc_controller"

# --- 2. Helper: Quaternion to Rotation Matrix ---
def q_to_rot_mat(q):
    """
    Converts quaternion [qw, qx, qy, qz] to 3x3 rotation matrix.
    Assumes q is [w, x, y, z].
    """
    w, x, y, z = q
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])
    return R

# --- 3. The New NDOB Class ---
class NDOB:
    def __init__(self, gain, dt):
        self.L = gain * np.eye(3) # Gain matrix (diagonal)
        self.dt = dt
        self.z = np.zeros(3)      # Internal state of the observer
        self.g = np.array([0.0, 0.0, -9.81]) # Gravity vector

    def update(self, velocity, quaternion, thrust_cmd, thrust_ratio, drag_coeff):
        """
        Estimates the disturbance acceleration and maps it to wind.
        """
        R = q_to_rot_mat(quaternion)
        
        thrust_vec_body = np.array([0.0, 0.0, thrust_cmd * thrust_ratio])
        accel_thrust = R @ thrust_vec_body
        accel_drag = -drag_coeff * velocity
        nominal_accel = accel_thrust + self.g + accel_drag


        z_dot = -self.L @ (self.z + self.L @ velocity + nominal_accel)
        self.z += z_dot * self.dt
        disturbance_accel = self.z + self.L @ velocity
        
        if drag_coeff > 0.001:
            wind_estimate = disturbance_accel / drag_coeff
        else:
            wind_estimate = np.zeros(3)
            
        return wind_estimate


class simulator():        
  def __init__(self, p_truth=0.0, wind_estimator=None):
    ## General settings
    self.traj, trajectory_name = xyz_sine_trajectory(DT)
 
    self.observed_state_history = []       
    self.control_history = []
    self.estimated_state_history = []
    self.wind_displacement_history = []
    self.wind_history = []
    self.wind_estimation_history = []
    
    # Init NDOB with a Gain (L). 
    # Start with L=5.0. Higher = Faster but noisier. Lower = Smoother but laggier.
    self.ndob = NDOB(gain=5.0, dt=DT) 
  
    
    self.current_pose = np.array([0.0, 0.0, 0.0,  ## Position
                         1.0, 0.0, 0.0, 0.0,      ## Quaternion
                         0.0, 0.0, 0.0,           ## Linear velocity
                         0.0, 0.0, 0.0])          ## Angular velocity
    
    self.step_counter = 0
    self.steps = self.traj.shape[1] - 1
 
    ## Define MPC Controller parameters
    self.N = 20
    self.skip_steps = 3
    self.first_solve = True
    print("Generating first set of controllers \n")
    self.ocp, self.sim_integrator = generate_ocp_controller()
 
    # NOTE: Shadow controller removed here. We use NDOB class instead.
 
    # Parameters: [Thrust ratio, drag ratio, angular velocity tau, centre rate, max rate, expo]
    self.actual_params = np.array([38.0, 0.5, 0.12, 100.0, 100.0, 0.5])
    self.est_params = np.array([38.0, 0.5, 0.12, 100.0, 100.0, 0.5])
 
    ## Wind model
    self.wind_strength = 10
    self.wind_variability = 10
    self.wind = Wind(wind_strength=self.wind_variability, baseline=self.wind_strength)
 
    ## Set the wind estimator
    self.wind_estimator = wind_estimator
    self.p_truth = p_truth
 
  
  def control_loop(self):            
    if self.step_counter + self.N * self.skip_steps > self.steps:
       return False

    ### 1. Get current state and previous control ###
    estimated_state = copy.deepcopy(self.current_pose[:13])
    
    # Extract specific states for NDOB
    current_vel = self.current_pose[7:10]
    current_quat = self.current_pose[3:7] # [w, x, y, z]

    if len(self.control_history) == 0:
        control = np.array([0.0, 0.0, 0.0, 0.0])
        prev_thrust_cmd = 0.0 # Hover thrust or zero
    else:
        control = np.array(self.control_history[-1][0:4])
        prev_thrust_cmd = control[2] # Assuming index 2 is thrust [roll, pitch, thrust, yaw]

    estimated_state_with_control = np.concatenate((estimated_state, control))

    ### 2. Estimate wind (NDOB Approach) ###
    # Get parameters needed for physics model
    thrust_ratio = self.est_params[0]
    drag_coeff = self.est_params[1]
    
    # Update NDOB
    wind_estimate = self.ndob.update(
        velocity=current_vel,
        quaternion=current_quat,
        thrust_cmd=prev_thrust_cmd,
        thrust_ratio=thrust_ratio,
        drag_coeff=drag_coeff
    )
    
    # Note: NDOB inherently smooths via integration, so extra EMA filter might be redundant, 
    # but we can keep it for extra smoothness if desired.
    # For now, I'll pass the NDOB output directly.

    ### 3. Determine wind for MPC ###
    wind = wind_estimate

    ### 4. Solve MPC ###
    set_trajectory_reference_aligned(self.ocp, self.traj, self.N, self.step_counter, self.skip_steps, self.est_params, wind, FREQUENCY_HZ)
    self.ocp.set(0, "lbx", estimated_state_with_control)
    self.ocp.set(0, "ubx", estimated_state_with_control)
    if self.first_solve:
        set_initial_guess(self.ocp, self.N)
        self.first_solve = False
    else:
        warm_start_from_previous_solution(self.ocp, self.N)
    status = self.ocp.solve()
    if status != 0:
        raise Exception(f'acados returned status {status}.')
    x = self.ocp.get(1, "x").copy()
    u = x[-4:]
    u_rate = self.ocp.get(0, "u").copy()

    ### 5. Simulate state transition (will be replaced by real drone) ###
    wind_dt = self.wind.get_wind(self.step_counter, FREQUENCY_HZ)
    self.wind_history.append(wind_dt)

    self.sim_integrator.set('x', estimated_state_with_control.reshape(-1))
    self.sim_integrator.set('u', u.reshape(-1))
    self.sim_integrator.set('p', np.concatenate([self.actual_params, np.array([1.0, 0.0, 0.0, 0.0]), wind_dt]))
    self.sim_integrator.solve()
    x_sim = self.sim_integrator.get('x').copy()
    self.current_pose = x_sim[:13].copy()

    self.step_counter += 1
    
    ### 6. Logging ###
    self.control_history.append( np.concatenate( (u, u_rate) ).tolist() )
    self.observed_state_history.append( self.current_pose[:13].tolist() )
    self.wind_estimation_history.append( wind_estimate.tolist())
    return True
 
if __name__ == '__main__':

  print("Initiating simulator")
  sim = simulator()
 
  print("Beginning control loop")
  while sim.control_loop() is True:
    continue
 
  ## Plot the simulation results
  fig, obs_axes = plt.subplots(2, 3, figsize=(18, 15))
  obs_axes = obs_axes.flatten()
 
  distance_err = []
  x_err = []
  y_err = []
  z_err = []
  for t, pose_t in enumerate(sim.observed_state_history):
    x_err.append(pose_t[0]-sim.traj[0,t])
    y_err.append(pose_t[1]-sim.traj[1,t])
    z_err.append(pose_t[2]-sim.traj[2,t])
  
  runtime = len(sim.observed_state_history)
  timesteps = DT * np.arange(runtime)
 
  # 1. Distance error
  obs_axes[0].plot(timesteps, x_err, label='x_err')
  obs_axes[0].plot(timesteps, y_err, label='y_err')
  obs_axes[0].plot(timesteps, z_err, label='z_err')
  obs_axes[0].legend()
  obs_axes[0].set_title("Distance between the quadcopter and target")
  obs_axes[0].set_xlabel("Time (s)")
  obs_axes[0].set_ylabel("Distance")
  obs_axes[0].grid(alpha=0.3)
 
  # 2. Trajectory mapping
  obs_axes[1].plot(timesteps, sim.traj[0,:runtime], label="x_ref", linestyle='--')
  obs_axes[1].plot(timesteps, sim.traj[1,:runtime], label="y_ref", linestyle='--')
  obs_axes[1].plot(timesteps, sim.traj[2,:runtime], label="z_ref", linestyle='--')
  obs_axes[1].plot(timesteps, np.array(sim.observed_state_history)[:runtime, 0], label="x_actual")
  obs_axes[1].plot(timesteps, np.array(sim.observed_state_history)[:runtime, 1], label="y_actual")
  obs_axes[1].plot(timesteps, np.array(sim.observed_state_history)[:runtime, 2], label="z_actual")
  obs_axes[1].legend()
  obs_axes[1].set_title("Trajectory vs Actual Position")
  obs_axes[1].set_xlabel("Time (s)")
  obs_axes[1].set_ylabel("Position")
  obs_axes[1].grid(alpha=0.3)
 
  # 3. Control outputs
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 0], label="roll")
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 1], label="pitch")
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 2], label="thrust")
  obs_axes[2].plot(timesteps, np.array(sim.control_history)[:runtime, 3], label="yaw")
 
  obs_axes[2].legend()
  obs_axes[2].set_title("Control")
  obs_axes[2].set_xlabel("Time (s)")
  obs_axes[2].set_ylabel("Throttle")
  obs_axes[2].grid(alpha=0.3)
 
  # 4. Wind
  adjusted_runtime = int(runtime/FREQUENCY_HZ)
  wind_history = sim.wind.history(adjusted_runtime)
  obs_axes[3].plot(np.arange(adjusted_runtime), wind_history[0], label="wind_across")
  obs_axes[3].plot(np.arange(adjusted_runtime), wind_history[1], label="wind_along")  
  obs_axes[3].plot(np.arange(adjusted_runtime), wind_history[2], label="wind_vertical")
  obs_axes[3].legend()
  obs_axes[3].set_title("Wind speeds")
  obs_axes[3].set_xlabel("Time (s)")
  obs_axes[3].set_ylabel("Wind speed (m/s)")
  obs_axes[3].grid(alpha=0.3)
 
  # 5. Estimated wind
  adjusted_runtime = int(runtime/FREQUENCY_HZ)
  obs_axes[4].plot(np.arange(len(sim.wind_estimation_history)), np.array(sim.wind_estimation_history)[:, 0], label="wind_estimate_across")
  obs_axes[4].plot(np.arange(len(sim.wind_estimation_history)), np.array(sim.wind_estimation_history)[:, 1], label="wind_estimate_along")  
  obs_axes[4].plot(np.arange(len(sim.wind_estimation_history)), np.array(sim.wind_estimation_history)[:, 2], label="wind_estimate_vertical")
  obs_axes[4].legend()
  obs_axes[4].set_title("Estiamted Wind speeds")
  obs_axes[4].set_xlabel("Time (s)")
  obs_axes[4].set_ylabel("Wind speed (m/s)")
  obs_axes[4].grid(alpha=0.3)
  plt.show()
  print(f"DONE: {sim.step_counter} STEPS TAKEN")
 
  data_to_save = {
    "input" : np.array(sim.wind_estimation_history).tolist(),
    "data": np.array(sim.wind_history).tolist(),
  }
 
  date_tag = datetime.datetime.now()
  tag = date_tag.strftime("%d") + date_tag.strftime("%m") + date_tag.strftime("%y") + date_tag.strftime("%H") + date_tag.strftime("%M")
  with open(f"./mpc_controller/data/data_{tag}_strength-{sim.wind_strength}_var-{sim.wind_variability}.json", "w") as f:
    json.dump(data_to_save, f)