import rclpy
import signal
import sys
import os
import numpy as np
from math import sqrt 
from rclpy.node import Node
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray
# from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix

import torch
from skrl.models.torch import Model
from ament_index_python.packages import get_package_share_directory
import torch.nn as nn
from scipy.spatial.transform import Rotation as R


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)

        self.current_pose = None
        self.setpoint = np.array([0.0, 0.0, 1.0])

        # Set up control loop
        self.control_frequency = 100.0
        self.dt = 1.0 / self.control_frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.pre_start_counter = 0
        self.pre_start_steps = self.control_frequency
        self.armed = False

        self.gui = GUI(self)

        # === Load RL environment and agent for inference ===
        package_dir = get_package_share_directory("controller_rl")
        checkpoint_path = os.path.join(package_dir, "best_model_bundle.pt")

        checkpoint = torch.load(checkpoint_path, map_location="cuda" if torch.cuda.is_available() else "cpu", weights_only=False)
        policy_model = InferencePolicy(
            observation_space=checkpoint["observation_space"],
            action_space=checkpoint["action_space"],
            device="cuda" if torch.cuda.is_available() else "cpu",
            cfg=checkpoint["model_cfg"]
        )

        policy_model.load_state_dict(checkpoint["state_dict"])
        policy_model.eval()
        self.agent = policy_model

        self.counter = 0

    # Recieve motion capture data
    def pose_callback(self, msg: MotionCaptureState):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation =  np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

        
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])


        quat_xyzw = [orientation[1], orientation[2], orientation[3], orientation[0]]
        rotation_matrix = R.from_quat(quat_xyzw).as_matrix()

        linear_velocity_body = rotation_matrix.T @ linear_velocity
        angular_velocity_body = angular_velocity

        # Gravity in body frame (assuming global [0, 0, -1])
        gravity_world = np.array([0.0, 0.0, -1.0])
        gravity_body = rotation_matrix.T @ gravity_world

        # Relative goal in body frame
        desired_pos_rel_world = self.setpoint - position
        desired_pos_b = rotation_matrix.T @ desired_pos_rel_world

        # Match the 12D input from training
        self.current_pose = np.concatenate((
            linear_velocity_body,     # 3
            angular_velocity_body,    # 3
            gravity_body,             # 3
            desired_pos_b             # 3
        )).astype(np.float32)


    def control_loop(self):

        # For saftey generate a message with all channels set to 0.0
        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = 0.0
        msg.channel_3 = 0.0

        # Pre-start state: Send 0.05 on all channels for one second before starting control loop.
        if self.armed and self.pre_start_counter < self.pre_start_steps:
            msg.armed = True
            msg.channel_0 = 0.05
            msg.channel_1 = 0.05
            msg.channel_2 = 0.05
            msg.channel_3 = 0.05
            self.pre_start_counter += 1

        elif self.armed and self.current_pose is not None:
            state = self.current_pose.astype(np.float32)
            obs = torch.from_numpy(state).unsqueeze(0)

            with torch.no_grad():
                action = self.agent.act(obs, role="policy", deterministic=True)
            action_np = action.cpu().numpy().flatten()

            force = action_np[0]  # Thrust
            roll = action_np[1] if len(action_np) > 1 else 0.0   # Roll
            pitch = action_np[2] if len(action_np) > 2 else 0.0  # Pitch
            yaw = action_np[3] if len(action_np) > 3 else 0.0    # Yaw
            
            # Clamp and scale force from [-1,1] to [0,1] then to actual thrust
            force = max(-1.0, min(force, 1.0))
            force = (force + 1.0) / 2.0  # Scale to [0,1]
            
            # Clamp angular commands
            roll = max(-1.0, min(roll, 1.0))
            pitch = max(-1.0, min(pitch, 1.0))
            yaw = max(-1.0, min(yaw, 1.0))

            # Quadcopter parameters
            Cf = 1.48e-6  # Force coefficient
            Ct = 0.01 * Cf  # Torque coefficient for yaw
            l_x = 0.0865  # Distance from center to motor in x direction (for pitch)
            l_y = 0.073   # Distance from center to motor in y direction (for roll)
            max_motor_speed = 4631.0
            
            # Moment magnitude constants (same as Isaac Sim)
            roll_magnitude = 0.01   
            pitch_magnitude = 0.01  
            yaw_magnitude = 0.01
            
            # Calculate thrust-to-weight ratio scaling
            training_tw_ratio = 3.8
            mass_estimate = 0.65  # kg (adjust based on your actual drone mass)
            g = 9.81
            max_thrust_needed = training_tw_ratio * mass_estimate * g

            desired_thrust = force * max_thrust_needed
            base_motor_omega_squared = desired_thrust / (4 * Cf)
            
            desired_roll_moment = roll * roll_magnitude
            desired_pitch_moment = pitch * pitch_magnitude
            desired_yaw_moment = yaw * yaw_magnitude
            
            roll_differential = desired_roll_moment / (2 * Cf * l_y)
            pitch_differential = desired_pitch_moment / (2 * Cf * l_x)
            yaw_differential = desired_yaw_moment / (2 * Ct)
            
            omega1_squared = max(0, base_motor_omega_squared ) # - roll_differential + pitch_differential + yaw_differential 
            omega2_squared = max(0, base_motor_omega_squared ) # - roll_differential - pitch_differential - yaw_differential
            omega3_squared = max(0, base_motor_omega_squared ) # + roll_differential + pitch_differential + yaw_differential
            omega4_squared = max(0, base_motor_omega_squared ) # + roll_differential - pitch_differential - yaw_differential

            # Take square root to get angular velocities
            omega1 = sqrt(omega1_squared)
            omega2 = sqrt(omega2_squared)
            omega3 = sqrt(omega3_squared)
            omega4 = sqrt(omega4_squared)
            
            # Normalize by max motor speed to get motor commands [0,1]
            u1 = min(omega1 / max_motor_speed, 1.0)
            u2 = min(omega2 / max_motor_speed, 1.0)
            u3 = min(omega3 / max_motor_speed, 1.0)
            u4 = min(omega4 / max_motor_speed, 1.0)

            u = [u1, u2, u3, u4]
            print(f"Observation: {np.round(state, 3)}")
            print(f"Throttle: {action_np[0]:.3f} Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")


            msg.armed = True
            msg.channel_0 = u1
            msg.channel_1 = u2
            msg.channel_2 = u3
            msg.channel_3 = u4

            self.counter += 1

            if self.counter > 100:
                self.on_close()
        else:         
            self.pre_start_counter = 0

        self.cmd_publisher_.publish(msg)
        print(f"control output {msg.channel_0}, {msg.channel_1}, {msg.channel_2}, {msg.channel_3}")

    # GUI functions
    def signal_handler(self, sig, frame):
        self.on_close()

    def on_close(self):
        self.gui.quit()
        rclpy.shutdown()
        sys.exit(0)

class InferencePolicy(Model):
    def __init__(self, observation_space, action_space, device, cfg):
        super().__init__(observation_space, action_space, device)
        input_dim = observation_space.shape[0]
        output_dim = action_space.shape[0]

        hidden_layers = cfg.get("hidden_layers", [64, 64])
        activation = getattr(nn, cfg.get("activation", "ReLU"))

        layers = []
        last_dim = input_dim
        for h in hidden_layers:
            layers.append(nn.Linear(last_dim, h))
            layers.append(activation())
            last_dim = h
        self.net_container = nn.Sequential(*layers) 

        self.policy_layer = nn.Linear(last_dim, output_dim)
        self.value_layer = nn.Linear(last_dim, 1)
        self.log_std_parameter = nn.Parameter(torch.zeros(output_dim))

    def act(self, inputs, role, deterministic=False):
        x = self.net_container(inputs)
        return self.policy_layer(x)



def main(args=None): 
    rclpy.init(args=args)
    controller = Controller()
    signal.signal(signal.SIGINT, controller.signal_handler)
    while rclpy.ok():
        rclpy.spin_once(controller, timeout_sec=0.1)
        controller.gui.handle_events()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
