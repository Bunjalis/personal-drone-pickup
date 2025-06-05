import rclpy
import signal
import sys
import numpy as np
from rclpy.node import Node
from .gui import GUI
from interfaces.msg import MotionCaptureState, ELRSCommand
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix

sys.path.append("/home/jett/Thesis/src/IsaacLab/")
import torch
from skrl.models.torch import GaussianModel, DeterministicModel
from skrl.agents.torch.ppo import PPO
from isaaclab_tasks import Isaac_Quadcopter_Direct_v0
from isaaclab.envs import DirectRLEnvCfg
from isaaclab_rl.skrl import SkrlVecEnvWrapper



class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_publisher_ = self.create_publisher(ELRSCommand, '/ELRSCommand', 10)
        self.pose_subscription_ = self.create_subscription(MotionCaptureState, '/motion_capture_state', self.pose_callback, 10)

        self.current_pose = None
        self.setpoint = np.array([0.0, 0.0, 1.0])

        # Set up control loop
        self.control_frequency = 30.0
        self.dt = 1.0 / self.control_frequency
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.pre_start_counter = 0
        self.pre_start_steps = self.control_frequency
        self.armed = False

        self.gui = GUI(self)

        # === Load RL environment and agent for inference ===
        env_cfg = DirectRLEnvCfg()
        env = Isaac_Quadcopter_Direct_v0(cfg=env_cfg)
        self.env = SkrlVecEnvWrapper(env, ml_framework="torch")

        # Define models matching your yaml architecture
        self.models = {}
        self.models["policy"] = GaussianModel(
            observation_space=self.env.observation_space,
            action_space=self.env.action_space,
            device="gpu" if torch.cuda.is_available() else "cpu",
            clip_actions=False,
            clip_log_std=True,
            min_log_std=-20.0,
            max_log_std=2.0,
            initial_log_std=0.0,
            net={"layers": [64, 64], "activation": "elu"}
        )
        self.models["value"] = DeterministicModel(
            observation_space=self.env.observation_space,
            action_space=self.env.action_space,
            device="gpu" if torch.cuda.is_available() else "cpu",
            clip_actions=False,
            net={"layers": [64, 64], "activation": "elu"}
        )

        self.agent = PPO(models=self.models, memory=None, cfg=None,
                        observation_space=self.env.observation_space,
                        action_space=self.env.action_space,
                        device="gpu")

        # Load trained weights
        self.agent.load("drone_cage_control/src/controller_RL/best_agent.pt")
        self.agent.eval()


    # Recieve motion capture data
    def pose_callback(self, msg: MotionCaptureState):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation =  np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])

        rotation_matrix = quaternion_matrix(np.array([orientation[1],orientation[2],orientation[3],orientation[0]]))[:3, :3]
        linear_velocity_body = np.dot(rotation_matrix.T, linear_velocity)

        print(f'linear_velocity_body: {linear_velocity_body}')

        self.current_pose = np.concatenate((position, orientation, linear_velocity_body, angular_velocity))

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
            # Use current_pose as observation
            state = self.current_pose.astype(np.float32)
            # Convert to torch tensor with batch dimension
            obs = torch.from_numpy(state).unsqueeze(0)  # shape: (1, obs_dim)

            # Run PPO inference - get action tensor (batch size 1)
            with torch.no_grad():
                action = self.agent.act(obs, deterministic=True)

            # Convert action tensor to numpy array
            action_np = action.cpu().numpy().flatten()

            # Map the action values to ELRS channels (assumed normalized between -1 and 1)
            scaled_action = 0.5 * (action_np + 1.0)

            msg.armed = True
            msg.channel_0 = float(scaled_action[0])
            msg.channel_1 = float(scaled_action[1])
            msg.channel_2 = float(scaled_action[2])
            msg.channel_3 = float(scaled_action[3])
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
