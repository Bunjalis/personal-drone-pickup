#!/usr/bin/env python3
# controller_skrl_node.py
import os
import sys
import signal
import json
import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Messages
from interfaces.msg import MotionCaptureState, ELRSCommand
from scipy.spatial.transform import Rotation as R

import torch

# --- use CLASSIC gym for tiny env so skrl works with its "gym" wrapper ---
import gym as ogym
import yaml

# ---------------- Tiny finite-bounds classic-gym env ----------------
class _TinyGymEnv(ogym.Env):
    metadata = {}
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        big = np.float32(1e6)
        self.observation_space = ogym.spaces.Box(
            low=-big * np.ones((obs_dim,), dtype=np.float32),
            high= big * np.ones((obs_dim,), dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = ogym.spaces.Box(
            low=-1.0 * np.ones((act_dim,), dtype=np.float32),
            high= 1.0 * np.ones((act_dim,), dtype=np.float32),
            dtype=np.float32,
        )
        self._obs = np.zeros((obs_dim,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        return self._obs.copy(), {}

    def step(self, action):
        return self._obs.copy(), 0.0, False, False, {}


class SKRLController(Node):
    def __init__(self):
        super().__init__("skrl_controller")

        # ---------------- Parameters ----------------
        pkg = get_package_share_directory("controller_rl")

        # Paths (defaults expect you to drop files into your run folder under share/)
        self.declare_parameter("run_dir", os.path.join(pkg))  # you can point this to a specific run directory
        self.declare_parameter("checkpoint_rel", "best_agent.pt")
        self.declare_parameter("agent_yaml_rel", "agent.yaml")
        self.declare_parameter("round_decimals", 3)

        # Optional action scaling (unit [-1,1] -> actuator space). Leave None to publish unit actions.
        self.declare_parameter("action_low", None)   # e.g. [0.0, -1.0, -1.0, -1.0]
        self.declare_parameter("action_high", None)  # e.g. [1.0,  1.0,  1.0,  1.0]

        # Control loop / setpoint / arming
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("setpoint", [0.0, 0.0, 0.515])
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("pre_start_seconds", 1.0)

        run_dir = self.get_parameter("run_dir").get_parameter_value().string_value
        ckpt_rel = self.get_parameter("checkpoint_rel").get_parameter_value().string_value
        agent_yaml_rel = self.get_parameter("agent_yaml_rel").get_parameter_value().string_value
        self.round_decimals = int(self.get_parameter("round_decimals").value)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / self.rate_hz
        self.setpoint = np.asarray(self.get_parameter("setpoint").value, dtype=np.float32).reshape(3)
        self.armed = bool(self.get_parameter("arm_on_start").value)
        self.pre_start_steps = int(float(self.get_parameter("pre_start_seconds").value) * self.rate_hz)

        low_param = self.get_parameter("action_low").value
        high_param = self.get_parameter("action_high").value
        self.action_low = np.asarray(low_param, dtype=np.float32) if low_param is not None else None
        self.action_high = np.asarray(high_param, dtype=np.float32) if high_param is not None else None
        if (self.action_low is not None) and (self.action_high is not None):
            self.get_logger().info(f"[actions] scaling enabled: low={self.action_low}, high={self.action_high}")
        else:
            self.get_logger().info("[actions] scaling disabled (publishing unit actions)")

        ckpt_path = os.path.join(run_dir, ckpt_rel)
        agent_yaml_path = os.path.join(run_dir, agent_yaml_rel)

        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
        if not os.path.exists(agent_yaml_path):
            raise FileNotFoundError(f"Agent YAML not found: {agent_yaml_path}")

        self.get_logger().info(f"Loading skrl agent from:\n- ckpt: {ckpt_path}\n- cfg : {agent_yaml_path}")

        # ---------------- Build skrl Runner exactly like your compare script ----------------
        # 1) infer dims from checkpoint
        ckpt = torch.load(ckpt_path, map_location="cpu")
        pol = ckpt.get("policy", {}) or ckpt.get("agent", {}).get("policy", {})
        if not pol:
            raise RuntimeError("Couldn't find 'policy' state_dict in checkpoint")
        obs_dim = int(pol["net_container.0.weight"].shape[1])
        act_dim = int(pol["policy_layer.bias"].shape[0])

        # 2) tiny env -> wrap with skrl "gym" wrapper
        from skrl.envs.wrappers.torch import wrap_env as wrap_env_torch
        env = _TinyGymEnv(obs_dim, act_dim)
        venv = wrap_env_torch(env, wrapper="gym")

        # 3) load exact agent config saved at training time
        with open(agent_yaml_path, "r") as f:
            agent_cfg = yaml.safe_load(f)

        # speed/quiet tweaks
        agent_cfg.setdefault("agent", {}).setdefault("experiment", {})
        agent_cfg["agent"]["experiment"]["write_interval"] = 0
        agent_cfg["agent"]["experiment"]["checkpoint_interval"] = 0

        # Optional: force device to avoid CUDA/CPU surprises (uncomment to pin to CPU)
        # agent_cfg["agent"]["device"] = "cpu"

        from skrl.utils.runner.torch import Runner
        self._runner = Runner(venv, agent_cfg)
        self._runner.agent.load(ckpt_path)
        self._runner.agent.set_running_mode("eval")

        # Agent device (policy + preprocessor live here)
        self._agent_device = next(self._runner.agent.policy.parameters()).device
        self.get_logger().info(f"Agent device: {self._agent_device}")

        # ---------------- ROS I/O ----------------
        self.pub_cmd = self.create_publisher(ELRSCommand, "/ELRSCommand", 10)
        self.sub_mocap = self.create_subscription(
            MotionCaptureState, "/motion_capture_state", self._pose_cb, 10
        )
        self.timer = self.create_timer(self.dt, self._control_loop)

        # State
        self._obs = None  # 19-D rounded observation
        self._prev_actions = np.zeros(4, dtype=np.float32)  # keep in unit space unless you trained differently
        self._counter = 0
        self._pre_start_counter = 0

        self.get_logger().info("SKRLController initialised.")

    # ---------------- Build 19-D observation exactly like training ----------------
    def _pose_cb(self, msg: MotionCaptureState):
        # Position & orientation (wxyz)
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float32)
        wxyz = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ], dtype=np.float32)

        # World-frame linear velocity; body-frame angular velocity (already body in your msg)
        v_world = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=np.float32)
        w_body = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z], dtype=np.float32)

        # Rotation (world->body)
        quat_xyzw = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=np.float32)
        Rwb = R.from_quat(quat_xyzw).as_matrix().astype(np.float32)
        v_body = Rwb.T @ v_world

        # Desired relative position in body frame
        pos_err_world = self.setpoint - p
        pos_err_body = Rwb.T @ pos_err_world

        # Heading error (yaw)
        x, y, z, w = quat_xyzw
        curr_yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        desired_yaw = 0.0
        yaw_err = np.arctan2(np.sin(curr_yaw - desired_yaw), np.cos(curr_yaw - desired_yaw))
        heading_error = np.array([np.sin(yaw_err), np.cos(yaw_err)], dtype=np.float32)

        # Observation layout: [v_b(3), w_b(3), quat wxyz(4), pos_err_b(3), heading(2), prev_actions(4)] = 19
        obs = np.concatenate(
            [v_body, w_body, wxyz, pos_err_body, heading_error, self._prev_actions], dtype=np.float32
        )

        # Round like play.py BEFORE agent.act
        self._obs = np.round(obs, self.round_decimals).astype(np.float32)

    # ---------------- Timer: compute action via skrl mean_actions ----------------
    def _control_loop(self):
        msg = ELRSCommand()
        msg.armed = False
        msg.channel_0 = 0.0
        msg.channel_1 = 0.0
        msg.channel_2 = -1.0
        msg.channel_3 = 0.0

        # pre-start arming window
        if self.armed and self._pre_start_counter < self.pre_start_steps:
            msg.armed = True
            msg.channel_2 = -1.0
            self._pre_start_counter += 1
            self.pub_cmd.publish(msg)
            return

        # inference when armed & we have an observation
        if self.armed and self._obs is not None:
            obs_t = torch.from_numpy(self._obs).to(self._agent_device, dtype=torch.float32).unsqueeze(0)

            print(f"[obs] {self._obs.round(3)}")

            with torch.inference_mode():
                outputs = self._runner.agent.act(obs_t, timestep=0, timesteps=0)
                info = outputs[-1] if isinstance(outputs, (tuple, list)) else {}
                unit_action = info.get("mean_actions", outputs[0]).squeeze(0).detach().cpu().numpy().astype(np.float32)

            # Store prev actions in the SAME space as training (usually unit space)
            self._prev_actions = unit_action.copy()

            # Optional scaling to actuator space
            if (self.action_low is not None) and (self.action_high is not None):
                action = ((unit_action + 1.0) * 0.5) * (self.action_high - self.action_low) + self.action_low
            else:
                action = unit_action
            
            print(f"[action] {action.round(3)}")

            # Map to radio channels (adjust mapping if your mixer differs)
            # roll, pitch, throttle/force, yaw
            roll, pitch, force, yaw = (float(action[1]), float(action[2]), float(action[0]), float(action[3]))

            msg.armed = True
            msg.channel_0 = roll
            msg.channel_1 = pitch
            msg.channel_2 = force
            msg.channel_3 = yaw

            self._counter += 1

        self.pub_cmd.publish(msg)

    # ---------------- Shutdown ----------------
    def on_close(self):
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SKRLController()
    signal.signal(signal.SIGINT, lambda *_: node.on_close())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
