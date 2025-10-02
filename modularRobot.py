import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'morphlib-main')))

from stable_baselines3 import PPO
from stable_baselines3 import A2C
from stable_baselines3 import SAC
from stable_baselines3 import TD3
from sb3_contrib import RecurrentPPO
from mujoco.glfw import glfw
from mujoco import viewer
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, DummyVecEnv
# import mediapy

import gymnasium as gym
from gymnasium.envs.registration import register
from gymnasium import spaces
import numpy as np
import mujoco
import re
import time
from stable_baselines3.common.vec_env import VecMonitor
import pandas as pd
import matplotlib.pyplot as plt
import argparse

from morphlib.bodies.robogen.gecko import gecko
from morphlib.bodies.robogen.gecko_long import gecko_long
from morphlib.bodies.robogen.snake import snake
# from morphlib.bodies.robogen.gecko_bookmark import gecko_bookmark
# from morphlib.bodies.robogen.gecko_halfbookmark import gecko_halfbookmark

from morphlib.bodies.robogen.modules.active_joint import ActiveJoint
# from morphlib.brain._make_cpg_network_structure_neighbor import active_hinges_to_cpg_network_structure_neighbor
from morphlib.tools.build_file import build_mjcf
from pyrr import Quaternion
from morphlib.terrains.mujoco_plane import mujoco_plane
# from morphlib.terrains.hill_and_valleys import hill_and_valleys

from morphlib.tools.mj_default_sim_setup import mujoco_setup_sim
from functools import partial
import torch


class CustomMujocoEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(self, body_builder, render_mode=None, seed=None, generate_terrain=False):
        """
        Custom MuJoCo environment.

        Args:
            body_builder: Function that returns a body object
            render_mode (str): Either "human" or "rgb_array" for rendering options.
            seed (int): Random seed for reproducibility
            generate_terrain (bool): Whether to generate terrain
        """
        super().__init__()
        body = body_builder()
        # body = gecko()
        # hill_and_valleys_seeded = partial(hill_and_valleys, seed=seed, generate_terrain=generate_terrain)
        xml = build_mjcf(bodies=[body], body_poss=[[0, 0, 0.1]], body_oris=[Quaternion()], terrain_builder=mujoco_plane,
                         sim_setup=mujoco_setup_sim, ts=0.001)

        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)

        # Rendering attributes
        self.render_mode = render_mode
        self.viewer = False

        self.frame_skip = 10
        self.dt = self.model.opt.timestep * self.frame_skip

        self._init_joint_tracking(xml)

        self.num_hinges = len(self.data.ctrl)
        # print(f"Number of hinges: {self.num_hinges}")
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.num_hinges,), dtype=np.float32
        )


        num_obs = self._calculate_obs_size()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(num_obs,), dtype=np.float32
        )
        # print(f"Observation space size: {num_obs}")
        self.prev_core_pos = None
        self.episode_length = 0
        self.max_episode_length = 1000
        self.action_scale = 0.5

    def _init_joint_tracking(self, xml):

        pattern = r'[^"]*jointy_0[^"]*'
        matches = re.findall(pattern, xml)
        joints_ys = []
        for string in matches:
            joints_y = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{string[:-1]}{i}") for i in range(3)
                        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{string[:-1]}{i}") != -1]
            if joints_y:
                joints_ys.append(joints_y)
        self.joints_ys = np.array(joints_ys, dtype=object) if joints_ys else np.array([])

        pattern = r'[^"]*jointx_0[^"]*'
        matches = re.findall(pattern, xml)
        joints_xs = []
        for string in matches:
            joints_x = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{string[:-1]}{i}") for i in range(3)
                        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{string[:-1]}{i}") != -1]
            if joints_x:
                joints_xs.append(joints_x)
        self.joints_xs = np.array(joints_xs, dtype=object) if joints_xs else np.array([])

    def _calculate_obs_size(self):
        base_size = 3 + 4 + self.num_hinges + self.num_hinges
        joints_y_size = sum(len(joints) for joints in self.joints_ys) * 2 if len(self.joints_ys) > 0 else 0
        joints_x_size = sum(len(joints) for joints in self.joints_xs) * 2 if len(self.joints_xs) > 0 else 0

        return base_size + joints_y_size + joints_x_size

    def step(self, action):
        """Step the simulation forward."""
        # Scale and clip action
        action = np.clip(action * self.action_scale, -1.0, 1.0)

        # Apply action and simulate
        for _ in range(self.frame_skip):
            self.data.ctrl[:] = action
            mujoco.mj_step(self.model, self.data)


        observation = self._get_obs()
        reward = self._compute_reward()

        self.episode_length += 1

        terminated = self._check_termination()
        truncated = self.episode_length >= self.max_episode_length
        info = {
            'episode_length': self.episode_length,
            'core_pos': self.data.geom_xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "core")].copy()
        }

        return observation.astype(np.float32), float(reward), terminated, truncated, info

    def reset(self, seed=None, options=None):
        """Reset the environment to its initial state."""
        super().reset(seed=seed)

        # Reset the simulation
        mujoco.mj_resetData(self.model, self.data)
        if seed is not None:
            np.random.seed(seed)
        noise_scale = 0.1
        for i in range(self.model.nq):
            self.data.qpos[i] += np.random.uniform(-noise_scale, noise_scale)

        # Set the initial state
        if len(self.joints_ys) > 0:
            for joints_y in self.joints_ys:
                if len(joints_y) > 0:
                    self.data.qpos[np.array(joints_y) + 6] = np.linspace(0.4, 0.8, len(joints_y))
        mujoco.mj_forward(self.model, self.data)
        core_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "core")
        self.prev_core_pos = self.data.geom_xpos[core_id].copy()
        self.episode_length = 0

        observation = self._get_obs()
        return observation.astype(np.float32), {}

    def _get_obs(self):
        """Collect observations from the simulation."""
        # Get core position and orientation
        core_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "core")
        core_pos = self.data.geom_xpos[core_id].copy()
        core_matrix = self.data.geom_xmat[core_id].copy()
        core_quat = np.array([0, 0, 0, 0], dtype=np.float64)
        mujoco.mju_mat2Quat(core_quat, core_matrix)

        # Get joint positions and velocities
        hinge_pos = self.data.qpos[7:7 + self.num_hinges]
        hinge_vel = self.data.qvel[6:6 + self.num_hinges]
        if len(self.joints_ys) > 0:
            joint_pos_y = []
            joint_vel_y = []
            for joints_y in self.joints_ys:
                if len(joints_y) > 0:
                    joint_pos_y.extend(self.data.qpos[np.array(joints_y) + 6])
                    joint_vel_y.extend(self.data.qvel[np.array(joints_y) + 5])
            joint_pos_y = np.array(joint_pos_y)
            joint_vel_y = np.array(joint_vel_y)
        else:
            joint_pos_y = np.array([])
            joint_vel_y = np.array([])

        if len(self.joints_xs) > 0:
            joint_pos_x = []
            joint_vel_x = []
            for joints_x in self.joints_xs:
                if len(joints_x) > 0:
                    joint_pos_x.extend(self.data.qpos[np.array(joints_x) + 6])
                    joint_vel_x.extend(self.data.qvel[np.array(joints_x) + 5])
            joint_pos_x = np.array(joint_pos_x)
            joint_vel_x = np.array(joint_vel_x)
        else:
            joint_pos_x = np.array([])
            joint_vel_x = np.array([])
        obs_parts = [core_pos, core_quat, hinge_pos, hinge_vel]
        if len(joint_pos_y) > 0:
            obs_parts.extend([joint_pos_y, joint_vel_y])
        if len(joint_pos_x) > 0:
            obs_parts.extend([joint_pos_x, joint_vel_x])

        return np.concatenate(obs_parts)

    def _compute_reward(self):
        """Compute the reward for the current step."""

        core_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "core")
        core_pos = self.data.geom_xpos[core_id].copy()
        if self.prev_core_pos is not None:
            core_vel = (core_pos - self.prev_core_pos) / self.dt
        else:
            core_vel = np.zeros(3)

        self.prev_core_pos = core_pos.copy()


        forward_reward = core_vel[0]
        lateral_penalty = -0.1 * abs(core_vel[1])
        stability_reward = -0.01 * abs(core_vel[2])
        action_penalty = -0.001 * np.sum(np.square(self.data.ctrl))
        fall_penalty = -1.0 if core_pos[2] < 0.05 else 0.0

        total_reward = forward_reward + lateral_penalty + stability_reward + action_penalty + fall_penalty

        return total_reward

    def _check_termination(self):
        """Check if the episode is terminated."""

        core_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "core")
        core_pos = self.data.geom_xpos[core_id]
        if core_pos[2] < 0.02:
            return True


        cube_xmat = self.data.geom_xmat[core_id].reshape(3, 3)
        local_z_world = cube_xmat[:, 2]
        world_z = np.array([0, 0, 1])
        if np.dot(local_z_world, world_z) < 0.3:
            return True

        return False

    def render(self, camera="track"):
        """Render the simulation."""
        if self.render_mode == "human":
            pass
        elif self.render_mode == "rgb_array":
            if not self.viewer:
                self._init_rendering()

            self.renderer.update_scene(self.data, scene_option=self.scene_option, camera=camera)
            pixels = self.renderer.render()
            return pixels

    def close(self):
        """Clean up resources."""
        pass

    def _init_rendering(self):
        """Initialize rendering resources."""
        self.renderer = mujoco.Renderer(self.model)
        self.scene_option = mujoco.MjvOption()
        self.viewer = True



morphologies = {
    "Snake": snake,
    "Gecko": gecko,
    "LongGecko": gecko_long,
}
for name, builder in morphologies.items():
    register(
        id=f"Custom{name}-v0",
        entry_point="__main__:CustomMujocoEnv",
        kwargs={"body_builder": builder, "render_mode": "rgb_array"}
    )


# Create multiple parallel environments
def make_env(env_id, seed, rank=0):
    """Create a single environment with proper seeding"""

    def _init():
        env = gym.make(env_id)
        env.reset(seed=seed + rank)
        return env

    return _init


def test_policy(model_path, env_id):
    """Test a trained policy and generate videos"""
    model = PPO.load(model_path)

    env = gym.make(env_id, render_mode="rgb_array")
    frames = []

    obs, _ = env.reset()
    done = False
    step_count = 0

    while not done and step_count < 1000:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)

        if step_count % 5 == 0:  # Record every 5th frame
            pixels = env.render()
            frames.append(pixels)

        step_count += 1

        if done or truncated:
            break

    env.close()




if __name__ == '__main__':

    algos = {
        "PPO": (PPO, "MlpPolicy"),
        "A2C": (A2C, "MlpPolicy"),
        "SAC": (SAC, "MlpPolicy"),
        "TD3": (TD3, "MlpPolicy"),
        "RecurrentPPO": (RecurrentPPO, "MlpLstmPolicy"),
    }

    os.makedirs("./logs", exist_ok=True)
    os.makedirs("./models", exist_ok=True)
    os.makedirs("./eval", exist_ok=True)
    parser = argparse.ArgumentParser()


    parser.add_argument('--algo', type=str, required=True, choices=list(algos.keys()))
    parser.add_argument('--morph', type=str, required=True, choices=list(morphologies.keys()))
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--n_envs', type=int, default=10) 
    parser.add_argument('--total_steps', type=int, default=5_000_000)
    parser.add_argument('--device', type=str, default="auto")
    parser.add_argument('--eval_freq', type=int, default=50000)
    parser.add_argument('--save_freq', type=int, default=100000)
    args = parser.parse_args()

    AlgoClass, policy = algos[args.algo]


    env_id = f"Custom{args.morph}-v0"


    envs = SubprocVecEnv([make_env(env_id, args.seed, rank=i) for i in range(args.n_envs)])
    envs = VecNormalize(envs, norm_obs=True, norm_reward=True, clip_obs=10.0)

    envs = VecMonitor(envs, filename=f"./logs/{args.morph}_{args.algo}_seed{args.seed}.csv")

    eval_callback = EvalCallback(
        envs,
        best_model_save_path=f"./models/best_{args.algo}_{args.morph}_seed{args.seed}",
        log_path=f"./logs/eval_{args.morph}_{args.algo}_seed{args.seed}",
        eval_freq=args.eval_freq,
        deterministic=True,
        render=False
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq,
        save_path=f"./models/checkpoints_{args.algo}_{args.morph}_seed{args.seed}/",
        name_prefix="rl_model"
    )

    if args.algo == "PPO":
        model = AlgoClass(
            policy, envs, verbose=1, seed=args.seed, device=args.device,
            learning_rate=3e-4, n_steps=2048, batch_size=64, n_epochs=10,
            gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.01
        )
    elif args.algo == "A2C":
        model = AlgoClass(
            policy, envs, verbose=1, seed=args.seed, device=args.device,
            learning_rate=3e-4, n_steps=10, gamma=0.99, gae_lambda=0.95, ent_coef=0.01
        )
    elif args.algo == "SAC":
        model = AlgoClass(
            policy, envs, verbose=1, seed=args.seed, device=args.device,
            learning_rate=3e-4, buffer_size=1000000, batch_size=256,
            tau=0.005, gamma=0.99, ent_coef='auto'
        )
    elif args.algo == "TD3":
        model = AlgoClass(
            policy, envs, verbose=1, seed=args.seed, device=args.device,
            learning_rate=3e-4, buffer_size=1000000, batch_size=256,
            tau=0.005, gamma=0.99, policy_delay=2
        )
    elif args.algo == "RecurrentPPO":
        model = AlgoClass(
            policy, envs, verbose=1, seed=args.seed, device=args.device,
            learning_rate=3e-4, n_steps=128, batch_size=32, n_epochs=10,
            gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.01
        )

    model.learn(
        total_timesteps=args.total_steps,
        callback=[eval_callback, checkpoint_callback]
    )


    model.save(f"./models/{args.algo}_{args.morph}_seed{args.seed}")
    if hasattr(model, "save_replay_buffer"):
        model.save_replay_buffer(f"./models/{args.algo}_{args.morph}_seed{args.seed}_buffer")

    envs.save(f"./models/{args.algo}_{args.morph}_seed{args.seed}_vecnormalize.pkl")

    eval_env = gym.make(env_id, render_mode=None)
    episode_rewards = []
    episode_lengths = []

    for i in range(50):
        obs, _ = eval_env.reset(seed=args.seed + 10000 + i)
        done = False
        truncated = False
        total_reward = 0.0
        episode_length = 0
        while not (done or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = eval_env.step(action)
            total_reward += reward
            episode_length += 1

        episode_rewards.append(total_reward)
        episode_lengths.append(episode_length)

    eval_env.close()

    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    mean_length = np.mean(episode_lengths)
    std_length = np.std(episode_lengths)
    with open(f"./eval/{args.morph}_{args.algo}_seed{args.seed}_eval.txt", "w") as f:
        f.write(f"mean_eval_reward:{mean_reward:.4f}\n")
        f.write(f"std_eval_reward: {std_reward:.4f}\n")
        f.write(f"mean_eval_length:{mean_length:.2f}\n")
        f.write(f"std_eval_length: {std_length:.2f}\n")
        f.write(f"min_reward: {min(episode_rewards):.4f}\n")
        f.write(f"max_reward:{max(episode_rewards):.4f}\n")


    print(f"Mean reward: {mean_reward:.4f} ± {std_reward:.4f}")
    print(f"Mean episode length: {mean_length:.2f} ± {std_length:.2f}")


    envs.close()