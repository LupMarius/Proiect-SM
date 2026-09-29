import gymnasium as gym
from gymnasium import spaces
import numpy as np
import os
import matplotlib.pyplot as plt

from stable_baselines3 import DQN, A2C, PPO
from stable_baselines3.common.callbacks import BaseCallback

class RobotEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 10}

    def __init__(self):
        super().__init__()

        self.render_mode = "human"

        self.action_space = spaces.Discrete(4)

        self.observation_space = spaces.Box(
            low = np.array([-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0], dtype = np.float32),
            high = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0], dtype = np.float32),
            dtype = np.float32
        )

        self.agent_pos = np.array([0, 0], dtype=np.float32)
        self.target_point = np.array([0, 0], dtype=np.float32)
        self.d_step = 0.1

        self.steps = 0
        self.max_steps = 5_000
        self.total_steps = 0

        self.old_distance = 2.0
        self.episode_reward = 0
    
    def get_distance(self):
        return np.sqrt((self.target_point[0] - self.agent_pos[0])**2 + (self.target_point[1] - self.agent_pos[1])**2)

    def get_delta_x(self):
        return self.target_point[0] - self.agent_pos[0]

    def get_delta_y(self):
        return self.target_point[1] - self.agent_pos[1]

    def step(self, action):

        terminated = False
        truncated = False
        self.steps += 1
        self.total_steps += 1

        reward = 0

        match action:
            case 0:
                self.agent_pos[0] += self.d_step
            case 1:
                self.agent_pos[0] -= self.d_step
            case 2:
                self.agent_pos[1] += self.d_step
            case 3:
                self.agent_pos[1] -= self.d_step

        reward += self.old_distance - self.get_distance()
        self.old_distance = self.get_distance()
        if self.get_distance() < 0.01: 
            reward += 50.0
            print("Reached finish")
            terminated = True

        observation = self.get_obs()

        if self.steps == self.max_steps: truncated = True

        self.episode_reward += reward

        info = {
            "pozitie robot": self.agent_pos,
            "comanda": action,
            "recompensa": self.episode_reward,
            "distanta": self.get_distance(),
            "tinta": self.target_point
                }
        # print(info)

        return observation, reward, terminated, truncated, info
    
    def reset(self, seed=None):
        super().reset(seed=seed)

        self.agent_pos = np.array([0.9, 0], dtype=np.float32)
        self.steps = 0
        if self.total_steps <= 400_000:
            self.target_point = [0.7 - 2 * (self.total_steps // 50_000) / 10, 0]
        else:
            self.target_point = [-0.9, 2 * ((self.total_steps + 200_000) // 100_000) / 10]
        
        self.old_distance = 2.0
        self.episode_reward = 0

        observation = self.get_obs()
        info = {}

        return observation, info

    def get_obs(self):
        d1 = np.abs(1.0 - self.agent_pos[0])
        d2 = np.abs(-1.0 + self.agent_pos[0])

        return np.array([d1, d2, self.agent_pos[0], self.agent_pos[1], self.target_point[0], self.target_point[1], self.get_delta_x(), self.get_delta_y()])
    
if __name__ == "__main__":
    env = RobotEnv()

    model_path = "neural_network"
    if os.path.exists(model_path + ".zip"):
        model = PPO.load(model_path, env = env)
    else:
        policy_kwargs = dict(net_arch=[1024, 1024, 1024])
        model = PPO(
            "MlpPolicy",
            env,
            policy_kwargs=policy_kwargs,
            verbose=1
        )
    model.learn(
        total_timesteps=1_000_000,
    )
    model.save("neural_network")

    obs, info = env.reset()

    while True:
        action, info = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        # print(info)

        if terminated or truncated:
            break