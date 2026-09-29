from flask import Flask, request, jsonify
import numpy as np

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import os
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
import threading

app = Flask(__name__)

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

        self.old_distance = 2.0
        self.episode_reward = 0

        observation = self.get_obs()
        info = {}

        return observation, info

    def get_obs(self):
        d1 = np.abs(1.0 - self.agent_pos[0])
        d2 = np.abs(-1.0 + self.agent_pos[0])

        return np.array([d1, d2, self.agent_pos[0], self.agent_pos[1], self.target_point[0], self.target_point[1], self.get_delta_x(), self.get_delta_y()])
    
    def set_target(self, point):
        self.target_point = point

env = RobotEnv()

model_path = "neural_network"
model = PPO.load(model_path, env = env)
current_target = {
    "set": False,
    "x": 0.0,
    "y": 0.0
}
target_lock = threading.Lock()

def build_observation(sensor_data):
    required_fields = ["d1", "d2", "x", "y"]

    for field in required_fields:
        if field not in sensor_data:
            raise ValueError(f"Lipsește câmpul: {field}")

    with target_lock:
        if not current_target["set"]:
            return None

        target_x = current_target["x"]
        target_y = current_target["y"]

    d1 = float(sensor_data["d1"])
    d2 = float(sensor_data["d2"])
    x = float(sensor_data["x"])
    y = float(sensor_data["y"])

    env.set_target(np.array([target_x, target_y], dtype=np.float32))

    delta_x = target_x - x
    delta_y = target_y - y

    obs = np.array([
        d1,
        d2,
        x,
        y,
        target_x,
        target_y,
        delta_x,
        delta_y
    ], dtype=np.float32)

    return obs

def action_to_command(action):
    if action == 0:
        return {"action": 0, "command": "move_right", "vx": 1.0, "vy": 0.0}

    elif action == 1:
        return {"action": 1, "command": "move_left", "vx": -1.0, "vy": 0.0}

    elif action == 2:
        return {"action": 2, "command": "move_up", "vx": 0.0, "vy": 1.0}

    elif action == 3:
        return {"action": 3, "command": "move_down", "vx": 0.0, "vy": -1.0}

    return {"action": -1, "command": "stop", "vx": 0.0, "vy": 0.0}

@app.route("/sensor", methods=["POST"])
def receive_sensor_data():
    sensor_data = request.get_json()

    if sensor_data is None:
        return jsonify({"error": "JSON invalid"}), 400

    try:
        obs = build_observation(sensor_data)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400

    if obs is None:
        return jsonify({
            "error": "Target point-ul nu a fost trimis încă",
            "hint": "Trimite mai întâi POST /target"
        }), 409

    action, _ = model.predict(obs, deterministic=True)
    action = int(action)

    command = action_to_command(action)
    command["obs"] = obs.tolist()

    return jsonify(command)

@app.route('/target', methods=["POST"])
def receive_target():
    data = request.get_json()

    if data is None:
        return jsonify({"error": "JSON invalid"}), 400

    if "target_x" not in data or "target_y" not in data:
        return jsonify({"error": "Lipsesc target_x sau target_y"}), 400

    with target_lock:
        current_target["x"] = float(data["target_x"])
        current_target["y"] = float(data["target_y"])
        current_target["set"] = True

    return jsonify({
        "status": "target_set",
        "target_x": current_target["x"],
        "target_y": current_target["y"]
    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)