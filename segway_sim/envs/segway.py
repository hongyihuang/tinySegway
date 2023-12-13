import numpy as np

from gymnasium import utils
from .mujoco_env import MujocoEnv
from gymnasium.spaces import Box


DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": 0,
    "distance": 1.5,
}


class SegwayEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 250
    }

    def __init__(self, max_ep_len = 1000, **kwargs):
        utils.EzPickle.__init__(self, **kwargs)
        self.step_count = 0
        self.max_ep_len = max_ep_len
        observation_space = Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64)
        MujocoEnv.__init__(
            self,
            "segway.xml",
            2,
            observation_space=observation_space,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
        )

    def step(self, a):
        self.do_simulation(a, self.frame_skip)
        ob = self._get_obs()
        if self.data.qpos[3] > 0.6:
            reward = np.array(self.data.qpos[3])
        else:
            reward = np.array(-1)
            truncated = True
            self.step_count = 0
            return ob, reward, False, truncated, {}
            
        truncated = self.step_count > self.max_ep_len
        if self.step_count > self.max_ep_len:
            truncated = True
            self.step_count = 0
        else:
            truncated = False
            self.step_count += 1
        return ob, reward, False, truncated, {}

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq, low=-0.01, high=0.01
        )
        qvel = self.init_qvel + self.np_random.uniform(
            size=self.model.nv, low=-0.01, high=0.01
        )
        self.set_state(qpos, qvel)
        return self._get_obs()

    def get_all_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel, self.data.sensordata]).ravel()

    def _get_obs(self):
        return np.array(self.data.sensordata)