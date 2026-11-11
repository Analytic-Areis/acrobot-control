"""
Train the SAC swing-up policy.

    ../phase2/.venv/bin/python train.py [total_steps]             original policy (1 kg links only)
    ../phase2/.venv/bin/python train.py [total_steps] --robust    robust policy

The robust policy differs in exactly two ways:
  1. every practice swing uses new random link masses, each in [0.7, 1.3] kg;
  2. it also observes the missing energy (E_top - E)/E_top.
Network, reward, catch zone and LQR are unchanged.

Saves policies/sac_swingup[_robust].zip and training_log[_robust].json and prints the
share of episodes that reach the LQR catch zone every 10 000 steps.
"""

import json
import os
import sys
import time

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env

from acrobot_env import AcrobotSwingUpEnv

HERE = os.path.dirname(os.path.abspath(__file__))
ROBUST = "--robust" in sys.argv
_num = [a for a in sys.argv[1:] if a.isdigit()]
TOTAL = int(_num[0]) if _num else 300_000
TAG = "_robust" if ROBUST else ""
ENV_KW = dict(mass_range=(0.7, 1.3), energy_obs=True) if ROBUST else {}


class Progress(BaseCallback):
    def __init__(self):
        super().__init__()
        self.done, self.caught, self.t0 = 0, 0, time.time()
        self.curve, self.total_episodes = [], 0

    def _on_step(self):
        for info, d in zip(self.locals["infos"], self.locals["dones"]):
            if d:
                self.done += 1
                self.total_episodes += 1
                self.caught += int(info.get("caught", False))
        if self.num_timesteps % 10_000 < self.training_env.num_envs:
            rate = self.caught / max(self.done, 1)
            print(f"{self.num_timesteps:7d} steps | episodes {self.done:4d} | reached catch zone "
                  f"{100 * rate:5.1f}% | {time.time() - self.t0:5.0f} s", flush=True)
            self.curve.append([int(self.num_timesteps), round(100 * rate, 1)])
            self.done = self.caught = 0
        return True


if __name__ == "__main__":
    env = make_vec_env(AcrobotSwingUpEnv, n_envs=4, seed=0, env_kwargs=ENV_KW)
    model = SAC("MlpPolicy", env, learning_rate=3e-4, buffer_size=300_000, batch_size=256,
                gamma=0.99, tau=0.005, train_freq=1, gradient_steps=1, learning_starts=5_000,
                policy_kwargs=dict(net_arch=[256, 256]), seed=0, verbose=0, device="cpu")
    cb = Progress()
    t0 = time.time()
    model.learn(total_timesteps=TOTAL, callback=cb)
    with open(os.path.join(HERE, f"training_log{TAG}.json"), "w") as fh:
        json.dump(dict(steps=TOTAL, episodes=cb.total_episodes, minutes=round((time.time() - t0) / 60, 1),
                       curve=cb.curve), fh)
    os.makedirs(os.path.join(HERE, "policies"), exist_ok=True)
    model.save(os.path.join(HERE, "policies", f"sac_swingup{TAG}"))
    print(f"saved policies/sac_swingup{TAG}.zip")
