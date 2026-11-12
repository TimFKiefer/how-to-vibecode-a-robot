# SPDX-License-Identifier: MIT
"""The Brain interface and the two baselines. A brain knows of the body only the number and limits of the joints.

`act` gets the sensor dict (sensors.SENSOR_CHANNELS) and returns one target angle per servo in radians. The `seed` of
`reset` is a brain seed with no relation to the episode seed (episode.run_episode).
"""

from dataclasses import dataclass

import numpy as np

from robot2sim.randomise import rng


@dataclass(frozen=True)
class BodySpec:
    n_servos: int
    lower_rad: np.ndarray
    upper_rad: np.ndarray


class Brain:
    def reset(self, body: BodySpec, task: str, seed: int) -> None:
        self.body = body

    def act(self, obs: dict) -> np.ndarray:
        raise NotImplementedError

    def observe(self, obs: dict, action: np.ndarray, next_obs: dict, done: bool) -> None:
        pass

    def state_bytes(self) -> int:
        return 0

    def info(self) -> dict:
        return {}


class Hold(Brain):
    """The servo_pos of its first act, every step."""

    def reset(self, body: BodySpec, task: str, seed: int) -> None:
        self.body = body
        self.target = None

    def act(self, obs: dict) -> np.ndarray:
        if self.target is None:
            self.target = np.array(obs["servo_pos"], dtype=float)
        return self.target.copy()


class Random(Brain):
    """A new uniform target inside the limits every `hold_steps` steps, from the brain seed's "brain" stream."""

    def __init__(self, hold_steps: int = 10):
        self.hold_steps = hold_steps

    def reset(self, body: BodySpec, task: str, seed: int) -> None:
        self.body = body
        self.rng = rng(seed, "brain")
        self.steps = 0
        self.target = None

    def act(self, obs: dict) -> np.ndarray:
        if self.steps % self.hold_steps == 0:
            self.target = self.rng.uniform(self.body.lower_rad, self.body.upper_rad)
        self.steps += 1
        return self.target.copy()


BRAINS = {"hold": Hold, "random": Random}
NAMES = (*BRAINS, "scripted")


def make(name: str, body: str = "v0") -> Brain:
    if name not in NAMES:
        raise ValueError(f"unknown brain {name!r}; valid: {', '.join(NAMES)}")
    if name == "scripted":
        from robot2sim.scripted import Scripted  # scripted.py imports Brain from this module

        return Scripted(body)
    return BRAINS[name]()
