import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

from urdfenvs.scene_examples.obstacles import *
from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv

def run_point_robot(n_steps=10000, render=False):
    robots = [
        GenericUrdfReacher(urdf="PDM-project/URDF/testRobot.urdf", mode="vel"),
    ]
    env: UrdfEnv = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)

    action = np.array([0.6,0,0.5,0.05,0.2])

    vel0 = np.zeros(env.n())
    pos0 = np.zeros(env.n())
    ob = env.reset(pos=pos0, vel=vel0)

    
    history = []
    for _ in range(n_steps):
        ob, *_ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_point_robot(render=True)
