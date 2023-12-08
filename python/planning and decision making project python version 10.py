import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np


def run_mobile_reacher(n_steps=10000, render=False, goal=False, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="mobilePanda.urdf", mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )


    action = np.zeros(env.n())
    vel0 = np.zeros(env.n())
    pos0 = np.array([ 0., 0.,  0.,  0.,  0.,  0., 0., 0.,  1.8675,  0.,  0.02,  0.02])
    # pos0[3] = 0 

    ob = env.reset(pos=pos0, vel=vel0)
    print(f"Initial observation : {ob}")
    history = []
    print(env)
    for i in range(n_steps):
        action[3] = 0
        ob, *_ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)
