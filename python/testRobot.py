import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

from urdfenvs.scene_examples.obstacles import *
from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv

def run_test_robot(n_steps=10000, render=False, goal = [0,0,0,0,0]):
    robots = [GenericUrdfReacher(urdf="PDM-project/URDF/testRobot.urdf", mode="vel"),]
    env: UrdfEnv = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=render)

    no_action = np.array([0,0,0,0,0])

    vel0 = np.zeros(env.n())
    pos0 = np.zeros(env.n())
    ob, _ = env.reset(pos=pos0, vel=vel0)
    speed = 0.1

    history = []
    for _ in range(n_steps):
        position_ob = ob["robot_0"]["joint_state"]["position"]
        largest_distance = max(map(abs,goal))
        # print (position_ob[0], position_ob[1])
        # print(position_ob[0] >= goal[0], position_ob[1] >= goal[1], position_ob[0] <= goal[0], position_ob[1] <= goal[1])

        if (position_ob[0] >= goal[0]-0.1 and position_ob[0] <= goal[0]+0.1 and position_ob[1] >= goal[1]-0.1 and position_ob[1] <= goal[1]+0.1):
            ob, *_ = env.step(no_action)
        
        elif(position_ob[0] >= goal[0]-0.1 and position_ob[0] <= goal[0]+0.1 and not (position_ob[1] >= goal[1]-0.1 and position_ob[1] <= goal[1]+0.1)):
            ob, *_ = env.step(np.array([0,speed*goal[1]/largest_distance,0,0,0]))

        elif(position_ob[1] >= goal[1]-0.1 and position_ob[1] <= goal[1]+0.1 and not (position_ob[0] >= goal[0]-0.1 and position_ob[0] <= goal[0]+0.1)):
            ob, *_ = env.step(np.array([speed*goal[0]/largest_distance,0,0,0,0]))

        else: 
            ob, *_ = env.step(np.array([speed*goal[0]/largest_distance,speed*goal[1]/largest_distance,0,0,0]))

        history.append(ob)
 
    env.close()
    return history


if __name__ == "__main__":
    run_test_robot(render=True, goal = [-1, 1, 0, 0 ,0])
