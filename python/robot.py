import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

from urdfenvs.scene_examples.obstacles import *
from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv

class Robot:
    def __init__(self):
        robots = [GenericUrdfReacher(urdf="PDM-project/URDF/testRobot.urdf", mode="vel"),]
        self.env: UrdfEnv = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=True)
        
        self.no_action = np.array([0,0,0,0,0])
        vel0 = np.zeros(self.env.n())
        pos0 = np.zeros(self.env.n())
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        
        self.goal = [0,0,0,0,0]
        self.speed = 0.1
        self.history = []
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]

    def close_simulation(self):
        self.env.close()
        return self.history
    
    def move_to_goal(self, goal):

        for _ in range(10000):
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            largest_distance = max(map(abs,goal))
            
            if (self.position_ob[0] >= goal[0]-0.1 and self.position_ob[0] <= goal[0]+0.1 and self.position_ob[1] >= goal[1]-0.1 and self.position_ob[1] <= goal[1]+0.1):
                self.ob, *_ = self.env.step(self.no_action)
                break
            
            elif(self.position_ob[0] >= goal[0]-0.1 and self.position_ob[0] <= goal[0]+0.1 and not (self.position_ob[1] >= goal[1]-0.1 and self.position_ob[1] <= goal[1]+0.1)):
                self.ob, *_ = self.env.step(np.array([0,self.speed*goal[1]/largest_distance,0,0,0]))

            elif(self.position_ob[1] >= goal[1]-0.1 and self.position_ob[1] <= goal[1]+0.1 and not (self.position_ob[0] >= goal[0]-0.1 and self.position_ob[0] <= goal[0]+0.1)):
                self.ob, *_ = self.env.step(np.array([self.speed*goal[0]/largest_distance,0,0,0,0]))

            else: 
                self.ob, *_ = self.env.step(np.array([self.speed*goal[0]/largest_distance,self.speed*goal[1]/largest_distance,0,0,0]))

            self.history.append(self.ob)

if __name__ == "__main__":
    robot_0 = Robot()
    robot_0.move_to_goal(goal = [-1, 1, 0, 0 ,0])
    robot_0.move_to_goal(goal = [-2, -2, 0, 0 ,0])
    history = robot_0.close_simulation()