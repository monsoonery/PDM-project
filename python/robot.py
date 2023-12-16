import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
import random
import time

from urdfenvs.scene_examples.obstacles import *
from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv

class Robot:
    def __init__(self):
        robots = [GenericUrdfReacher(urdf="PDM-project/URDF/testRobot.urdf", mode="vel"),]
        self.env: UrdfEnv = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=True)
        
        self.no_action = np.array([0,0,0,0,0])
        vel0 = np.zeros(self.env.n())
        pos0 = np.array([0,0,0,0,np.pi/2])
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        
        self.goal = [0,0,0,0,0]
        self.prev_goal = [0,0,0,0,0]
        self.speed = 0.3
        self.history = []
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]

    def close_simulation(self):
        self.env.close()
        return self.history
    
    def angle_interval(self, q):
        while q <= -np.pi:
            q= q+2*np.pi
        
        while q >= np.pi:
            q = q-2*np.pi

        return q
    
    def move_x_y(self, goal, distance_goal,largest_distance):
        if (self.position_ob[0] >= goal[0]-0.05 and self.position_ob[0] <= goal[0]+0.05 and 
            self.position_ob[1] >= goal[1]-0.05 and self.position_ob[1] <= goal[1]+0.05): 
            # print("            2")
            x = 0 
            y = 0

        elif (self.position_ob[0] >= goal[0]-0.05 and self.position_ob[0] <= goal[0]+0.05 and not (self.position_ob[1] >= goal[1]+0.05 and self.position_ob[1] <= goal[1]+0.05)):
            # print("            3")
            x = 0
            y = self.speed*distance_goal[1]/largest_distance

        elif (self.position_ob[1] >= goal[1]-0.05 and self.position_ob[1] <= goal[1]+0.05 and not (self.position_ob[0] >= goal[0]+0.05 and self.position_ob[0] <= goal[0]+0.05)):
            # print("            4")
            x = self.speed*distance_goal[0]/largest_distance
            y = 0
            
        else:
            # print("            5")
            x = self.speed*distance_goal[0]/largest_distance
            y = self.speed*distance_goal[1]/largest_distance
        return x,y
    
    def move_to_goal(self, goal):
        x = 0
        y = 0
        q0 = 0
        q1 = 0
        q2 = 0
        
        distance_goal = []

        for item in range(2):
            distance_goal.append(goal[item]-self.prev_goal[item])

        largest_distance = max(map(abs,distance_goal)) 

        goal[2] = self.angle_interval(goal[2])

        temp = goal[2]-self.prev_goal[2]
        while temp < 0:
            temp = temp + 2*np.pi
        distance_goal.append(temp)

        for _ in range(10000):
            
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            self.position_ob[2] = self.angle_interval(self.position_ob[2])
            # print(self.position_ob)
            
            if (self.position_ob[0] >= goal[0]-0.05 and self.position_ob[0] <= goal[0]+0.05 and 
                self.position_ob[1] >= goal[1]-0.05 and self.position_ob[1] <= goal[1]+0.05 and
                self.position_ob[2] >= goal[2]-0.05 and self.position_ob[2] <= goal[2]+0.05):# and 
                #self.position_ob[3] >= goal[3]-0.05 and self.position_ob[3] <= goal[3]+0.05 and
                #self.position_ob[4] >= goal[4]-0.05 and self.position_ob[4] <= goal[4]+0.05):
                self.ob, *_ = self.env.step(self.no_action)
                self.prev_goal = goal
                # print("            1")
                break

            x,y = self.move_x_y(goal, distance_goal, largest_distance)

            if (self.position_ob[2] >= goal[2]-0.05 and self.position_ob[2] <= goal[2]+0.05):
                # print("            6")
                q0 = 0

            elif (distance_goal[2] >= np.pi):
                # print("            7")
                q0 = -self.speed

            elif (distance_goal[2] <= np.pi):
                # print("            8")
                q0 = self.speed
            
            self.ob, *_ = self.env.step(np.array([x,y,q0,q1,q2])) 
            self.history.append(self.ob)

if __name__ == "__main__":
    robot_0 = Robot()
    for i in range(10):
        num = random.uniform(-np.pi,np.pi)
        robot_0.move_to_goal(goal = [0, 0, num, 0 , np.pi/2])
        print(num)
        time.sleep(1)

    # robot_0.move_to_goal(goal = [-1, 1, 0, 0 , np.pi/2])
    # robot_0.move_to_goal(goal = [-2, -2, 1, 0 , np.pi/2])
    history = robot_0.close_simulation()