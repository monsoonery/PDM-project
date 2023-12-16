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
        vel0 = np.array([0,0,0,0,0])
        pos0 = np.array([0,0,0,0,0])
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        
        self.goal = [0,0,0,0,0]
        self.prev_goal = [0,0,0,0,0]
        self.reached = [False, False, False, False, False]
        self.speed = 0.3
        self.history = []
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]

    def close_simulation(self):
        self.env.close()
        return self.history
    
    def angle_interval(self, q):
        while q <= -np.pi: q = q+2*np.pi
        while q >= np.pi : q = q-2*np.pi

        return q
    
    def move_x_y(self, distance_goal, largest_distance):
        if (self.position_ob[0] >= self.goal[0]-0.05 and self.position_ob[0] <= self.goal[0]+0.05 and 
            self.position_ob[1] >= self.goal[1]-0.05 and self.position_ob[1] <= self.goal[1]+0.05): 
            # print("            2")
            x = 0 
            y = 0

        elif (self.position_ob[0] >= self.goal[0]-0.05 and self.position_ob[0] <= self.goal[0]+0.05 and not (self.position_ob[1] >= self.goal[1]-0.05 and self.position_ob[1] <= self.goal[1]+0.05)):
            # print("            3")
            if self.position_ob[1] >= self.goal[1]-0.05: y = -self.speed
            elif self.position_ob[1] <= self.goal[1]+0.05: y = self.speed
            else: y = 0
            x = 0

        elif (self.position_ob[1] >= self.goal[1]-0.05 and self.position_ob[1] <= self.goal[1]+0.05 and not (self.position_ob[0] >= self.goal[0]-0.05 and self.position_ob[0] <= self.goal[0]+0.05)):
            # print("            4")
            if self.position_ob[0] >= self.goal[0]-0.05: x = -self.speed
            elif self.position_ob[0] <= self.goal[0]+0.05: x = self.speed
            else: x = 0
            y = 0
            
        else:
            # print("            5")
            x = self.speed*distance_goal[0]/largest_distance
            y = self.speed*distance_goal[1]/largest_distance
        return x,y
    
    def move_angle(self, index, angle_between_goals, printen = False):
        if (self.position_ob[index] >= self.goal[index]-0.05 and self.position_ob[index] <= self.goal[index]+0.05):
            q = 0
            self.reached[index] = True
            # if printen == True: print("            6 = ", q)

        elif (angle_between_goals >= np.pi and not self.reached[index]):
            q = -self.speed
            # if printen == True: print("            7 = ", q)

        elif (angle_between_goals <= np.pi and not self.reached[index]):
            q = self.speed
            # if printen == True: print("            8 = ", q)
        
        elif (self.reached[index] and self.position_ob[index] >= self.goal[index]): 
            q = -self.speed
            # if printen == True: print("            9 = ", q)

        elif (self.reached[index] and self.position_ob[index] <= self.goal[index]): 
            q = self.speed
            # if printen == True: print("           10 = ", q)

        else: q = 0 # Should never happen
            

        return q
    
    def prepare_distance(self):
        distance_goal = []

        for item in range(2):
            distance_goal.append(self.goal[item]-self.prev_goal[item])

        largest_distance = max(map(abs,distance_goal)) 

        for i in range(2,5):
            self.goal[i] = self.angle_interval(self.goal[i])
            temp = self.goal[i]-self.prev_goal[i]
            while temp < 0:
                temp = temp + 2*np.pi
            distance_goal.append(temp)

        return distance_goal, largest_distance
    
    def move_to_goal(self, goal):
        self.goal = goal
        self.reached = [False, False, False, False, False]
        distance_goal, largest_distance = self.prepare_distance()

        for _ in range(10000):
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            self.position_ob[2] = self.angle_interval(self.position_ob[2])
            self.position_ob[3] = self.angle_interval(self.position_ob[3])
            self.position_ob[4] = self.angle_interval(self.position_ob[4])
            
            if (self.position_ob[0] >= self.goal[0]-0.05 and self.position_ob[0] <= self.goal[0]+0.05 and 
                self.position_ob[1] >= self.goal[1]-0.05 and self.position_ob[1] <= self.goal[1]+0.05 and
                self.position_ob[2] >= self.goal[2]-0.05 and self.position_ob[2] <= self.goal[2]+0.05 and 
                self.position_ob[3] >= self.goal[3]-0.05 and self.position_ob[3] <= self.goal[3]+0.05 and
                self.position_ob[4] >= self.goal[4]-0.05 and self.position_ob[4] <= self.goal[4]+0.05):
                self.ob, *_ = self.env.step(self.no_action)
                self.prev_goal = self.goal
                # print("            1")
                break

            x,y = self.move_x_y(distance_goal, largest_distance)
            q0 = self.move_angle(2, distance_goal[2])
            q1 = self.move_angle(3, distance_goal[3])
            q2 = self.move_angle(4, distance_goal[4])
            
            self.ob, *_ = self.env.step(np.array([x,y,q0,q1,q2])) 
            self.history.append(self.ob)

if __name__ == "__main__":
    robot_0 = Robot()
    for i in range(15):
        x = random.uniform(-1.2,1.2)
        y = random.uniform(-1.2,1.2)
        q0 = random.uniform(-np.pi,np.pi)
        q1 = random.uniform(-np.pi/3,np.pi/3)
        q2 = random.uniform(-np.pi/3,np.pi/3)
        robot_0.move_to_goal(goal = [x, y, q0, q1 , q2])
        time.sleep(1)

    history = robot_0.close_simulation()