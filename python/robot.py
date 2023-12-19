import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np
import random
import time

from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv
import obstacles
import math

verbose = True # set to False to silence (most) print statements

sphere1Dict = {
    "type": "sphere",
    "geometry": {"position": [2.0, 2.0, 1.0], "radius": 1.0},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
sphere1 = SphereObstacle(name="Sphere1", content_dict=sphere1Dict)

sphere2Dict = {
    "type": "sphere",
    "geometry": {"position": [-2.0, -2.0, 1.0], "radius": 0.5},
    "rgba": [0.1, 0.8, 0.9, 1.0],
}
sphere2 = SphereObstacle(name="Sphere2", content_dict=sphere2Dict)

class Robot:
    def __init__(self):
        robots = [GenericUrdfReacher(urdf="URDF/testRobot.urdf", mode="vel"),]
        self.env: UrdfEnv = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=True)

        self.env.add_obstacle(obstacles.wall_obstacles)
        self.env.add_obstacle(obstacles.cylinder_obstacle)
        self.env.add_obstacle(obstacles.sphereObst1)
        self.obstacles = [sphere1, sphere2]
        for obstacle in self.obstacles:
            self.env.add_obstacle(obstacle)
        
        self.no_action = np.array([0,0,0,0,0])
        vel0 = np.array([0,0,0,0,0])
        pos0 = np.array([0,0,0,0,0])
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        
        self.goal = [0,0,0,0,0]
        self.prev_goal = [0,0,0,0,0]
        self.reached = [False, False, False, False, False]
        self.speed = 0.5
        self.interval = 0.03
        self.history = []
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]


        self.self_collision = False
        
        self.l1 = 0.6/2 # the mobile platform clips through the floor so not exactly 0.6?
        self.l2 = 0.7
        self.l3 = 0.6

    def close_simulation(self):
        self.env.close()
        return self.history
    
    def angle_interval(self, q):
        while q <= -np.pi: q = q+2*np.pi
        while q >= np.pi : q = q-2*np.pi

        return q
    
    def move_x_y(self, distance_goal, largest_distance):
        if (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and 
            self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval): 
            # print("            2")
            x = 0 
            y = 0

        elif (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and not (self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval)):
            # print("            3")
            if self.position_ob[1] >= self.goal[1]-self.interval: y = -self.speed
            elif self.position_ob[1] <= self.goal[1]+self.interval: y = self.speed
            else: y = 0
            x = 0

        elif (self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval and not (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval)):
            # print("            4")
            if self.position_ob[0] >= self.goal[0]-self.interval: x = -self.speed
            elif self.position_ob[0] <= self.goal[0]+self.interval: x = self.speed
            else: x = 0
            y = 0
            
        else:
            # print("            5")
            x = self.speed*distance_goal[0]/largest_distance
            y = self.speed*distance_goal[1]/largest_distance
        return x,y
    
    def move_angle(self, index, angle_between_goals, printen = False):
        if (self.position_ob[index] >= self.goal[index]-self.interval and self.position_ob[index] <= self.goal[index]+self.interval):
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
    
    def check_self_collision_goal(self):
        if (self.goal[3] > np.pi/2 or self.goal[3] < -np.pi/2): self.self_collision = True
        if (self.goal[4] > 2*np.pi/3 or self.goal[4] < -2*np.pi/3): self.self_collision = True
        # z = kinametics(self.goal)
        # if (z < 0): self.self_collision = True
        # print(1, self.self_collision)

    def check_self_collision_pose(self):
        if (self.position_ob[3] > np.pi/2 or self.position_ob[3] < -np.pi/2): self.self_collision = True
        if (self.position_ob[4] > 2*np.pi/3 or self.position_ob[4] < -2*np.pi/3): self.self_collision = True
        # z = kinametics(self.position_ob)
        # if (z < 0): self.self_collision = True
        # print(2, self.self_collision)

    def move_to_goal(self, goal):
        self.goal = goal
        self.reached = [False, False, False, False, False]
        distance_goal, largest_distance = self.prepare_distance()
        
        self.self_collision = False
        self.check_self_collision_goal()

        for _ in range(10000):
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            self.position_ob[2] = self.angle_interval(self.position_ob[2])
            self.position_ob[3] = self.angle_interval(self.position_ob[3])
            self.position_ob[4] = self.angle_interval(self.position_ob[4])
            
            self.check_self_collision_pose()
            
            if (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and 
                self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval and
                self.position_ob[2] >= self.goal[2]-self.interval and self.position_ob[2] <= self.goal[2]+self.interval and 
                self.position_ob[3] >= self.goal[3]-self.interval and self.position_ob[3] <= self.goal[3]+self.interval and
                self.position_ob[4] >= self.goal[4]-self.interval and self.position_ob[4] <= self.goal[4]+self.interval):
                self.ob, *_ = self.env.step(self.no_action)
                self.prev_goal = self.goal
                # print("            1")
                break

            if (self.self_collision == True):
                self.ob, *_ = self.env.step(self.no_action)
                break

            x,y = self.move_x_y(distance_goal, largest_distance)
            q0 = self.move_angle(2, distance_goal[2])
            q1 = self.move_angle(3, distance_goal[3])
            q2 = self.move_angle(4, distance_goal[4])
            
            self.ob, *_ = self.env.step(np.array([x,y,q0,q1,q2])) 
            self.history.append(self.ob)

    def intersects_any_obstacle(self, x, y, z, r):
        """
        Computes Euclidean distance between obstacles and robot segments, both represented 
        as spheres, and returns whether this part of the robot intersects with any of said
        obstacles (aka, would be in collision)
        INPUT: x, y, z, r -> coordinates and radius of a sphere on the robot
        OUTPUT: boolean -> True = This sphere is in collision with one or more objects
        """

        flag = False
        #if (verbose): print(f"x_robot: {x}, y_robot: {y}, z_robot: {z}, r_robot: {r}")

        for obstacle in self.obstacles:
            if (verbose): print(f"Checking obstacle with name {obstacle.name()}")

            x_obst = obstacle.position()[0]
            y_obst = obstacle.position()[1]
            z_obst = obstacle.position()[2]
            r_obst = obstacle.radius() 

            distance = np.sqrt((x - x_obst)**2 + (y - y_obst)**2 + (z - z_obst)**2)

            if (verbose): print(f"- Euclidean distance of robot sphere to {obstacle.name()} = {distance}", )
            if (verbose): print(f"- Radius of robot sphere + radius of {obstacle.name()} = {r_obst + r}")

            if (distance < r_obst + r):
                if (verbose): print(f'[!] Robot is in collision with {obstacle.name()}. Quitting this loop')
                flag = True
            else:
                if (verbose): print(f'[x] No collision detected with {obstacle.name()}. Continuing')
            
        return flag

    def is_in_collision(self, config):
        """
        Using the 'union of spheres' method to calculate collisions between robot and obstacles
        INPUT: config -> List containing configuration of the robot, format [x, y, q1, q2, q3] 
        OUTPUT: boolean -> True = this configuration is in collision
        """

        ##### Extract values
        x, y, q1, q2, q3 = config

        ##### Step 1: Check x and y collision (mobile base only)
        if (verbose): print("~~~~~~~~ Currently checking x and y collision ~~~~~~~~")
        x_robot = x
        y_robot = y
        z_robot = self.l1 / 2 # center of base
        r_robot = self.l1 / 2 # value taken from URDF, assuming the base is just 1 sphere (extremely simplified method)

        if self.intersects_any_obstacle(x_robot, y_robot, z_robot, r_robot):
            if (verbose): print("Collision!")
            return True
        else:
            if (verbose): print("No collision :)")

        ##### Step 2: Check first link collision (has dimension l2, beetje ongelukkige naam i know)
        if (verbose): print("~~~~~~~~ Currently checking collision with first link ~~~~~~~~")
        l2_division = 7 # in how many spheres will l2 be divided?

        for l2 in np.linspace(0, self.l2, l2_division):
            if (verbose): print(f"~~~~~~~ l2 = {l2}")
            x_l2 = l2 * np.cos(q1) * np.sin(q2) + x
            y_l2 = l2 * np.sin(q1) * np.sin(q2) + y
            z_l2 = l2 * np.cos(q2) + self.l1
            r_l2 = 0.1 # from URDF

            if self.intersects_any_obstacle(x_l2, y_l2, z_l2, r_l2):
                if (verbose): print("Collision!")
                return True
            else:
                if (verbose): print("No collision :)")

        ##### Step 3: Check second link collision (has dimension l3)
        if (verbose): print("~~~~~~~~ Currently checking collision with second link ~~~~~~~~")
        l3_division = 6

        for l3 in np.linspace(0, self.l3, l3_division):
            if (verbose): print(f"~~~~~~~ l3 = {l3}")
            x_l3 = np.cos(q1) * (self.l2 * np.sin(q2) + l3 * np.cos(q2 + q3)) + x
            y_l3 = np.sin(q1) * (self.l2 * np.sin(q2) + l3 * np.cos(q2 + q3)) + y
            z_l3 = self.l2 * np.cos(q2) - l3 * np.sin(q2 + q3) + self.l1
            r_l3 = 0.1 # from URDF

            if self.intersects_any_obstacle(x_l3, y_l3, z_l3, r_l3):
                if (verbose): print("Collision!")
                return True
            else:
                if (verbose): print("No collision :)")
        
        # If none of the above ever return True, then there must be no collisions
        return False

if __name__ == "__main__":
    robot_0 = Robot()
    robot_0.move_to_goal(goal = [-1, 1, 0, 0 ,0])

    next_configuration = [0.9, 0.9, 0, 0 ,0] 
    if (not robot_0.is_in_collision(next_configuration)):
        robot_0.move_to_goal(goal = next_configuration)
    else:
        print("Passed due to collision")
        
    robot_0.move_to_goal(goal = [0, 0, 0, 0 ,0])
    
    for i in range(15):
        x = random.uniform(-1.2,1.2)
        y = random.uniform(-1.2,1.2)
        q0 = random.uniform(-np.pi,np.pi)
        q1 = random.uniform(-np.pi,np.pi)
        q2 = random.uniform(-np.pi,np.pi)
        robot_0.move_to_goal(goal = [x, y, q0, q1, q2])
        time.sleep(1)

    history = robot_0.close_simulation()