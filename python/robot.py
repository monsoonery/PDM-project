import gymnasium as gym
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import numpy as np

from urdfenvs.scene_examples.obstacles import *
from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv
import math

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
        self.obstacles = [sphere1, sphere2]
        for obstacle in self.obstacles:
            self.env.add_obstacle(obstacle)
        
        self.no_action = np.array([0,0,0,0,0])
        vel0 = np.zeros(self.env.n())
        pos0 = np.zeros(self.env.n())
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        
        self.goal = [0,0,0,0,0]
        self.prev_goal = [0,0,0,0,0]
        self.speed = 1
        self.history = []
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]

        self.l1 = 1.
        self.l2 = 1.
        self.l3 = 1.
        

    def close_simulation(self):
        self.env.close()
        return self.history
    
    def move_to_goal(self, goal):
        distance_goal = []
        for item in range(len(goal)):
            distance_goal.append(goal[item]-self.prev_goal[item])
        largest_distance = max(map(abs,distance_goal)) 
        
        for _ in range(10000):
            
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            
            if (self.position_ob[0] >= goal[0]-0.1 and self.position_ob[0] <= goal[0]+0.1 and self.position_ob[1] >= goal[1]-0.1 and self.position_ob[1] <= goal[1]+0.1):
                self.ob, *_ = self.env.step(self.no_action)
                self.prev_goal = goal
                break
            
            elif(self.position_ob[0] >= goal[0]-0.1 and self.position_ob[0] <= goal[0]+0.1 and not (self.position_ob[1] >= goal[1]-0.1 and self.position_ob[1] <= goal[1]+0.1)):
                self.ob, *_ = self.env.step(np.array([0,self.speed*distance_goal[1]/largest_distance,0,0,0]))

            elif(self.position_ob[1] >= goal[1]-0.1 and self.position_ob[1] <= goal[1]+0.1 and not (self.position_ob[0] >= goal[0]-0.1 and self.position_ob[0] <= goal[0]+0.1)):
                self.ob, *_ = self.env.step(np.array([self.speed*distance_goal[0]/largest_distance,0,0,0,0]))

            else: 
                self.ob, *_ = self.env.step(np.array([self.speed*distance_goal[0]/largest_distance,self.speed*distance_goal[1]/largest_distance,0,0,0]))

            self.history.append(self.ob)

    def is_in_collision(self, x, y, q1, q2, q3):
        """
        using a lite version of the 'union of spheres' method
        """
        # check x and y collision ("disk" only ("disk" = the robot's driving platform))
        x_robot = x
        y_robot = y
        z_robot = self.l1 / 2 # center of the disk
        r_robot = self.l1 / 2 # radius needed for union of spheres representation
        print(f"x_robot: {x_robot}, y_robot: {y_robot}, z_robot: {z_robot}, r_robot: {r_robot}")

        # check collision with each obstacle
        for obstacle in self.obstacles:
            print(f"Checking obstacle with name {obstacle.name()}")
            x_obst = obstacle.position()[0]
            y_obst = obstacle.position()[1]
            z_obst = obstacle.position()[2]
            r_obst = obstacle.radius() 

            distance = np.sqrt((x_robot - x_obst)**2 + (y_robot - y_obst)**2 + (z_robot - z_obst)**2)
            print(f"- Euclidean distance of robot to {obstacle.name()} = {distance}", )
            print(f"- Radius of robot disk + radius of {obstacle.name()} = {r_obst + r_robot}")

            if (distance < r_obst + r_robot):
                print("Collision!")
                #return True # NOTE TO SELF: DO NOT UNCOMMENT RETURN STATEMENTS UNTIL ENTIRE FUNCTION IS DONE
            else:
                print("No collision :)")
                #return False
        return 

        # check link 1 collision

        # check link 2 collision

        # check link 3 collision
        for l1 in np.linspace(0, self.l1, 4):
            for l2 in np.linspace(0, self.l2, 4):
                for l3 in np.linspace(0, self.l3, 4):
                    x_e = np.cos(q1) * (l2 * np.sin(q2) + l3 * np.cos(q2 + q3)) + d0 * np.cos(q1) + d0 * np.cos(q0)
                    y_e = np.sin(q1) * (l2 * np.sin(q2) + l3 * np.cos(q2 + q3)) + d0 * np.cos(q1) + d0 * np.sin(q0)
                    z_e = l2 * np.cos(q2) - l3 * np.sin(q2 + q3) + l1
                    for obstacle in self.obstacles:
                        print(obstacle.size())
                        x_obst = 1.
                        y_obst = 2.
                        z_obst = 3.
                        distance = np.sqrt((x_e - x_obst)**2 + (y_e - y_obst)**2 + (z_e - z_obst)**2)
                        #obstacle.size()
                        pass

if __name__ == "__main__":
    robot_0 = Robot()
    robot_0.is_in_collision(-2, -2, 0, 0, 0)
    robot_0.move_to_goal(goal = [-1, 1, 0, 0 ,0])
    robot_0.move_to_goal(goal = [-2, -2, 0, 0 ,0])
    robot_0.move_to_goal(goal = [0, 0, 0, 0 ,0])
    robot_0.move_to_goal(goal = [-2, -2, 0, 0 ,0])
    history = robot_0.close_simulation()