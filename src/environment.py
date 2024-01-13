
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.scene_examples.goal import *
from urdfenvs.urdf_common.urdf_env import UrdfEnv
import gymnasium as gym

from src import obstacles

show_collision_spheres = False
show_obstacles = True

class Environment: 
    def __init__(self):
        robots = [GenericUrdfReacher(urdf="URDF/testRobot.urdf", mode="vel"),]
        self.env: UrdfEnv = gym.make("urdf-env-v0", dt=0.01, robots=robots, render=True)

        # Initialize list of obstacles defined in obstacles.py
        self.collision_obstacles = obstacles.collision_obstacles
        self.decorative_obstacles = obstacles.decorative_obstacles

        
        if show_collision_spheres:
            for obstacle in self.collision_obstacles:
                self.env.add_obstacle(obstacle)
        if show_obstacles:
            for obstacle in self.decorative_obstacles:
                self.env.add_obstacle(obstacle)
    
    def close_simulation(self):
        self.env.close()
