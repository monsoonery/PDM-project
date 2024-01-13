import os
import numpy as np
import random
import time

from src import robot
from src import RRT_v3 as RRTstar
from src import environment

"""
This file first runs RRT* to find a path for the robot, then launches the simulation for the robot to follow
the found path. You can edit the variables below the comment "RRT* parameters" to explore the different features
of our RRT* implementation. Please do not edit any other variables.
"""

def print_time(name_step):
    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)
    print(name_step, " : ", current_time)

if __name__ == "__main__":
     # RRT* parameters
     random.seed(14)                # Random seed for sampling
     sample_radius = 5              # For a random configuration sample, if there are no existing nodes within this radius, it is too far away and discarded immediately
     neighbor_radius = 3            # Radius for finding neighbors that can be rewired to a given node
     n_expansions = 1000            # Number of (maximum) iterations to run the algorithm for
     stop_when_goal_reached = True  # If True, algorithm stops as soon as a valid solution is found instead of up to n_expansions
     animate_plot = True            # If True, plots RRT* graph nodes and edges in real-time
     verbose = False                # If True, prints extra information to the console for each iteration step
     file_directory = None          # Absolute file path to save live plot images (for generating animation). Set to None if you don't want to save these
     #file_directory = "D:\\My Files\\Documents\\Studie\\RO47005 Planning & Decision Making\\PDM-project\\plot_output" 



     # Mobile manipulator dimensions (from URDF)
     l1 = 0.4
     l2 = 0.7
     l3 = 0.6

     # Environment parameters
     room = {
          "width": [-10, 10],
          "length": [-10, 10],
          "height": 3,
          "margin_of_closeness_to_goal": 1.5
          }

     # Start configuration and goal xyz-coordinates
     initial_config = [-9.5, -9.5, 0, 0, (1/2)*np.pi]
     goal = [9, 9, 1]

     # Create RRT* algorithm object and run RRTstar
     rrt = RRTstar.RRTstar(l1, l2, l3, room)
     start_time_RRT = time.time()
     rrt.run(initial_config, goal, 
               sample_radius=sample_radius,
               neighbor_radius=neighbor_radius,
               n_expansions=n_expansions, 
               stop_when_goal_reached=stop_when_goal_reached, 
               animate_plot=animate_plot,
               file_directory=file_directory,
               verbose=verbose)

     # Calculate how long it took to run RRT
     end_time_RRT = time.time()
     elapsed_time_RRT = end_time_RRT - start_time_RRT
     print(f"This execution of RRT* took {elapsed_time_RRT/60} minutes.")

     # Retrieve the configurations that result in the shortest path
     result = rrt.get_shortest_path()
     print(f"Shortest path configs: {result}")

     # Store configurations of shortest path in same folder as live plot images
     if file_directory:
          filename = os.path.join(file_directory, "configs.txt")
          f = open(filename, 'a+' )
          f.write("Shortest path configs: " + repr(result) + '\n' )
          f.close()

     # Display results (the shortest path) in a final figure
     rrt.plot_results()

     # Create environment and robot object for simulation
     env = environment.Environment()
     coffeebot = robot.Robot(env.env, initial_config, l1, l2, l3)

     # Follow the planned path in the simulation
     if result:
          print_time("Start Moving")
          for config in result:
               print(f"current config: {config}")
               coffeebot.move_to_goal(goal = config)
          time.sleep(1)
          print_time("Done Moving")

     time.sleep(1)
     env.close_simulation()

