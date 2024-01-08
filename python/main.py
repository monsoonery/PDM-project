import time
import matplotlib.pyplot as plt
import random
import numpy as np

import robot
import RRT_v3 as RRTstar
import environment

def print_time(name_step):
    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)
    print(name_step, " : ", current_time)


if __name__ == "__main__":
     # Manipulator parameters
     l1 = 0.4
     l2 = 0.7
     l3 = 0.6

     # Environment parameters
     room = {
          "width": [-10, 10],
          "length": [-10, 10],
          "height": 4,
          "margin_of_closeness_to_goal": 1
          }

     # RRT parameters
     sample_radius = 4              # For a random configuration sample, if there are no existing nodes within this radius, it is too far away and discarded immediately
     neighbor_radius = 3            # Radius for finding neighbors that can be rewired to a given node
     n_expansions = 10000           # Number of iterations to run the algorithm for
     also_run_normal_RRT = False    # If True, both RRT and RRT* algorithms will be executed 
     stop_when_goal_reached = False # If True, algorithm stops as soon as a valid solution is found instead of up to n_expansions
     animate_plot = True            # If True, plots RRT* graph nodes and edges in real-time
     verbose = True                 # If True, prints the iterations (i = 1, i = 2 etc.) to the terminal to keep track of progress
     #file_directory = None         # Enter directory to save plot images (for generating animation). Set to None if you don't want to save these
     file_directory = "D:\\My Files\\Documents\\Studie\\RO47005 Planning & Decision Making\\PDM-project\\plot_images" 

     # Start configuration and goal xyz-coordinates
     initial_config = [-9.5, -9.5, 0, 0, (1/2)*np.pi]
     goal = [9, 9, 1]



     # Create RRT algorithm object and run RRTstar
     rrt = RRTstar.RRTstar(l1, l2, l3, room)
     start_time_RRT = time.time()
     rrt.run(initial_config, goal, 
               sample_radius=sample_radius,
               neighbor_radius=neighbor_radius,
               n_expansions=n_expansions, 
               stop_when_goal_reached=stop_when_goal_reached, 
               also_run_normal_RRT=also_run_normal_RRT,
               animate_plot=animate_plot,
               file_directory=file_directory,
               verbose=verbose)

     # Calculate how long it took to run RRT
     end_time_RRT = time.time()
     elapsed_time_RRT = end_time_RRT - start_time_RRT
     print(f"This execution of RRT took {elapsed_time_RRT/60} minutes.")

     # Retrieve the configurations that result in the shortest path
     result = rrt.get_shortest_path()
     print(f"Shortest path configs: {result}")

     # Display results (the shortest path) in a final figure
     rrt.plot_results(also_run_normal_RRT)

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

     time.sleep(10)
     env.close_simulation()
