import time
import matplotlib.pyplot as plt
import random
import numpy as np

import robot
import RRT_v3 as RRTstar
import environment

if __name__ == "__main__":
     # manipulator parameters
     l1 = 0.4
     l2 = 0.7
     l3 = 0.6

     # environment parameters
     room = {
          "width": [-10, 10],
          "length": [-10, 10],
          "height": 8,
          "margin_of_closeness_to_goal": 1
          }

     # RRT parameters
     sample_radius = 3
     neighbor_radius = 3
     n_expansions = 5000
     also_run_normal_RRT = False
     stop_when_goal_reached = True
     animate_plot = False

     # start configuration and goal coordinates
     initial_config = [-9.5, -9.5, 0, 0, (1/2)*np.pi]
     goal = [9, 9, 1]

     # Create environment, robot and RRT algorithm objecs
     env = environment.Environment()
     coffeebot = robot.Robot(env.env, initial_config, l1, l2, l3)
     rrt = RRTstar.RRTstar(l1, l2, l3, room)

     # run RRTstar
     rrt.run(initial_config, goal, 
            sample_radius=sample_radius,
            neighbor_radius=neighbor_radius,
            n_expansions=n_expansions, 
            stop_when_goal_reached=stop_when_goal_reached, 
            also_run_normal_RRT=also_run_normal_RRT,
            animate_plot=animate_plot)

     # Display result (the shortest path)
     result = rrt.get_shortest_path()
     print(f"Shortest path configs: {result}")
     rrt.plot_results(also_run_normal_RRT)

     # Retrieve and follow the planned path
     for config in result:
          print(f"current config: {config}")
          coffeebot.move_to_goal(goal = config)
     time.sleep(1)

     env.close_simulation()