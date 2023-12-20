import random
import numpy as np
import time
import ast
import matplotlib.pyplot as plt

import robot
import RRT_v1 as RRTstar
import environment

if __name__ == "__main__":
    l1 = 0.4
    l2 = 0.7
    l3 = 0.6

    # Create environment, robot and RRT algorithm objecs
    env = environment.Environment()
    coffeebot = robot.Robot(env.env, l1, l2, l3)
    rrt = RRTstar.RRTstar(l1, l2, l3)

    initial_config = [0, 0, 0, 0, 0]
    goal = [8, 8, 2]
    n_expansions = 1000

    data_dict = rrt.generate_graph(n_expansions, initial_config, goal)
    
    rrt.plot_results()

    results = rrt.get_shortest_path()

    for config in results:
        print(f"current config: {config}")
        coffeebot.move_to_goal(goal = config)
        time.sleep(1)

    env.close_simulation()