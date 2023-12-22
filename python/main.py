import time
import matplotlib.pyplot as plt
import random

import robot
import RRT_v2 as RRTstar
import environment

if __name__ == "__main__":
    l1 = 0.4
    l2 = 0.7
    l3 = 0.6

    # RRT* parameters
    initial_config = [-10, -10, 0, 0, 0]
    goal = [8, 8, 2]
    n_expansions = 1000

    # Create environment, robot and RRT algorithm objecs
    env = environment.Environment()
    coffeebot = robot.Robot(env.env, initial_config, l1, l2, l3)
    rrt = RRTstar.RRTstar(l1, l2, l3)

    # Run RRT*
    rrt.generate_graph_RRTstar(1000, initial_config, goal)
    rrt.get_shortest_path()

    # Plot graph
    rrt.plot_results_RRTstar()
    plt.show()

    # Retrieve and follow the planned path
    results = rrt.get_shortest_path()
    for config in results:
         print(f"current config: {config}")
         coffeebot.move_to_goal(goal = config)
         time.sleep(1)

    env.close_simulation()