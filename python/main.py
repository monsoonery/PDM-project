import random
import numpy as np
import time
import ast
import matplotlib.pyplot as plt

import robot
import RRT_v1 as RRTstar
import environment

def plot_results(data_dict):
    start = [0, 0]

    # Extract x and y values from the keys
    x_values = []
    y_values = []

    for key in data_dict.keys():
        # Assuming the first two elements are x and y
        key = ast.literal_eval(key)

        x_values.append(key[0])
        y_values.append(key[1]) 

    # Plotting
    for key in data_dict.keys():
        # Assuming the first two elements are x and y
        x_and_y = ast.literal_eval(key)[:2]
        for item in data_dict[key]:
            s_and_z = item[:2]
            plt.plot([x_and_y[0], s_and_z[0]], [x_and_y[1], s_and_z[1]], 'k-')

    for key in data_dict.keys():
        # Assuming the first two elements are x and y
        if rrt.check_if_configuration_is_at_goal(ast.literal_eval(key), goal):
            current_node = ast.literal_eval(key)
            while current_node[:2] != start:
                #print(f"current_node is {current_node[:2]} and not {start} so continuing")
                for parentnode, children in data_dict.items():
                    if current_node in children:
                        #print(f"line from {current_node[:2]} to {ast.literal_eval(parentnode)[:2]}")
                        plt.plot([current_node[:2][0], ast.literal_eval(parentnode)[:2][0]], [current_node[:2][1], ast.literal_eval(parentnode)[:2][1]], color='green')
                        current_node = ast.literal_eval(parentnode)       

    plt.scatter(x_values, y_values)

    plt.scatter(start[0], start[1], color='red', label='Start Configuration')
    plt.scatter(goal[0], goal[1], color='green', label='Goal Configuration')

    margin = rrt.room_variables["margin_of_closeness_to_goal"]
    #square = plt.Rectangle((50 - margin, 50 - margin), 2 * margin, 2 * margin,
    #                            color='blue', fill=False, label='Margin')
    #plt.gca().add_patch(square)

    plt.title('Scatter Plot of Dictionary Keys')
    plt.xlabel('X Values')
    plt.ylabel('Y Values')
    plt.show()


if __name__ == "__main__":
    l1 = 0.4
    l2 = 0.7
    l3 = 0.6

    env = environment.Environment()
    coffeebot = robot.Robot(env.env, l1, l2, l3)
    rrt = RRTstar.RRTstar(l1, l2, l3)

    initial_config = [0, 0, 0, 0, 0]
    goal = [8, 8, 2]
    n_expansions = 1000

    data_dict, results = rrt.generate_graph(n_expansions, initial_config, goal)
    
    plot_results(data_dict)
    
    for config in results:
        print(f"current config: {config}")
        coffeebot.move_to_goal(goal = config)
        time.sleep(1)

    env.close_simulation()