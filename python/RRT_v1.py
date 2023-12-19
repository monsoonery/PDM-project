import numpy as np
import random

import matplotlib.pyplot as plt
import ast


class RRTstar:
    def __init__(self, l1, l2, l3):
        self.room_variables = {
            "width": 100,
            "length": 100,
            "height": 20,
            "margin_of_closeness_to_goal": 10
        }

        self.robot_variables = {
            "l1": l1,
            "l2": l2,
            "l3": l3,
            "radius_base": 0.4
        }

        self.all_samples = []
        self.tree = {}


    def get_random_sample(self):
        x = random.randint(0, self.room_variables["width"])
        y = random.randint(0, self.room_variables["length"])
        q1 = random.randint(0, 360)
        q2 = random.randint(0, 360)
        q3 = random.randint(0, 360)
        return [x,y,q1,q2,q3]
    
    def in_collision(self, config=[0, 0, 0, 0, 0]):
        return False
    
    def check_distance_of_two_nodes(self, node_1, node_2):
        x1, y1, q1_node1, q2_node1, q3_node1 = node_1
        x2, y2, q1_node2, q2_node2, q3_node2 = node_2
        
        distance_xy = np.sqrt((x2-x1)**2+(y2-y1)**2)
        diff_list = [abs(q1_node1-q1_node2), abs(q2_node1-q2_node2), abs(q3_node1-q3_node2)]
        for i in range(len(diff_list)):
            if -90 <= diff_list[i] <= 90:
                #print(f"{diff_list[i]} is between -90 and 90")
                diff_list[i] = diff_list[i]
            elif diff_list[i] > 90:
                #print(f"{diff_list[i]} is higher than 90")
                diff_list[i] = diff_list[i] - 2*90
            elif diff_list[i] < -90:
                #print(f"{diff_list[i]} is lower than -90")
                diff_list[i] = diff_list[i] + 2*90

            #uncomment for degrees:
            #diff_list[i] = diff_list[i]
                
            #uncomment for radians:
            diff_list[i] = diff_list[i] * (np.pi/180)

        distance_q1_q2 = distance_xy + sum(diff_list)
        return distance_q1_q2
    
    def position_end_point(self, config):
        l1 = self.robot_variables["l1"]
        l2 = self.robot_variables["l2"]
        l3 = self.robot_variables["l3"]
        x, y, q1, q2, q3 = config
        
        x_end = np.cos(q1)*(np.sin(q2)*l2 + np.cos(q2 + q3)*l3) + x
        y_end = np.sin(q1)*(np.sin(q2)*l2 - np.cos(q2 + q3)*l3) + y
        z_end = np.cos(q2)*l2 - np.sin(q2 + q3)*l3 + l1

        return [x_end, y_end, z_end]
    
    def check_if_configuration_is_at_goal(self, config, goal):
        x1, y1, z1 = self.position_end_point(config)
        x2, y2, z2 = goal
        margin = self.room_variables["margin_of_closeness_to_goal"]
        
        # Check if the absolute differences in each dimension are within the margin
        x_close = abs(x1 - x2) <= margin
        y_close = abs(y1 - y2) <= margin
        z_close = abs(z1 - z2) <= margin

        # Return True if all dimensions are close, otherwise return False
        #print(x_close, y_close, z_close)
        return x_close and y_close and z_close

    def run_algorithm(self, n_expansions=1000, initial_configuration=[0, 0, 0, 0, 0], goal_xyz=[0, 0, 0]):
        #initalize start
        self.all_samples.append(initial_configuration)
        self.tree[str(initial_configuration)] = []

        for i in range(n_expansions):
            sample = self.get_random_sample()
            if not self.in_collision(sample):
                #find closest node
                closest_node = min(self.all_samples, key=lambda node: self.check_distance_of_two_nodes(sample, node))

                #initialize new node to tree
                self.tree[str(sample)] = []
                self.all_samples.append(sample)

                #reference path of closest node to new node on tree
                self.tree[str(closest_node)].append(sample)

                at_goal = self.check_if_configuration_is_at_goal(sample, goal_xyz)

            if at_goal:
                break
        return self.tree
        


#==================================================================================================================================
#==================================================================================================================================
#==================================================================================================================================
#==================================================================================================================================
#==================================================================================================================================

if __name__ == "__main__":
    rrt = RRTstar(0.4, 0.7, 0.6)
    data_dict = rrt.run_algorithm(n_expansions=1000, initial_configuration=[0, 0, 0, 0, 0], goal_xyz=[50, 50, 2])

    start = [0, 0]
    goal = [50, 50, 2]

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

    #circle = plt.Circle((50, 50), define_space_and_robot()[0]["margin_of_closeness_to_goal"], color='blue', fill=False, label='Margin')
    #plt.gca().add_patch(circle)

    margin = rrt.room_variables["margin_of_closeness_to_goal"]
    square = plt.Rectangle((50 - margin, 50 - margin), 2 * margin, 2 * margin,
                                color='blue', fill=False, label='Margin')
    plt.gca().add_patch(square)

    plt.title('Scatter Plot of Dictionary Keys')
    plt.xlabel('X Values')
    plt.ylabel('Y Values')
    plt.show()
