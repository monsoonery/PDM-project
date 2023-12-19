import numpy as np
import random

import matplotlib.pyplot as plt
import ast

import obstacles

verbose = False

class RRTstar:
    def __init__(self, l1, l2, l3):
        self.room_variables = {
            "width": 20,
            "length": 20,
            "height": 8,
            "margin_of_closeness_to_goal": 2
        }

        self.robot_variables = {
            "l1": l1,
            "l2": l2,
            "l3": l3,
            "radius_base": 0.4
        }

        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        self.all_samples = []
        self.tree = {}

        self.obstacles = obstacles.collision_obstacles

    def get_random_sample(self):
        #x = random.randint(0, self.room_variables["width"])
        #y = random.randint(0, self.room_variables["length"])
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        q1 = random.uniform(0, 2*np.pi)
        q2 = random.uniform(-np.pi/2, np.pi/2)
        q3 = random.uniform(-2*np.pi/3, 2*np.pi/3)
        return [x,y,q1,q2,q3]

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

    def in_collision(self, config):
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
            x_l2, y_l2, z_l2 = self.forward_kinematics(config, self.l1, l2, 0)
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
            x_l3, y_l3, z_l3 = self.forward_kinematics(config, self.l1, self.l2, l3)
            r_l3 = 0.1 # from URDF

            if self.intersects_any_obstacle(x_l3, y_l3, z_l3, r_l3):
                if (verbose): print("Collision!")
                return True
            else:
                if (verbose): print("No collision :)")
        
        # If none of the above ever return True, then there must be no collisions
        return False
    
    def check_distance_of_two_nodes(self, node_1, node_2):
        x1, y1, q1_node1, q2_node1, q3_node1 = node_1
        x2, y2, q1_node2, q2_node2, q3_node2 = node_2
        
        distance_xy = np.sqrt((x2-x1)**2+(y2-y1)**2)
        diff_list = [abs(q1_node1-q1_node2), abs(q2_node1-q2_node2), abs(q3_node1-q3_node2)]
        for i in range(len(diff_list)):
            if -0.5*np.pi <= diff_list[i] <= 0.5*np.pi:
                #print(f"{diff_list[i]} is between -90 and 90")
                diff_list[i] = diff_list[i]
            elif diff_list[i] > 0.5*np.pi:
                #print(f"{diff_list[i]} is higher than 90")
                diff_list[i] = diff_list[i] - np.pi
            elif diff_list[i] < -0.5*np.pi:
                #print(f"{diff_list[i]} is lower than -90")
                diff_list[i] = diff_list[i] + np.pi

            #uncomment for degrees:
            #diff_list[i] = diff_list[i]
                
            #uncomment for radians:
            diff_list[i] = diff_list[i] * (np.pi/180)

        distance_q1_q2 = distance_xy + sum(diff_list)
        return distance_q1_q2
    
    def get_endpoint_coordinates(self, config):
        return self.forward_kinematics(config, self.l1, self.l2, self.l3)

    def forward_kinematics(self, config, l1, l2, l3):
        x, y, q1, q2, q3 = config
        x_p = np.cos(q1)*(np.sin(q2)*l2 + np.cos(q2 + q3)*l3) + x
        y_p = np.sin(q1)*(np.sin(q2)*l2 - np.cos(q2 + q3)*l3) + y
        z_p = np.cos(q2)*l2 - np.sin(q2 + q3)*l3 + l1
        
        return [x_p, y_p, z_p]

    def check_if_configuration_is_at_goal(self, config, goal):
        x1, y1, z1 = self.get_endpoint_coordinates(config)
        x2, y2, z2 = goal
        margin = self.room_variables["margin_of_closeness_to_goal"]
        
        # Check if the absolute differences in each dimension are within the margin
        x_close = abs(x1 - x2) <= margin
        y_close = abs(y1 - y2) <= margin
        z_close = abs(z1 - z2) <= margin

        # Return True if all dimensions are close, otherwise return False
        #print(x_close, y_close, z_close)
        return x_close and y_close and z_close

    def generate_graph(self, n_expansions=1000, initial_configuration=[0, 0, 0, 0, 0], goal_xyz=[0, 0, 0]):
        #initalize start
        self.all_samples.append(initial_configuration)
        self.tree[str(initial_configuration)] = []

        for i in range(n_expansions):
            sample = self.get_random_sample()
            if not self.in_collision(sample):
                # TODO for loop to check if arm would be in collision 
                # WHILE travelling from node 1 to 2



                #find closest node
                closest_node = min(self.all_samples, key=lambda node: self.check_distance_of_two_nodes(sample, node))

                #initialize new node to tree
                self.tree[str(sample)] = []
                self.all_samples.append(sample)

                #reference path of closest node to new node on tree
                self.tree[str(closest_node)].append(sample)

                at_goal = self.check_if_configuration_is_at_goal(sample, goal_xyz)
            else:
                print("that sample causes collision")
            if at_goal:
                break

        for key in self.tree.keys():
            Finalpath = []
            if self.check_if_configuration_is_at_goal(ast.literal_eval(key), goal_xyz):
                current_node = ast.literal_eval(key)
                while current_node != [0, 0, 0, 0, 0]:
                    for parentnode, children in self.tree.items():
                        if current_node in children:
                            print(current_node)
                            Finalpath.insert(0, current_node)
                            current_node = ast.literal_eval(parentnode)

        return self.tree, Finalpath
        

if __name__ == "__main__":
    rrt = RRTstar(0.4, 0.7, 0.6)

    start = [0, 0]
    goal = [8, 8, 2]

    data_dict, _ = rrt.generate_graph(n_expansions=1000, initial_configuration=[0, 0, 0, 0, 0], goal_xyz=goal)

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
    square = plt.Rectangle((goal[0] - margin, goal[1] - margin), 2 * margin, 2 * margin,
                                color='blue', fill=False, label='Margin')
    plt.gca().add_patch(square)

    plt.title('Scatter Plot of Dictionary Keys')
    plt.xlabel('X Values')
    plt.ylabel('Y Values')
    plt.show()
