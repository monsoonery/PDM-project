import numpy as np
import random

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import obstacles

verbose = False

class RRTstar:
    def __init__(self, l1, l2, l3):
        # room properties
        self.room = {
            "width": 20,
            "length": 20,
            "height": 8,
            "margin_of_closeness_to_goal": 2
        }
        self.obstacles = obstacles.collision_obstacles

        # robot parameters (needed for collision checks)
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        self.initial_config = [0,0,0,0,0]
        self.goal_xyz = [0,0,0]

        # dicts for RRT
        self.nodes = {}
        self.edges = {}

        # dict for RRT star
        self.vertices = {}

        # for storing the shortest path
        self.shortest_path_keys = []
        self.shortest_path_configs = []

    # Returns a random sample in configuration space
    def get_random_sample(self):
        #x = random.randint(0, self.room["width"])
        #y = random.randint(0, self.room["length"])
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        q1 = random.uniform(0, 2*np.pi)
        q2 = random.uniform(-np.pi/2, np.pi/2)
        q3 = random.uniform(-2*np.pi/3, 2*np.pi/3)
        return [x,y,q1,q2,q3]

    # Calculates distance between two nodes
    def get_distance_of_two_nodes(self, node_1, node_2):
        x1, y1, q1_node1, q2_node1, q3_node1 = node_1
        x2, y2, q1_node2, q2_node2, q3_node2 = node_2
        
        distance_xy = np.sqrt((x2-x1)**2+(y2-y1)**2)
        diff_list = [abs(q1_node1-q1_node2), abs(q2_node1-q2_node2), abs(q3_node1-q3_node2)]

        # shift angle differences to be between -pi and pi
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

    # Determines the Euclidean distance between a sphere on the robot and all obstacles and returns whether they are in collision
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

    # Determines if steering towards a given configuration would cause collision with any objects
    def in_collision(self, start_config, end_config):
        """
        Determines if the robot will be in collision with obstacles if it were to be steered from start_config
        to end_config. The robot's "hitbox" is approximated using the 'union of spheres' method

        INPUT: start_config -> List containing start configuration of the robot, format [x, y, q1, q2, q3] 
               end_config -> List containing end configuration of the robot, format [x, y, q1, q2, q3]
        OUTPUT: boolean -> if True, moving between the start and end configurations will result in collision
        """

        # Steering function is a straight line in config space, so we can interpolate values 
        # between the current config and the desired config
        q_deltas = np.linspace(start_config, end_config, 10)
        for config in q_deltas:
            ##### Extract values
            x, y, q1, q2, q3 = config

            ##### Step 1: Check x and y collision (mobile base only)
            if (verbose): print("~~~~~~~~ Currently checking x and y collision ~~~~~~~~")
            x_robot = x
            y_robot = y
            z_robot = self.l1 / 2 # center of base
            r_robot = self.l1 / 2 # value taken from URDF, assuming the base is just 1 sphere (extremely simplified)

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
            
        # If none of the q_delta's ever return True, then there must be no collisions
        return False
    
    # Determines the xyz-coordinates of a point on the robot
    def forward_kinematics(self, config, l1, l2, l3):
        x, y, q1, q2, q3 = config
        x_p = np.cos(q1)*(np.sin(q2)*l2 + np.cos(q2 + q3)*l3) + x
        y_p = np.sin(q1)*(np.sin(q2)*l2 - np.cos(q2 + q3)*l3) + y
        z_p = np.cos(q2)*l2 - np.sin(q2 + q3)*l3 + l1
        
        return [x_p, y_p, z_p]

    # Determines the xyz-coordinates of the robot endpoint using the forward_kinematics model
    def get_endpoint_coordinates(self, config):
        return self.forward_kinematics(config, self.l1, self.l2, self.l3)

    # Returns whether or not a (random) configuration is sufficiently close to the goal
    def config_is_near_goal(self, config, goal):
        x1, y1, z1 = self.get_endpoint_coordinates(config)
        x2, y2, z2 = goal
        margin = self.room["margin_of_closeness_to_goal"]
        
        # Check if the absolute differences in each dimension are within the margin
        x_close = abs(x1 - x2) <= margin
        y_close = abs(y1 - y2) <= margin
        z_close = abs(z1 - z2) <= margin

        # Return True if all dimensions are close, otherwise return False
        #print(x_close, y_close, z_close)
        return x_close and y_close and z_close

    # Runs the RRT* (STAR) algorithm to create a graph/tree
    def generate_graph_RRTstar(self, n_expansions=1000, initial_config=[0, 0, 0, 0, 0], goal_xyz=[0, 0, 0]):
        self.goal_xyz = goal_xyz
        self.initial_config = initial_config

        sample_threshold = 3

        self.vertices = {
            # DATA STRUCTURE:
            # node ID: [config/coordinates, cost-to-reach-from-start, parent node ID]
            0: [self.initial_config, 0, 0]
        }

        neighborhood = 3

        node_id = 1
        # Iterate until a path is found OR the max number of iterations is reached, whichever comes sooner
        for i in range(1, n_expansions):
            #print(f"i = {i}")
            # [1] Get a random configuration sample
            q_rand = self.get_random_sample()

            # [2] Find the nearest node this sample could be theoretically connected to
            #     If q_rand is too far away from this node, discard this sample
            key_q_near = min(self.vertices.keys(), 
                             key=lambda node: self.get_distance_of_two_nodes(
                                q_rand, self.vertices[node][0]
                                )
                             )
            if self.get_distance_of_two_nodes(q_rand, self.vertices[key_q_near][0]) > sample_threshold:
                #print("this sample would be too far away")
                continue

            # [3] Check if the robot will be in collision at any point between q_near and q_rand
            #     If there is collision, discard this sample
            if self.in_collision(self.vertices[key_q_near][0], q_rand):  
                #print("this sample would be in collision (possibly while moving) :<")
                continue

            # --------- If the code gets to this point, the randomly sampled config is valid and collision-free ---------

            q_new = q_rand

            # --------- From here on out the STAR part of the algorithm is implemented: rewiring nodes ---------

            # [4] Cost to reach q_new from start via q_near
            #     This is currently our "best guess" for shortest path from start to q_new
            cost_q_near_to_q_new = self.get_distance_of_two_nodes(self.vertices[key_q_near][0], q_new)
            cost_start_to_q_new_via_q_near = self.vertices[key_q_near][1] + cost_q_near_to_q_new

            # [5] Add an entry for the q_new node: [config, cost start to node, parent node]
            key_q_new = node_id
            self.vertices[key_q_new] = [q_new, cost_start_to_q_new_via_q_near, key_q_near]
            
            # [6] Find all neighbors of q_new, these are candidates for rewiring 
            keys_q_neighbors = []
            for key_node in self.vertices.keys():
                distance = self.get_distance_of_two_nodes(self.vertices[key_node][0], q_new)
                if distance <= neighborhood:
                    keys_q_neighbors.append(key_node)
            #print(f"neighbors: {keys_q_neighbors}")

            # [7] Check if q_new can be connected to a different node with a lower total cost than its current connection
            for key_q_neighbor in keys_q_neighbors:
                start_to_q_neighbor_cost = self.vertices[key_q_neighbor][1]
                q_neighbor_to_q_new_cost = self.get_distance_of_two_nodes(self.vertices[key_q_neighbor][0], q_new)
                # "if cost of start->neighbor->q_new is smaller than the cost currently registered for q_new"
                total_cost = start_to_q_neighbor_cost + q_neighbor_to_q_new_cost
                if (total_cost < self.vertices[key_q_new][1]):
                    # We need to check if this connection would cause a collision before rewiring
                    if not self.in_collision(self.vertices[key_q_neighbor][0], q_new):
                        # update q_new's cost and parent
                        #print(self.vertices[key_q_new][1])
                        self.vertices[key_q_new][2] = key_q_neighbor
                        self.vertices[key_q_new][1] = total_cost

            
            # [] Stop when a valid path to the goal is found
            #    The last added node's parents are connected to the start point in the (so far) shortest way
            if self.config_is_near_goal(q_rand, goal_xyz):
                break

            node_id += 1
            
        if i == n_expansions: print("Max number of iterations reached :(")

    # Runs the RRT algorithm to create a graph/tree
    def generate_graph_RRT(self, n_expansions=1000, initial_config=[0, 0, 0, 0, 0], goal_xyz=[0, 0, 0]):
        self.goal_xyz = goal_xyz
        self.initial_config = initial_config

        sample_threshold = 3
        
        self.nodes = {
            # DATA STRUCTURE: 
            # NODE_ID: CONFIGURATION
            0: initial_config
        }

        self.edges = {
            # DATA STRUCTURE: 
            # NODE_ID: [(NEXT_NODE_ID, DISTANCE/COST), (NEXT_NODE_ID, DISTANCE/COST), etc.]
            # 0: [(1, 3.2), (2, 4.56)],
        }

        node_id = 1
        # Iterate until a path is found OR the max number of iterations is reached, whichever comes sooner
        for i in range(1, n_expansions):
            # [1] Get a random configuration sample
            q_rand = self.get_random_sample()

            # [2] Find the nearest node this sample could be connected to
            #     If q_rand is too far away from this node, discard this sample
            key_q_near = min(self.nodes.keys(), key=lambda node: self.get_distance_of_two_nodes(q_rand, self.nodes[node]))

            if self.get_distance_of_two_nodes(q_rand, self.nodes[key_q_near]) > sample_threshold:
                #print("this sample would be too far away")
                continue

            # [3] Check if the robot will be in collision at any point between q_near and q_rand
            #     If there is collision, discard this sample
            if self.in_collision(self.nodes[key_q_near], q_rand):  
                #print("this sample would be in collision (possibly while moving) :<")
                continue

            # --------- If the code gets to this point, the randomly sampled config is valid and collision-free ---------

            # Add this new node to the node dictionary
            key_q_rand = node_id
            self.nodes[key_q_rand] = q_rand

            # Add an edge to the edges dictionary
            # If the nearest node doesn't have edges yet, create an entry in the dict
            # else append the edge entry to the existing list of edges for the nearest node
            distance_q_rand_to_q_near = self.get_distance_of_two_nodes(q_rand, self.nodes[key_q_near])
            if key_q_near in self.edges:
                self.edges[key_q_near].append((key_q_rand, distance_q_rand_to_q_near))
            else:
                self.edges[key_q_near] = [(key_q_rand, distance_q_rand_to_q_near)]

            if self.config_is_near_goal(q_rand, goal_xyz):
                break

            node_id += 1
        
        return

    # Returns the shortest path from start to goal using a backwards search
    def get_shortest_path(self):
        if not self.shortest_path_configs:
            self.shortest_path_keys = []

            key_child_node = list(self.vertices.keys())[-1]
            key_parent_node = self.vertices[key_child_node][2]

            while (key_parent_node != 0):
                self.shortest_path_keys.append(key_child_node)

                # Retrieve the parent node key from the child node's dict entry
                key_parent_node = self.vertices[key_child_node][2]

                # Set the child node key as the parent node for the next iteration
                key_child_node = key_parent_node

            for key in self.shortest_path_keys:
                self.shortest_path_configs.append(self.vertices[key][0])

            # Nodes/configs are listed from goal to start, so lists must be reversed
            self.shortest_path_keys.reverse()
            self.shortest_path_configs.reverse()
                
        return self.shortest_path_configs

    # Plots the RRT* (STAR) graph
    def plot_results_RRTstar(self):
        plt.figure(2)
        # Get list of node id's (dict keys) for iterating over
        keys_nodes = self.vertices.keys()

        # Extract datapoints from node list: get x, y, etc of each node
        x_values = [self.vertices[key_node][0][0] for key_node in keys_nodes]
        y_values = [self.vertices[key_node][0][1] for key_node in keys_nodes]
        q1_values = [self.vertices[key_node][0][2] for key_node in keys_nodes]
        q2_values = [self.vertices[key_node][0][3] for key_node in keys_nodes]
        q3_values = [self.vertices[key_node][0][4] for key_node in keys_nodes]
        
        # Plot obstacles
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle.position()[0], obstacle.position()[1]), obstacle.radius(), color='grey')
            plt.gca().add_patch(circle)
        
        # Plot nodes
        plt.scatter(x_values, y_values, label='Nodes')

        # Plot edges
        flag = False
        for key_node in keys_nodes:
            # Retrieve the parent node key from the child node's dict entry
            key_parent_node = self.vertices[key_node][2]

            # get child node config
            child_node = self.vertices[key_node][0]
            # get parent node config
            parent_node = self.vertices[key_parent_node][0]

            # Plot an edge between child and parent
            if not flag:
                plt.plot([parent_node[0], child_node[0]], [parent_node[1], child_node[1]], color='black', label = "Edges")
                flag = True
            else:
                plt.plot([parent_node[0], child_node[0]], [parent_node[1], child_node[1]], color='black', )
        
        # Plot shortest path
        flag = False
        for key in self.shortest_path_keys:
            # Retrieve the parent node key from the child node's dict entry
            key_parent_node = self.vertices[key][2]

            # get child node config
            child_node = self.vertices[key][0]
            # get parent node config
            parent_node = self.vertices[key_parent_node][0]

            # Plot an edge between child and parent
            if not flag:
                plt.plot([parent_node[0], child_node[0]], [parent_node[1], child_node[1]], color='yellow', label='Shortest path')
                flag = True
            else:
                plt.plot([parent_node[0], child_node[0]], [parent_node[1], child_node[1]], color='yellow')
            

        # Plot start and goal points with different colors
        plt.scatter(x_values[0], y_values[0], color='red', label='Start configuration')
        plt.scatter(self.goal_xyz[0], self.goal_xyz[1], color='green', label='Goal configuration')

        # Plot goal margin
        margin = self.room["margin_of_closeness_to_goal"]
        square = plt.Rectangle((self.goal_xyz[0] - margin, self.goal_xyz[1] - margin), 2 * margin, 2 * margin,
                                    color='blue', fill=False, label='Goal margin')
        plt.gca().add_patch(square)

        plt.title('RRT* graph!!!!!!!!!!!!!!!!!!!')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.xlim([-10, 10])
        plt.ylim([-10, 10])
        plt.legend()
        return
    
    # Plots the RRT graph
    def plot_results_RRT(self):
        # Get list of node id's (dict keys) for iterating over
        plt.figure(1)
        keys_nodes = self.nodes.keys()

        # Extract datapoints from node list: get x, y, etc of each node
        x_values = [self.nodes[key_node][0] for key_node in keys_nodes]
        y_values = [self.nodes[key_node][1] for key_node in keys_nodes]
        q1_values = [self.nodes[key_node][2] for key_node in keys_nodes]
        q2_values = [self.nodes[key_node][3] for key_node in keys_nodes]
        q3_values = [self.nodes[key_node][4] for key_node in keys_nodes]
        
        # Plot obstacles
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle.position()[0], obstacle.position()[1]), obstacle.radius(), color='grey')
            plt.gca().add_patch(circle)

        # Plot edges
        for key_node in self.edges.keys():
            for edge in self.edges[key_node]:
                key_next_node = edge[0]
                key_current_node = key_node

                next_node = self.nodes[key_current_node]
                current_node = self.nodes[key_next_node]
                plt.plot([current_node[0], next_node[0]], [current_node[1], next_node[1]], 'k-')
        
        # Plot nodes
        plt.scatter(x_values, y_values)

        # Plot shortest path

        
        # Plot start and goal points with different colors
        plt.scatter(x_values[0], y_values[0], color='red', label='Start Configuration')
        plt.scatter(self.goal_xyz[0], self.goal_xyz[1], color='green', label='Goal Configuration')

        # Plot goal margin
        margin = self.room["margin_of_closeness_to_goal"]
        square = plt.Rectangle((self.goal_xyz[0] - margin, self.goal_xyz[1] - margin), 2 * margin, 2 * margin,
                                    color='blue', fill=False, label='Margin')
        plt.gca().add_patch(square)

        plt.title('RRT graph')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.xlim([-10, 10])
        plt.ylim([-10, 10])
        return



if __name__ == "__main__":
    rrt = RRTstar(0.4, 0.7, 0.6)

    # start configuration
    config_s = [0, 0, 0, 0, (1/2)*np.pi]
    goal = [8, 8, 2]

    rrt.generate_graph_RRTstar(n_expansions=1000, initial_config=config_s, goal_xyz=goal)
    rrt.generate_graph_RRT(n_expansions=1000, initial_config=config_s, goal_xyz=goal)
    
    print(rrt.get_shortest_path())

    rrt.plot_results_RRT()
    rrt.plot_results_RRTstar()
    plt.show()


