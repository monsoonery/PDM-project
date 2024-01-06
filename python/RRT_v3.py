import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import obstacles

# Flags for enabling extra print statements
debugCollision = False
debugRRT = False

class RRTstar:
    def __init__(self, l1, l2, l3, room):
        # room properties
        self.room = room
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

    """--------------------- HELPER FUNCTIONS ---------------------"""
    # Returns a random sample in configuration space
    def get_random_sample(self):
        x = random.uniform(self.room["width"][0], self.room["width"][1])
        y = random.uniform(self.room["length"][0], self.room["length"][1])
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
        if (debugCollision): print(f"Robot sphere parameters: x: {x}, y: {y}, z: {z}, r: {r}")

        for obstacle in self.obstacles:
            if (debugCollision): print(f"Checking obstacle with name {obstacle.name()}")

            x_obst = obstacle.position()[0]
            y_obst = obstacle.position()[1]
            z_obst = obstacle.position()[2]
            r_obst = obstacle.radius() 

            distance = np.sqrt((x - x_obst)**2 + (y - y_obst)**2 + (z - z_obst)**2)

            if (debugCollision): print(f"- Euclidean distance of robot sphere to {obstacle.name()} = {distance}", )
            if (debugCollision): print(f"- Radius of robot sphere + radius of {obstacle.name()} = {r_obst + r}")

            if (distance < r_obst + r):
                if (debugCollision): print(f'[!] Robot is in collision with {obstacle.name()}. Quitting this loop')
                flag = True
            else:
                if (debugCollision): print(f'[x] No collision detected with {obstacle.name()}. Continuing')
            
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
            if (debugCollision): print("~~~~~~~~ Currently checking x and y collision ~~~~~~~~")
            x_robot = x
            y_robot = y
            z_robot = self.l1 / 2 # center of base
            r_robot = np.sqrt(2*(self.l1 / 2)**2) # radius of sphere that encapsulates the cylindrical base

            if self.intersects_any_obstacle(x_robot, y_robot, z_robot, r_robot):
                if (debugCollision): print("Collision!")
                return True
            else:
                if (debugCollision): print("No collision :)")

            ##### Step 2: Check first link collision (has dimension l2, beetje ongelukkige naam i know)
            if (debugCollision): print("~~~~~~~~ Currently checking collision with first link ~~~~~~~~")
            l2_division = 7 # in how many spheres will l2 be divided?

            for l2 in np.linspace(0, self.l2, l2_division):
                if (debugCollision): print(f"~~~~~~~ l2 = {l2}")
                x_l2, y_l2, z_l2 = self.forward_kinematics(config, self.l1, l2, 0)
                r_l2 = 0.1 # from URDF

                if self.intersects_any_obstacle(x_l2, y_l2, z_l2, r_l2):
                    if (debugCollision): print("Collision!")
                    return True
                else:
                    if (debugCollision): print("No collision :)")

            ##### Step 3: Check second link collision (has dimension l3)
            if (debugCollision): print("~~~~~~~~ Currently checking collision with second link ~~~~~~~~")
            l3_division = 6

            for l3 in np.linspace(0, self.l3, l3_division):
                if (debugCollision): print(f"~~~~~~~ l3 = {l3}")
                x_l3, y_l3, z_l3 = self.forward_kinematics(config, self.l1, self.l2, l3)
                r_l3 = 0.1 # from URDF

                if self.intersects_any_obstacle(x_l3, y_l3, z_l3, r_l3):
                    if (debugCollision): print("Collision!")
                    return True
                else:
                    if (debugCollision): print("No collision :)")
            
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

    # This function updates the pyplot to show the RRT* expansion live
    def animate(self):
        plt.clf()

        # Extract datapoints from node list: get x, y, etc of each node
        keys_nodes = self.vertices.keys()
        x_values = [self.vertices[key_node][0][0] for key_node in keys_nodes]
        y_values = [self.vertices[key_node][0][1] for key_node in keys_nodes]
        q1_values = [self.vertices[key_node][0][2] for key_node in keys_nodes]
        q2_values = [self.vertices[key_node][0][3] for key_node in keys_nodes]
        q3_values = [self.vertices[key_node][0][4] for key_node in keys_nodes]
        
        # Plot obstacles
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle.position()[0], obstacle.position()[1]), 
                                obstacle.radius(), 
                                color='grey')
            plt.gca().add_patch(circle)
    
        # Plot nodes
        plt.scatter(x_values, y_values, s=15, label='Nodes')

        # Plot edges
        for i, key_node in enumerate(keys_nodes):
            # Retrieve the parent node key from the child node's dict entry
            key_parent_node = self.vertices[key_node][2]

            # get child node config
            child_node = self.vertices[key_node][0]
            # get parent node config
            parent_node = self.vertices[key_parent_node][0]

            # Plot an edge between child and parent
            if i == 0:
                plt.plot([parent_node[0], child_node[0]], 
                        [parent_node[1], child_node[1]], 
                        color='black', linewidth=1, label = "Edges")
            else:
                plt.plot([parent_node[0], child_node[0]], 
                        [parent_node[1], child_node[1]], 
                        color='black', linewidth=1)
        
        # Plot shortest path
        for i, key_child_node in enumerate(self.shortest_path_keys):
            # Retrieve the parent node key from the child node's dict entry
            key_parent_node = self.vertices[key_child_node][2]

            # get child node and parent node config
            child_node = self.vertices[key_child_node][0]
            parent_node = self.vertices[key_parent_node][0]

            # Plot an edge between child and parent
            if i == 0:
                plt.plot([parent_node[0], child_node[0]], 
                        [parent_node[1], child_node[1]], 
                        linewidth=1.5, color='m', label='Shortest path')
            else:
                plt.plot([parent_node[0], child_node[0]], 
                        [parent_node[1], child_node[1]], 
                        linewidth=1.5, color='m')

        # Plot start and goal points with different colors
        if x_values: plt.scatter(x_values[0], y_values[0], 
                    color='red', s=100,
                    label='Start configuration')
        plt.scatter(self.goal_xyz[0], self.goal_xyz[1], 
                    color='green', s=100,
                    label='Goal configuration')

        # Plot goal margin
        margin = self.room["margin_of_closeness_to_goal"]
        circle = plt.Circle((self.goal_xyz[0], self.goal_xyz[1]), 
                            margin,
                            color='green', fill=False, 
                            label='Goal margin')
        plt.gca().add_patch(circle)

        plt.title('RRT* graph (LIVE!)')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.xlim([-10, 10])
        plt.ylim([-10, 10])
        plt.legend(loc='upper left')
        plt.tight_layout()
        plt.pause(0.1)


    """--------------------- ACTUAL FUNCTIONS ---------------------"""
    # Returns the shortest path from start to goal using a backwards search (works for RRT* only!)
    def get_shortest_path(self):
        # if we try to call this function without running the algorithm first, throw an error
        if not self.vertices:
            print("Can't determine shortest path: Run RRT first using the run() method!")
            return []
        
        # Find all nodes whose is_near_goal property is true
        keys_nodes_near_goal = [key for key, value in self.vertices.items() if value[3] == True]
        vertices_near_goal = {key: value for key, value in self.vertices.items() if key in keys_nodes_near_goal}

        # If the list is empty, RRT* has not found a solution
        if not keys_nodes_near_goal:
            print("Can't determine shortest path: no solution was found (yet). Try to run RRT* again with a higher number of iterations")
            return []
        
        # From this list, find the node with the smallest cost/distance from start to goal
        key_node_with_smallest_cost = min(vertices_near_goal, key=lambda x: vertices_near_goal[x][2])

        # Search backwards from goal to start to find all the configs for the robot to follow
        key_child_node = key_node_with_smallest_cost
        key_parent_node = self.vertices[key_child_node][2]
        self.shortest_path_keys = []
        while (key_parent_node != 0):
            self.shortest_path_keys.append(key_child_node)

            # Retrieve the parent node key from the child node's dict entry
            key_parent_node = self.vertices[key_child_node][2]

            # Set the child node key as the parent node for the next iteration
            key_child_node = key_parent_node

        for key in self.shortest_path_keys:
            self.shortest_path_configs.append(self.vertices[key][0])

        # Nodes/configs are listed from goal to start due to backwards search, so they must be reversed
        self.shortest_path_keys.reverse()
        self.shortest_path_configs.reverse()

        return self.shortest_path_configs

    # Plots the result of RRT* (and optionally also RRT)
    def plot_results(self, also_plot_normal_RRT=False):
        num_iterations = 1 + also_plot_normal_RRT

        for iter in range(1, num_iterations+1):
            plt.ioff()
            plt.figure(iter+1)

            # Extract datapoints from node list: get x, y, etc of each node
            if iter == 1:
                keys_nodes = self.vertices.keys()
                x_values = [self.vertices[key_node][0][0] for key_node in keys_nodes]
                y_values = [self.vertices[key_node][0][1] for key_node in keys_nodes]
                q1_values = [self.vertices[key_node][0][2] for key_node in keys_nodes]
                q2_values = [self.vertices[key_node][0][3] for key_node in keys_nodes]
                q3_values = [self.vertices[key_node][0][4] for key_node in keys_nodes]
            elif iter == 2:
                keys_nodes = self.nodes.keys()
                x_values = [self.nodes[key_node][0] for key_node in keys_nodes]
                y_values = [self.nodes[key_node][1] for key_node in keys_nodes]
                q1_values = [self.nodes[key_node][2] for key_node in keys_nodes]
                q2_values = [self.nodes[key_node][3] for key_node in keys_nodes]
                q3_values = [self.nodes[key_node][4] for key_node in keys_nodes]
            
             # Plot obstacles
        
            # Plot obstacles
            for obstacle in self.obstacles:
                circle = plt.Circle((obstacle.position()[0], obstacle.position()[1]), 
                                    obstacle.radius(), 
                                    color='grey')
                plt.gca().add_patch(circle)
        
            # Plot nodes
            plt.scatter(x_values, y_values, s=15, label='Nodes')

            # Plot edges
            if iter == 1:
                for i, key_node in enumerate(keys_nodes):
                    # Retrieve the parent node key from the child node's dict entry
                    key_parent_node = self.vertices[key_node][2]

                    # get child node config
                    child_node = self.vertices[key_node][0]
                    # get parent node config
                    parent_node = self.vertices[key_parent_node][0]

                    # Plot an edge between child and parent
                    if i == 0:
                        plt.plot([parent_node[0], child_node[0]], 
                                [parent_node[1], child_node[1]], 
                                color='black', linewidth=1, label = "Edges")
                    else:
                        plt.plot([parent_node[0], child_node[0]], 
                                [parent_node[1], child_node[1]], 
                                color='black', linewidth=1)
            elif iter == 2:
                for key_node in self.edges.keys():
                    for edge in self.edges[key_node]:
                        key_next_node = edge[0]
                        key_current_node = key_node

                        next_node = self.nodes[key_current_node]
                        current_node = self.nodes[key_next_node]
                        plt.plot([current_node[0], next_node[0]], [current_node[1], next_node[1]], 'k-')

            # Plot shortest path
            if iter == 1:
                for i, key_child_node in enumerate(self.shortest_path_keys):
                    # Retrieve the parent node key from the child node's dict entry
                    key_parent_node = self.vertices[key_child_node][2]

                    # get child node and parent node config
                    child_node = self.vertices[key_child_node][0]
                    parent_node = self.vertices[key_parent_node][0]

                    # Plot an edge between child and parent
                    if i == 0:
                        plt.plot([parent_node[0], child_node[0]], 
                                [parent_node[1], child_node[1]], 
                                linewidth=1.5, color='m', label='Shortest path')
                    else:
                        plt.plot([parent_node[0], child_node[0]], 
                                [parent_node[1], child_node[1]], 
                                linewidth=1.5, color='m')

            # Plot start and goal points with different colors
            plt.scatter(x_values[0], y_values[0], 
                        color='red', s=100,
                        label='Start configuration')
            plt.scatter(self.goal_xyz[0], self.goal_xyz[1], 
                        color='green', s=100,
                        label='Goal configuration')

            # Plot goal margin
            margin = self.room["margin_of_closeness_to_goal"]
            square = plt.Rectangle((self.goal_xyz[0] - margin, self.goal_xyz[1] - margin), 
                                2 * margin, 2 * margin,
                                color='green', fill=False, 
                                label='Goal margin')
            plt.gca().add_patch(square)

            if iter == 1: plt.title('RRT* graph :)')
            if iter == 2: plt.title('RRT graph')
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.xlim([-10, 10])
            plt.ylim([-10, 10])
            if iter == 1: plt.legend(loc='upper left')
            plt.tight_layout()
            plt.show()

    # Runs the RRT* algorithm (and optionally also RRT) to create a graph/tree
    def run(self, initial_config, goal_xyz, sample_radius = 2, neighbor_radius=3, n_expansions=1000, stop_when_goal_reached=True, also_run_normal_RRT = False, animate_plot=False, verbose=False):
        # Initialize variables
        self.goal_xyz = goal_xyz
        self.initial_config = initial_config

        sample_radius = sample_radius
        neighbor_radius = neighbor_radius

        self.vertices = {
            # DATA STRUCTURE:
            # node ID: [config/coordinates, cost-to-reach-from-start, parent node ID, is_near_goal]
            0: [self.initial_config, 0, 0, False]
        }
        self.shortest_path_configs = []
        self.shortest_path_keys = []
        if also_run_normal_RRT:
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

        # Prepare plot for live animation
        if animate_plot:
            plt.figure(1)
            plt.ion() 
            self.animate()
            plt.show()

        # Main loop
        node_id = 1
        print(f"Now running RRT*! This will take a while...")
        for i in range(0, n_expansions):
            if verbose: print(f"i = {i}")
            
            # [1] Get a random configuration sample q_rand
            q_rand = self.get_random_sample()
            if debugRRT: print(f"i = {i}: Sample: {q_rand}")

            # [2] Find the nearest node this sample could be theoretically connected to
            #     If the sample is too far away from this node, discard this sample
            key_q_near = min(self.vertices.keys(), 
                             key=lambda node: self.get_distance_of_two_nodes(q_rand, self.vertices[node][0]))
            if debugRRT: print(f"i = {i}: Nearest node: {self.vertices[key_q_near][0]}")
            
            if self.get_distance_of_two_nodes(q_rand, self.vertices[key_q_near][0]) > sample_radius:
                if debugRRT: print(f"i = {i}: This sample would be too far away")
                continue

            # [3] Check if the robot will be in collision at any point between q_near and q_rand
            #     If there is collision, discard this sample
            if self.in_collision(self.vertices[key_q_near][0], q_rand):  
                if debugRRT: print(f"i = {i}: This sample would be in collision (possibly while moving)")
                continue

            # [-] Code below only runs when we compare the performance of RRT and RRT*
            if also_run_normal_RRT:
                # Add q_rand to the node dict
                key_q_rand = node_id
                self.nodes[key_q_rand] = q_rand

                # Add an edge to the edges dict
                # If the nearest node doesn't have edges yet, create an entry in the dict
                # else append the edge to the nearest node's list of edges
                distance_q_rand_to_q_near = self.get_distance_of_two_nodes(q_rand, self.nodes[key_q_near])
                if key_q_near in self.edges:
                    self.edges[key_q_near].append((key_q_rand, distance_q_rand_to_q_near))
                else:
                    self.edges[key_q_near] = [(key_q_rand, distance_q_rand_to_q_near)]

            # [-] Rename the node to make it clear that the next lines of code are part of RRT*
            q_new = q_rand

            # [4] Cost to reach q_new from start via q_near
            #     This is currently our "best guess" for shortest path from start to q_new
            cost_q_near_to_q_new = self.get_distance_of_two_nodes(self.vertices[key_q_near][0], q_new)
            cost_start_to_q_new_via_q_near = self.vertices[key_q_near][1] + cost_q_near_to_q_new

            # [5] Add an entry for q_new: [config, cost start to node, parent node, is_near_goal]
            key_q_new = node_id 
            self.vertices[key_q_new] = [q_new, cost_start_to_q_new_via_q_near, key_q_near, False]
            
            # [6] Find all neighbors of q_new, these are candidates for rewiring 
            keys_q_neighbors = [key_node for key_node in self.vertices.keys() 
                                if self.get_distance_of_two_nodes(self.vertices[key_node][0], q_new) <= neighbor_radius]
            if debugRRT: print(f"i = {i}: Neighbors: {keys_q_neighbors}")

            # [7] Check if q_new can be connected to the start via a neighboring node
            #     such that its cost is lower than its current connection
            for key_q_neighbor in keys_q_neighbors:
                current_total_cost = self.vertices[key_q_new][1]

                start_to_q_neighbor_cost = self.vertices[key_q_neighbor][1]
                q_neighbor_to_q_new_cost = self.get_distance_of_two_nodes(self.vertices[key_q_neighbor][0], q_new)
                new_total_cost = start_to_q_neighbor_cost + q_neighbor_to_q_new_cost
                
                if (new_total_cost < current_total_cost):
                    if debugRRT: print(f"i = {i}: Candidate found for rewiring q_new to a neighbor")
                    # Only rewire if this new connection is collision-free
                    if not self.in_collision(self.vertices[key_q_neighbor][0], q_new):
                        # Update q_new's cost and parent node ID
                        if debugRRT: print(f"i = {i}: Lower cost route found. Old cost: {current_total_cost}, new cost: {new_total_cost}")
                        self.vertices[key_q_new][2] = key_q_neighbor
                        self.vertices[key_q_new][1] = new_total_cost
            
            # [8] Check if any of the neighbors can be connected to the start via q_new 
            #     such that its cost is lower than its current connection
            for key_q_neighbor in keys_q_neighbors:
                current_total_cost = self.vertices[key_q_neighbor][1]

                start_to_q_new_cost = self.vertices[key_q_new][1]
                q_new_to_q_neighbor_cost = self.get_distance_of_two_nodes(q_new, self.vertices[key_q_neighbor][0])
                new_total_cost = start_to_q_new_cost + q_new_to_q_neighbor_cost

                if (new_total_cost < current_total_cost):
                    if debugRRT: print(f"i = {i}: Candidate found for rewiring a neighbor to q_new")
                    # Only rewire if this new connection is collision-free
                    if not self.in_collision(q_new, self.vertices[key_q_neighbor][0]):
                        # Update q_neighbor's cost and parent node ID
                        if debugRRT: print(f"i = {i}: Lower cost route found. Old cost: {current_total_cost}, new cost: {new_total_cost}")
                        self.vertices[key_q_neighbor][2] = key_q_new
                        self.vertices[key_q_neighbor][1] = new_total_cost

            # [9] If the sample is near the goal, set its property is_near_goal to True
            #     Additionally we break out of the loop early if the algorithm was set to stop once the goal is reached
            if self.config_is_near_goal(q_rand, goal_xyz):
                if debugRRT: print(f"i = {i}: This configuration is sufficiently close to the goal.")
                self.vertices[key_q_new][3] = True
                self.get_shortest_path()
                if stop_when_goal_reached:
                    print(f"Goal reached after {i} iterations. Stopping algorithm.")
                    break
            
            # Update live plot
            if animate_plot: self.animate()

            # Increment node ID
            node_id += 1
            if not debugRRT: print(f"node_id = {node_id}")

            if i == n_expansions: 
                print(f"Max number of iterations ({n_expansions}) reached. Stopping algorithm.")
        
        return

        

if __name__ == "__main__":
    # env params
    room = {
            "width": [-10, 10],
            "length": [-10, 10],
            "height": 8,
            "margin_of_closeness_to_goal": 1
            }
    
    rrt = RRTstar(l1=0.4, l2=0.7, l3=0.6, room=room)

    # start config and goal point
    config_s = [-9.5, -9.5, 0, 0, (1/2)*np.pi]
    goal = [9, 9, 1]

    # RRT parameters
    sample_radius = 3
    neighbor_radius = 3
    n_expansions = 10000
    also_run_normal_RRT = False
    stop_when_goal_reached = True
    animate_plot = True
    verbose = True

    rrt.run(config_s, goal, 
            sample_radius=sample_radius,
            neighbor_radius=neighbor_radius,
            n_expansions=n_expansions, 
            stop_when_goal_reached=stop_when_goal_reached, 
            also_run_normal_RRT=also_run_normal_RRT,
            animate_plot=animate_plot,
            verbose=verbose)

    print(f"Shortest path configs: {rrt.get_shortest_path()}")

    rrt.plot_results(also_run_normal_RRT)
    


