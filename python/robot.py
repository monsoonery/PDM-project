import numpy as np
import random
import time

class Robot:
    def __init__(self, env, l1, l2, l3):
        # Robot size parameters
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        
        self.env = env

        # Robot pose variables
        vel0 = np.array([0,0,0,0,0])
        pos0 = np.array([0,0,0,0,0])
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        self.no_action = np.array([0,0,0,0,0])

        # Robot movement
        self.goal = [0,0,0,0,0]
        self.prev_goal = [0,0,0,0,0]
        self.reached = [False, False, False, False, False]
        self.speed = 0.5
        self.interval = 0.03
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]

        # Robot collision flags
        self.self_collision = False
    
    def angle_interval(self, q):
        while q <= -np.pi: q = q+2*np.pi
        while q >= np.pi : q = q-2*np.pi

        return q
    
    # Move mobile platform step by step
    def move_x_y(self, distance_goal, largest_distance):
        if (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and 
            self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval): 
            # print("            2")
            x = 0 
            y = 0

        elif (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and not (self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval)):
            # print("            3")
            if self.position_ob[1] >= self.goal[1]-self.interval: y = -self.speed
            elif self.position_ob[1] <= self.goal[1]+self.interval: y = self.speed
            else: y = 0
            x = 0

        elif (self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval and not (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval)):
            # print("            4")
            if self.position_ob[0] >= self.goal[0]-self.interval: x = -self.speed
            elif self.position_ob[0] <= self.goal[0]+self.interval: x = self.speed
            else: x = 0
            y = 0
            
        else:
            # print("            5")
            x = self.speed*distance_goal[0]/largest_distance
            y = self.speed*distance_goal[1]/largest_distance
        return x,y
    
    # Move manipulator (arm) step by step
    def move_angle_continous(self, index, angle_between_goals, printen = False):
        if (self.position_ob[index] >= self.goal[index]-self.interval and self.position_ob[index] <= self.goal[index]+self.interval):
            q = 0
            self.reached[index] = True
            # if printen == True: print("            6 = ", q)

        elif (angle_between_goals >= np.pi and not self.reached[index]):
            q = -self.speed
            # if printen == True: print("            7 = ", q)

        elif (angle_between_goals <= np.pi and not self.reached[index]):
            q = self.speed
            # if printen == True: print("            8 = ", q)
        
        elif (self.reached[index] and self.position_ob[index] >= self.goal[index]): 
            q = -self.speed
            # if printen == True: print("            9 = ", q)

        elif (self.reached[index] and self.position_ob[index] <= self.goal[index]): 
            q = self.speed
            # if printen == True: print("           10 = ", q)

        else: q = 0 # Should never happen
            
        return q
    
    def move_angle_revolute(self, index):
        if (self.position_ob[index] >= self.goal[index]-self.interval and self.position_ob[index] <= self.goal[index]+self.interval):
            q = 0
        elif (self.position_ob[index] < self.goal[index]): q = self.speed
        elif (self.position_ob[index] > self.goal[index]): q = -self.speed
        else: q = 0 # Should never happen

        return q

    # Obtain distances needed for straight-line steering function
    def obtain_distances(self):
        distance_goal = []

        # for x and y value: determine absolute distance and store in distance_goal
        for item in range(2):
            distance_goal.append(self.goal[item]-self.prev_goal[item])

        # determine if x or y is the largest distance to cross
        largest_distance = max(map(abs,distance_goal)) 

        # for q1, q2, q3: determine absolute 'distance' and store in distance_goal
        for i in range(2,5):
            # keep angle between -pi and pi iterval
            self.goal[i] = self.angle_interval(self.goal[i])
            # take difference, must be between 0 and 2pi, to determine rotation direction
            temp = self.goal[i]-self.prev_goal[i]
            # if difference < 0, add 2pi
            while temp < 0:
                temp = temp + 2*np.pi
            distance_goal.append(temp)

        return distance_goal, largest_distance
    
    # Main function that moves entire mobile manipulator from point one to point two (the "goal")
    def move_to_goal(self, goal):
        self.goal = goal
        self.reached = [False, False, False, False, False]
        distance_goal, largest_distance = self.obtain_distances()
        
        self.self_collision = False
        #self.check_self_collision_goal()

        for _ in range(10000):
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            self.position_ob[2] = self.angle_interval(self.position_ob[2])
            self.position_ob[3] = self.angle_interval(self.position_ob[3])
            self.position_ob[4] = self.angle_interval(self.position_ob[4])
            #print(self.position_ob, self.goal)
            #self.check_self_collision_pose()
            
            if (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and 
                self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval and
                self.position_ob[2] >= self.goal[2]-self.interval and self.position_ob[2] <= self.goal[2]+self.interval and 
                self.position_ob[3] >= self.goal[3]-self.interval and self.position_ob[3] <= self.goal[3]+self.interval and
                self.position_ob[4] >= self.goal[4]-self.interval and self.position_ob[4] <= self.goal[4]+self.interval):
                self.ob, *_ = self.env.step(self.no_action)
                self.prev_goal = self.goal
                # print("            1")
                break

            if (self.self_collision == True):
                self.ob, *_ = self.env.step(self.no_action)
                break

            x,y = self.move_x_y(distance_goal, largest_distance)
            q0 = self.move_angle_continous(2, distance_goal[2])
            q1 = self.move_angle_revolute(3)
            q2 = self.move_angle_revolute(4)
            
            self.ob, *_ = self.env.step(np.array([x,y,q0,q1,q2])) 
