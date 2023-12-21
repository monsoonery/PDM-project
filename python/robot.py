import numpy as np

class Robot:
    def __init__(self, env, initial_config, l1, l2, l3):
        # Robot size parameters
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        
        self.env = env

        # Robot pose variables
        vel0 = np.array([0,0,0,0,0])
        pos0 = np.array(initial_config)
        self.ob, _ = self.env.reset(pos=pos0, vel=vel0)
        self.no_action = np.array([0,0,0,0,0])

        # Robot movement
        self.goal = [0,0,0,0,0]
        self.prev_goal = [0,0,0,0,0]
        self.reached = [False, False, False, False, False]
        self.speed = 1.5
        self.interval = 0.01
        self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
    
    def angle_interval(self, q):
        while q <= -np.pi: q = q+2*np.pi
        while q >= np.pi : q = q-2*np.pi

        return q
    
    # Move mobile platform step by step
    def move_x_y(self, distance_goal, time_to_reach_goal):
        if (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and 
            self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval): 
            # print("            2")
            x = 0 
            y = 0

        elif (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and not (self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval)):
            # print("            3")
            if self.position_ob[1] >= self.goal[1]+self.interval: y = -abs(distance_goal[1])/time_to_reach_goal
            elif self.position_ob[1] <= self.goal[1]-self.interval: y = abs(distance_goal[1])/time_to_reach_goal
            else: y = 0
            x = 0

        elif (self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval and not (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval)):
            # print("            4")
            if self.position_ob[0] >= self.goal[0]+self.interval: x = -abs(distance_goal[0])/time_to_reach_goal
            elif self.position_ob[0] <= self.goal[0]-self.interval: x = abs(distance_goal[0])/time_to_reach_goal
            else: x = 0
            y = 0
            
        else:
            if (self.position_ob[0] >= self.goal[0]+self.interval):
                x = -abs(distance_goal[0])/time_to_reach_goal

            elif (self.position_ob[0] <= self.goal[0]-self.interval):
                x = abs(distance_goal[0])/time_to_reach_goal

            if (self.position_ob[1] >= self.goal[1]+self.interval):
                y = -abs(distance_goal[1])/time_to_reach_goal

            elif (self.position_ob[1] <= self.goal[1]-self.interval):
                y = abs(distance_goal[1])/time_to_reach_goal
            # print("            5")

        return x,y
    
    # Move manipulator (arm) step by step
    def move_angle_continous(self, index, angle_between_goals, distance_goal, time_to_reach_goal):
        if (self.position_ob[index] >= self.goal[index]-self.interval and self.position_ob[index] <= self.goal[index]+self.interval):
            q = 0
            self.reached[index] = True
            # print("            6 = ", q)

        elif (angle_between_goals[index] >= np.pi and not self.reached[index]):
            q = -abs(distance_goal[index])/time_to_reach_goal
            # print("            7 = ", index)

        elif (angle_between_goals[index] <= np.pi and not self.reached[index]):
            q = abs(distance_goal[index])/time_to_reach_goal
            # print("            8 = ", index)
        
        elif (self.reached[index] and self.position_ob[index] >= self.goal[index]+self.interval): 
            q = -self.speed
            # print("            9 = ", index)

        elif (self.reached[index] and self.position_ob[index] <= self.goal[index]-self.interval): 
            q = self.speed
            # print("           10 = ", index)

        else: q = 0 # Should never happen
            
        return q
    
    def move_angle_revolute(self, index, distance_goal, time_to_reach_goal):
        if (self.position_ob[index] >= self.goal[index]-self.interval and self.position_ob[index] <= self.goal[index]+self.interval):
            q = 0
            # print("           11 = ", index)

        elif (self.position_ob[index] < self.goal[index]): 
            q = abs(distance_goal[index])/time_to_reach_goal
            # print("           12 = ", index)

        elif (self.position_ob[index] > self.goal[index]): 
            q = -abs(distance_goal[index])/time_to_reach_goal
            # print("           13 = ", index)

        else: q = 0 # Should never happen

        return q

    # Obtain distances needed for straight-line steering function
    def obtain_distances(self):
        distance_goal = []
        angle_between_goals = [0,0]

        # for x and y value: determine absolute distance and store in distance_goal
        for item in range(5):
            distance_goal.append(self.goal[item]-self.prev_goal[item])

        # determine if x or y is the largest distance to cross
        largest_distance = max(map(abs,distance_goal)) # for q1, q2, q3: determine absolute 'distance' and store in distance_goal
        time_to_reach_goal = largest_distance/self.speed

        # print(['%.2f' % elem for elem in distance_goal], largest_distance, time_to_reach_goal)
        # print(['%.2f' % elem for elem in self.goal])

        # for q1, q2, q3: determine absolute 'distance' and store in distance_goal
        for i in range(2,5):
            # keep angle between -pi and pi iterval
            self.goal[i] = self.angle_interval(self.goal[i])
            # take difference, must be between 0 and 2pi, to determine rotation direction
            temp = self.goal[i]-self.prev_goal[i]
            # if difference < 0, add 2pi
            while temp < 0:
                temp = temp + 2*np.pi
            angle_between_goals.append(temp)

        # time.sleep(1)
        return distance_goal, angle_between_goals, time_to_reach_goal
    
    # Main function that moves entire mobile manipulator from point one to point two (the "goal")
    def move_to_goal(self, goal):
        self.goal = goal
        self.reached = [False, False, False, False, False]
        distance_goal, angle_between_goals, time_to_reach_goal = self.obtain_distances()
        
        for _ in range(10000):
            self.position_ob = self.ob["robot_0"]["joint_state"]["position"]
            self.position_ob[2] = self.angle_interval(self.position_ob[2])
            self.position_ob[3] = self.angle_interval(self.position_ob[3])
            self.position_ob[4] = self.angle_interval(self.position_ob[4])
            print(['%.2f' % elem for elem in self.position_ob],['%.2f' % elem for elem in self.goal])
            
            if (self.position_ob[0] >= self.goal[0]-self.interval and self.position_ob[0] <= self.goal[0]+self.interval and 
                self.position_ob[1] >= self.goal[1]-self.interval and self.position_ob[1] <= self.goal[1]+self.interval and
                self.position_ob[2] >= self.goal[2]-self.interval and self.position_ob[2] <= self.goal[2]+self.interval and 
                self.position_ob[3] >= self.goal[3]-self.interval and self.position_ob[3] <= self.goal[3]+self.interval and
                self.position_ob[4] >= self.goal[4]-self.interval and self.position_ob[4] <= self.goal[4]+self.interval):
                self.ob, *_ = self.env.step(self.no_action)
                self.prev_goal = self.goal
                # print("            1")
                break

            x,y = self.move_x_y(distance_goal, time_to_reach_goal)
            q0 = self.move_angle_continous(2, angle_between_goals, distance_goal, time_to_reach_goal)
            q1 = self.move_angle_revolute(3, distance_goal, time_to_reach_goal)
            q2 = self.move_angle_revolute(4, distance_goal, time_to_reach_goal)
            
            self.ob, *_ = self.env.step(np.array([x,y,q0,q1,q2])) 
