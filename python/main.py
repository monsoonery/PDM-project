import time
import matplotlib.pyplot as plt

import robot
import RRT_v2 as RRTstar
import environment

if __name__ == "__main__":
    l1 = 0.4
    l2 = 0.7
    l3 = 0.6

    # Create environment, robot and RRT algorithm objecs
    env = environment.Environment()
    coffeebot = robot.Robot(env.env, l1, l2, l3)
    rrt = RRTstar.RRTstar(l1, l2, l3)

    # RRT* parameters
    initial_config = [0, 0, 0, 0, 0]
    goal = [8, 8, 2]
    n_expansions = 1000

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
    
    for i in range(15):
        x = random.uniform(-10,10)
        y = random.uniform(-10,10)
        q0 = random.uniform(-np.pi,np.pi)
        q1 = random.uniform(-np.pi/2,np.pi/2)
        q2 = random.uniform(-2*np.pi/3,2*np.pi/3)
        coffeebot.move_to_goal(goal = [x, y, q0, q1 , q2])
        time.sleep(1)

    env.close_simulation()