import numpy as np
import time

from src import robot
from src import environment

"""
This file contains only the code required to simulate the robot, no motion planning algorithm is executed here.
Enter a list of configurations (obtained from the motion planner) in the variable "result" below to run a simulation.
A list from one of our experiments has been entered already to get you started.
Other lists can be found in the text file inside the folder "configs"
"""

if __name__ == "__main__":
     # Enter a list of configurations for the robot here!
     result = [
          [-8.167698243551119, -9.005704657790966, 1.6145609717192058, -0.8655022727358632, 1.6690978207162517], 
          [-7.81280222018831, -8.870422580185593, 1.1783109372396956, 0.8866355062493931, -1.6159495905506898], 
          [-6.525642862025157, -8.170753728711002, 3.2940744075272734, -0.9879767107918849, 2.093572669317726], 
          [-5.437644773014426, -6.975964182662519, 0.813369533019164, 0.952114875166203, -0.14352990665422571], 
          [-4.003418054429144, -6.178960316121768, 3.3285936960508877, 0.1887495062998974, 1.2743127228318563], 
          [-2.4535642208384525, -5.5321489153417005, 5.7861518225812025, -1.4868138486570346, -1.4736974991888347], 
          [-0.819739370622953, -5.036832791162383, 2.178828693350878, -1.480813904064572, 1.5212572430219655], 
          [1.5703710519528116, -3.7083198874409042, 0.13436375447283827, -0.6662313256252961, -0.6507551964904319], 
          [3.177096399999826, -3.268495917779111, 2.559161831107811, 0.48498544200123606, -0.7886807384796393], 
          [4.773163590821605, -2.154932517921446, 3.962394057284899, -0.805256722925922, -2.0649430491505423], 
          [7.005843862636489, -0.128190488683396, 6.110123525921024, 1.0780540223799795, -0.07578696325792356], 
          [7.292705652829451, 0.6987927672590857, 4.373944604582231, 0.43414056170498094, -1.7361010308380398], 
          [8.123264230749001, 2.1354012390180728, 3.1324836941617544, 0.2546486853312311, 0.8213081346669946], 
          [7.993544500644461, 4.3239294202309235, 3.778836685463981, 1.1935066259655809, -1.51879268708107], 
          [9.457994657792064, 6.638686931167772, 4.502638006660721, -1.494263571577534, 1.8421455480807856]
          ]
     
     # Manipulator parameters
     l1 = 0.4
     l2 = 0.7
     l3 = 0.6

     # Start configuration and goal xyz-coordinates
     initial_config = [-9.5, -9.5, 0, 0, (1/2)*np.pi]
     goal = [9, 9, 1]

     # Create environment and robot object for simulation
     env = environment.Environment()
     coffeebot = robot.Robot(env.env, initial_config, l1, l2, l3)

     
     # Follow the planned path in the simulation
     if result:
          for config in result:
               time.sleep(0.1)
               print(f"current config: {config}")
               coffeebot.move_to_goal(goal = config)
     time.sleep(2)
     env.close_simulation()

