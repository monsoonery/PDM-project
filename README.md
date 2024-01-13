# PDM-project
A Python project for motion planning and simulation of a mobile manipulator in a static environment.

Authored by Mees Chammat, Quirine Engbers, Fin Kruseman Aretz and Eduardo Jerez\
January 13th, 2024\
RO47005 Planning & Decision Making - Project

## About this project
We programmed the RRT* algorithm in Python from scratch, and used _gym\_envs\_urdf_ by Max Spahn for simulating the robot. The mobile manipulator is represented by a simplified URDF. The obstacles are added into the simulation environment using objects from _mpscenes.obstacles_. 

Our implementation uses several classes in separate files to handle motion planning and simulation. They are stored in the `src` folder:
- `environment.py`: For creating and launching the simulation environment with the robot and obstacles
- `robot.py`: Contains the parameters and movement functions of the robot used in the simulation environment.
- `RRT_v3.py`: Contains all code related to running RRT*, including random sampling, collision detection, and generating graphs and plots.
- `obstacles.py`: Contains the definitions of _mpscenes.obstacles_ objects that are used by both `RRT_v3.py` and `environment.py`


All code files are readable on their own and have comments. 

## Getting started
### Prerequisites
This project has been built and tested on Windows 10, so consider these instructions to be for Windows only.

You need Python version 3.8 or higher. Also install the _gym\_envs\_urdf_ package first, either by [cloning from GitHub](https://github.com/maxspahn/gym_envs_urdf) or by running: 
```
pip3 install urdfenvs
```

## Run
### RRT* + simulation
**Inside the PDM-project folder**, open a command prompt window and run:
```
python main.py
```

This will launch a matplotlib plot window*, in which new nodes and edges should appear as time goes on. 

**Note:** depending on the chosen parameters, random chance, and the computing power of your device, the calculations can take anywhere from 3 minutes to 3 hours! As such, we added an option to stop the algorithm at any point _without terminating the entire Python script_. For this, make sure the cmd/Powershell window is in focus, and then press `Ctrl + C`. 

After RRT* is done running, a final plot will be shown, and the number of iterations, elapsed time, and configurations for the shortest path will be printed to the terminal.
- If a path was found, closing the plot window will automatically launch the simulation of the robot following this path . 
- If no path was found, closing the plot window open and close the simulation window immediately.

*only if `animate_plot = True`, see "Reproducing our test results" below

### Simulation only
**Inside the PDM-project folder**, open a command prompt window and run:
```
python main_simulation_only.py
```

This will launch a simulation window and should show our robot in its environment following a path.

To edit the path the robot follows, open `main_simulation_only.py` and change the list of configurations in the variable `results`. **We have included the resulting paths from our tests in the folder `configs` as .txt files. You can copy-paste the contexts of these .txt files into the variable `results`.**

## Reproducing our test results
The file `main.py` contains a list of parameters that can be edited to run the various cases mentioned in our report. The most relevant variables are:

1. `random.seed()`: The random seed used by random() in the RRT* code.
    - Trial 1: seed 6
    - Trial 2: seed 42
    - Trial 3: seed 14
2. `sample_radius`: For a random configuration sample, if there are no existing nodes within this radius, it is too far away and discarded immediately
3. `neighbor_radius`: Radius for finding neighbors that can be rewired to a given node
4. `n_expansions`: Number of (maximum) iterations to run the algorithm for
5. `stop_when_goal_reached`: If True, algorithm stops as soon as a valid solution is found instead of up to n_expansions
6. `animate_plot`: If True, plots RRT* graph nodes and edges in real-time

The following sections show the parameters we used for each test case:

### Case 1.1:
```
sample_radius = 3
neighbor_radius = 3 
n_expansions = 10000
stop_when_goal_reached = True
```

### Case 1.2:
```
sample_radius = 5
neighbor_radius = 3 
n_expansions = 10000
stop_when_goal_reached = True 
```

### Case 1.3:
```
sample_radius = 3
neighbor_radius = 5 
n_expansions = 10000
stop_when_goal_reached = True
```

### Case 2.1:
```
sample_radius = 5	
neighbor_radius = 3 	
n_expansions = 600	
stop_when_goal_reached = False	
```

**Additionally,** in src/obstacles.py, set `block_middle = True` to ensure the extra obstacles are INCLUDED for this case!

### Case 2.2:
```
sample_radius = 5	
neighbor_radius = 3 	
n_expansions = 600	
also_run_normal_RRT = False	
stop_when_goal_reached = False	
```

**Additionally,** in src/obstacles.py, set `block_middle = False` to ensure the extra obstacles are EXCLUDED for this case!