# PDM-project
A Python project for motion planning and simulation of a mobile manipulator in a static environment.

Authored by Mees Chammat, Quirine Engbers, Fin Kruseman Aretz and Eduardo Jerez\
January 13th, 2024\
RO47005 Planning & Decision Making - Project

## About this project
We programmed the RRT* algorithm in Python from scratch, and used _gym_envs_urdf_ for simulating the robot.

## Getting started
### Prerequisites
You need Python version 3.8 or higher

Also install the _gym_envs_urdf_ package first: 
```
pip3 install urdfenvs
```

## Run
### RRT* + simulation
Inside the PDM-project folder, open a command prompt window and run:
```
python main.py
```
### Simulation only
Inside the PDM-project folder, open a command prompt window and run:
```
python main_simulation_only.py
```

This will launch a pybullet window and should show our robot in its environment following a path.
To edit the path the robot follows, open `python main_simulation_only.py` and edit the list of configurations in the variable `results`. For convenience, we have included the paths from our tests in the folder `configs` as .txt files. You can copy-paste the contexts of these .txt files into the variable `results`.

## Reproducing our test results
The file `main.py` contains a list of parameters that can be edited to run the various cases mentioned in our report. The most relevant variables are:

1. `random.seed(14)`: The random seed used by random() in the RRT* code.
    - Trial 1: seed 6
    - Trial 2: seed 42
    - Trial 3: seed 14
2. `sample_radius`: For a random configuration sample, if there are no existing nodes within this radius, it is too far away and discarded immediately
3. `neighbor_radius`: Radius for finding neighbors that can be rewired to a given node
4. `n_expansions = 1000`: Number of (maximum) iterations to run the algorithm for
5. `stop_when_goal_reached = True`: If True, algorithm stops as soon as a valid solution is found instead of up to n_expansions
6. `animate_plot`: If True, plots RRT* graph nodes and edges in real-time

For transparency, the following sections show the parameters we used for each test case:

### Case 1.1:
sample_radius = 3
neighbor_radius = 3 
n_expansions = 10000
also_run_normal_RRT = False
stop_when_goal_reached = True

### Case 1.2:
sample_radius = 5
neighbor_radius = 3 
n_expansions = 10000
also_run_normal_RRT = False
stop_when_goal_reached = True 

### Case 1.3:
sample_radius = 3
neighbor_radius = 3 
n_expansions = 10000
also_run_normal_RRT = False
stop_when_goal_reached = True

### Case 2.1:
sample_radius = 5	
neighbor_radius = 3 	
n_expansions = 600	
also_run_normal_RRT = False	
stop_when_goal_reached = False	
animate_plot = True 	
verbose = False	

**Additionally,** in obstacles.py, set `block_middle = True` to ensure the extra obstacles are INCLUDED for this case!

### Case 2.2:
sample_radius = 5	
neighbor_radius = 3 	
n_expansions = 600	
also_run_normal_RRT = False	
stop_when_goal_reached = False	
animate_plot = True 	
verbose = False	

**Additionally,** in obstacles.py, set `block_middle = False` to ensure the extra obstacles are EXCLUDED for this case!