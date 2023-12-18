from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle

obst1Dict = {
    "type": "sphere",
    "geometry": {"position": [2.0, 2.0, 1.0], "radius": 1.0},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
sphereObst1 = SphereObstacle(name="simpleSphere", content_dict=obst1Dict)

wall_length = 10
wall_obstacles_dicts = {
        'type': 'box', 
        'geometry': {
            'position': [-wall_length/2.0, 0.0, 0.4], 'width': wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    }
wall_obstacles = BoxObstacle(
    name="cylinder_obstacle",
    content_dict=wall_obstacles_dicts
)

cylinder_obstacle_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [2.0, -3.0, 0.5],
        "radius": 0.5,
        "height": 1.0,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
cylinder_obstacle = CylinderObstacle(
    name="cylinder_obstacle",
    content_dict=cylinder_obstacle_dict
)
