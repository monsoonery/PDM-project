from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle

sphere1_dict = {
    "type": "sphere",
    "geometry": {"position": [2.0, 2.0, 1.0], "radius": 1.0},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
sphere1 = SphereObstacle(
    name="Sphere1", 
    content_dict=sphere1_dict
)

sphere2_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.0, -2.0, 1.0], "radius": 0.5},
    "rgba": [0.1, 0.8, 0.9, 1.0],
}
sphere2 = SphereObstacle(
    name="Sphere2", 
    content_dict=sphere2_dict
)

wall_length = 22
wall1_dict = {
        'type': 'box', 
        'geometry': {
            'position': [wall_length/2, 0.0, 0.4], 'width': wall_length, 'height': 0.8, 'length': 0.2
        },
        'high': {
            'position' : [wall_length/2, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.2,
        },
        'low': {
            'position' : [wall_length/2, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.2,
        },
    }
wall2_dict = {
        'type': 'box', 
        'geometry': {
            'position': [0.0, wall_length/2, 0.4], 'width': 0.2, 'height': 0.8, 'length': wall_length
        },
        'high': {
            'position' : [0.0, wall_length/2, 0.4],
            'width': 0.2,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, wall_length/2, 0.4],
            'width': 0.2,
            'height': 0.8,
            'length': wall_length,
        },
    }
wall3_dict = {
        'type': 'box', 
        'geometry': {
            'position': [0.0, -wall_length/2, 0.4], 'width': 0.2, 'height': 0.8, 'length': wall_length
        },
        'high': {
            'position' : [0.0, -wall_length/2, 0.4],
            'width': 0.2,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, -wall_length/2, 0.4],
            'width': 0.2,
            'height': 0.8,
            'length': wall_length,
        },
    }
wall4_dict = {
        'type': 'box', 
        'geometry': {
            'position': [-wall_length/2, 0.0, 0.4], 'width': wall_length, 'height': 0.8, 'length': 0.2
        },
        'high': {
            'position' : [-wall_length/2, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.2,
        },
        'low': {
            'position' : [-wall_length/2, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.2,
        },
    }
wall1 = BoxObstacle(
    name="Wall1",
    content_dict=wall1_dict
)
wall2 = BoxObstacle(
    name="Wall2",
    content_dict=wall2_dict
)
wall3 = BoxObstacle(
    name="Wall3",
    content_dict=wall3_dict
)
wall4 = BoxObstacle(
    name="Wall4",
    content_dict=wall4_dict
)

cylinder1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [2.0, -3.0, 0.5],
        "radius": 0.5,
        "height": 1.0,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
cylinder1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=cylinder1_dict
)

collision_obstacles = [sphere1, sphere2]
#[sphere1, sphere2]
# sphere1, sphere2, wall1, cylinder1
# TODO collision code can't check wall/cylinder shapes, only spheres 
# (obstacle.radius() doesnt work on BoxObstacle objects)

decorative_obstacles = [wall1, wall2, wall3, wall4]