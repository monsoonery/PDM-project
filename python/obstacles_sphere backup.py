from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle

#Room obstacle 1
#========================================================================
table1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [4,5,1], "radius": 1.0},
    "rgba": [0.1,0.6,0.29, 1.0],
}
table1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [6,5,1], "radius": 1.0},
    "rgba": [0.1,0.6,0.29, 1.0],
}
table1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [5,5,1], "radius": 1.0},
    "rgba": [0.1,0.6,0.29, 1.0],
}
#========================================================================
table1_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=table1_1_dict
)
table1_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=table1_2_dict
)
table1_3 = SphereObstacle(
    name="Sphere1", 
    content_dict=table1_3_dict
)
#========================================================================

#Room obstacle 2
#========================================================================
table2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-5,-2,1], "radius": 1.0},
    "rgba": [0.1,0.6,0.29, 1.0],
}
table2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-5,-3,1], "radius": 1.0},
    "rgba": [0.1,0.6,0.29, 1.0],
}
table2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-5,-4,1], "radius": 1.0},
    "rgba": [0.1,0.6,0.29, 1.0],
}
#========================================================================
table2_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=table2_1_dict
)
table2_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=table2_2_dict
)
table2_3 = SphereObstacle(
    name="Sphere1", 
    content_dict=table2_3_dict
)
#========================================================================

#Room obstacle 3
#========================================================================
table3_1_dict = {
    "type": "sphere",
    "geometry": {"position": [3,-6,0.5], "radius": 1.5},
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
#========================================================================
table3_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=table3_1_dict
)
#========================================================================

#Room obstacle 4
#========================================================================
chairsphere1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.5,-3,0.35], "radius": 0.7},
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
chairsphere1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.5,-3,1], "radius": 0.7},
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
chairsphere1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.5,-3,1.35], "radius": 0.7},
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
#========================================================================
chairsphere1_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere1_1_dict
)
chairsphere1_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere1_2_dict
)
chairsphere1_3= SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere1_3_dict
)
#========================================================================

#Room obstacle 6
#========================================================================
closetsphere1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6,0.7], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6+1.4,0.7], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6-1.4,0.7], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_4_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6,0.7+1.4], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_5_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6+1.4,0.7+1.4], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_6_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6-1.4,0.7+1.4], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_7_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6,0.7+1.4*2], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_8_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6+1.4,0.7+1.4*2], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere1_9_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6-1.4,0.7+1.4*2], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
#========================================================================
closetsphere1_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_1_dict
)
closetsphere1_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_2_dict
)
closetsphere1_3= SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_3_dict
)
closetsphere1_4 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_4_dict
)
closetsphere1_5 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_5_dict
)
closetsphere1_6= SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_6_dict
)
closetsphere1_7 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_7_dict
)
closetsphere1_8 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_8_dict
)
closetsphere1_9= SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere1_9_dict
)
#========================================================================

#Room obstacle 7
#========================================================================
closetsphere2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-6,4,0.7], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-6+1.4,4,0.7], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-6-1.4,4,0.7], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_4_dict = {
    "type": "sphere",
    "geometry": {"position": [-6,4,0.7+1.4], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_5_dict = {
    "type": "sphere",
    "geometry": {"position": [-6+1.4,4,0.7+1.4], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_6_dict = {
    "type": "sphere",
    "geometry": {"position": [-6-1.4,4,0.7+1.4], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_7_dict = {
    "type": "sphere",
    "geometry": {"position": [-6,4,0.7+1.4*2], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_8_dict = {
    "type": "sphere",
    "geometry": {"position": [-6+1.4,4,0.7+1.4*2], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
closetsphere2_9_dict = {
    "type": "sphere",
    "geometry": {"position": [-6-1.4,4,0.7+1.4*2], "radius": 0.7},
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
#========================================================================
closetsphere2_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_1_dict
)
closetsphere2_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_2_dict
)
closetsphere2_3= SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_3_dict
)
closetsphere2_4 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_4_dict
)
closetsphere2_5 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_5_dict
)
closetsphere2_6= SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_6_dict
)
closetsphere2_7 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_7_dict
)
closetsphere2_8 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_8_dict
)
closetsphere2_9= SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere2_9_dict
)
#========================================================================

#Room obstacle 2
#========================================================================
table4_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-6,8,0.5], "radius": 1.5},
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
#========================================================================
table4_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=table4_1_dict
)
#========================================================================

#Room obstacle 8
#========================================================================
chairsphere2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [6.5,-6,0.35], "radius": 0.7},
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
chairsphere2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [6.5,-6,1], "radius": 0.7},
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
chairsphere2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [6.5,-6,1.35], "radius": 0.7},
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
#========================================================================
chairsphere2_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere2_1_dict
)
chairsphere2_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere2_2_dict
)
chairsphere2_3= SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere2_3_dict
)
#========================================================================


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

collision_obstacles = []

#add table 1
collision_obstacles += [table1_1, table1_2, table1_3]

#add table 2
collision_obstacles += [table2_1, table2_2, table2_3]

#add table 3
collision_obstacles += [table3_1]

#add table 4
collision_obstacles += [table4_1]

#add chair 1
collision_obstacles += [chairsphere1_1, chairsphere1_2, chairsphere1_3]

#add chair 2
collision_obstacles += [chairsphere2_1, chairsphere2_2, chairsphere2_3]

#add closet 1 and 2
collision_obstacles += [closetsphere1_1, closetsphere1_2, closetsphere1_3, closetsphere1_4, closetsphere1_5, closetsphere1_6, closetsphere1_7, closetsphere1_8, closetsphere1_9]
collision_obstacles += [closetsphere2_1, closetsphere2_2, closetsphere2_3, closetsphere2_4, closetsphere2_5, closetsphere2_6, closetsphere2_7, closetsphere2_8, closetsphere2_9]

#[sphere1, sphere2]
# sphere1, sphere2, wall1, cylinder1
# TODO collision code can't check wall/cylinder shapes, only spheres 
# (obstacle.radius() doesnt work on BoxObstacle objects)

decorative_obstacles = [wall1, wall2, wall3, wall4]
