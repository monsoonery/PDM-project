from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle

#Room obstacle 1
#========================================================================
location_table1 = [5,5,1]
width_table = 4
length_table = 2
table_top1 = {
    'type': 'box',
    'geometry': {
        'position': location_table1,  # Adjust the position as needed
        'width': width_table,  # Adjust the size as needed
        'height': 0.2,
        'length': length_table,
    },
}

table_leg1_1 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]+0.2-length_table/2, location_table1[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}

table_leg1_2 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]+0.2-length_table/2, location_table1[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}

table_leg1_3 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]-0.2+length_table/2, location_table1[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}

table_leg1_4 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]-0.2+length_table/2, location_table1[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}
#========================================================================
tabletop1 = BoxObstacle(
    name="Table1",
    content_dict=table_top1
)

tableleg1_1 = BoxObstacle(
    name="Table1",
    content_dict=table_leg1_1
)
tableleg1_2 = BoxObstacle(
    name="Table1",
    content_dict=table_leg1_2
)
tableleg1_3 = BoxObstacle(
    name="Table1",
    content_dict=table_leg1_3
)
tableleg1_4 = BoxObstacle(
    name="Table1",
    content_dict=table_leg1_4
)
#========================================================================
#========================================================================

#Room obstacle 2
#========================================================================
location_table2 = [-5,-3,1]
width_table = 2
length_table = 4
table_top2 = {
    'type': 'box',
    'geometry': {
        'position': location_table2,  # Adjust the position as needed
        'width': width_table,  # Adjust the size as needed
        'height': 0.2,
        'length': length_table,
    },
}

table_leg2_1 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]+0.2-length_table/2, location_table2[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}

table_leg2_2 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]+0.2-length_table/2, location_table2[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}

table_leg2_3 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]-0.2+length_table/2, location_table2[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}

table_leg2_4 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]-0.2+length_table/2, location_table2[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
}
#========================================================================
tabletop2 = BoxObstacle(
    name="Table1",
    content_dict=table_top2
)

tableleg2_1 = BoxObstacle(
    name="Table1",
    content_dict=table_leg2_1
)
tableleg2_2 = BoxObstacle(
    name="Table1",
    content_dict=table_leg2_2
)
tableleg2_3 = BoxObstacle(
    name="Table1",
    content_dict=table_leg2_3
)
tableleg2_4 = BoxObstacle(
    name="Table1",
    content_dict=table_leg2_4
)
#========================================================================
#========================================================================

#Room obstacle 2
#========================================================================
position_roundtable1 = [3, -6]
roundtablefoot1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable1[0], position_roundtable1[1], 0.15],
        "radius": 0.5,
        "height": 0.3,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
roundtableleg1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable1[0], position_roundtable1[1], 0.5],
        "radius": 0.1,
        "height": 1,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
roundtabletop1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable1[0], position_roundtable1[1], 1],
        "radius": 1.5,
        "height": 0.1,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
#========================================================================
roundtablefoot1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=roundtablefoot1_dict
)
roundtableleg1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=roundtableleg1_dict
)
roundtabletop1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=roundtabletop1_dict
)
#========================================================================
#========================================================================

#Room obstacle 4
#========================================================================
location_chair1 = [-2.5,-3,1]
chairseat1 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair1[0],location_chair1[1],0.3],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 0.6,
        'length': 0.7,
    },
    "rgba": [0.6, 0.3, 0.1, 1.0],
}

chairback1 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair1[0]+0.4,location_chair1[1], 0.8],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 1.6,
        'length': 0.1,
    },
    "rgba": [0.6, 0.3, 0.1, 1.0],
}
#========================================================================
chairseat1 = BoxObstacle(
    name="Table1",
    content_dict=chairseat1
)

chairback1 = BoxObstacle(
    name="Table1",
    content_dict=chairback1
)

#========================================================================
#========================================================================

#Room obstacle 6
#========================================================================
closet1_dict = {
    'type': 'box',
    'geometry': {
        'position': [-4,6,1.5],  # Adjust the position as needed
        'width': 2,  # Adjust the size as needed
        'height': 3,
        'length': 0.7,
    },
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
#========================================================================
closet1 = BoxObstacle(
    name="Table1",
    content_dict=closet1_dict
)
#========================================================================
#========================================================================

#Room obstacle 7
#========================================================================
closet2_dict = {
    'type': 'box',
    'geometry': {
        'position': [-6,4,1.5],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 3,
        'length': 2,
    },
    "rgba": [0.3, 0.5, 0.6, 1.0],
}
#========================================================================
closet2 = BoxObstacle(
    name="Table1",
    content_dict=closet2_dict
)
#========================================================================
#========================================================================

#Room obstacle 2
#========================================================================
position_roundtable2 = [-6, 8]
roundtablefoot2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable2[0], position_roundtable2[1], 0.15],
        "radius": 0.5,
        "height": 0.3,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
roundtableleg2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable2[0], position_roundtable2[1], 0.5],
        "radius": 0.1,
        "height": 1,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
roundtabletop2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable2[0], position_roundtable2[1], 1],
        "radius": 1.5,
        "height": 0.1,
    },
    "rgba": [0.1, 0.3, 0.3, 1.0],
}
#========================================================================
roundtablefoot2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=roundtablefoot2_dict
)
roundtableleg2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=roundtableleg2_dict
)
roundtabletop2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=roundtabletop2_dict
)
#========================================================================
#========================================================================

#Room obstacle 8
#========================================================================
location_chair2 = [6.5,-6,1]
chairseat2 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair2[0],location_chair2[1],0.3],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 0.6,
        'length': 0.7,
    },
    "rgba": [0.6, 0.3, 0.1, 1.0],

}

chairback2 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair2[0]+0.4,location_chair2[1], 0.8],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 1.6,
        'length': 0.1,
    },
    "rgba": [0.6, 0.3, 0.1, 1.0],

}
#========================================================================
chairseat2 = BoxObstacle(
    name="Table1",
    content_dict=chairseat2
)

chairback2 = BoxObstacle(
    name="Table1",
    content_dict=chairback2
)

#========================================================================
#========================================================================


sphere1_dict = {
    "type": "sphere",
    "geometry": {"position": [2.0, 2.0, 1.0], "radius": 1.0},
    "rgba": [0.6,0.45,0.29, 1.0],
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

collision_obstacles = []

#add table 1
collision_obstacles += [tabletop1, tableleg1_1, tableleg1_2, tableleg1_3, tableleg1_4]

#add table 2
collision_obstacles += [tabletop2, tableleg2_1, tableleg2_2, tableleg2_3, tableleg2_4]

#add table 3
collision_obstacles += [roundtablefoot1, roundtableleg1, roundtabletop1]

#add table 4
collision_obstacles += [roundtablefoot2, roundtableleg2, roundtabletop2]

#add chair 1
collision_obstacles += [chairseat1, chairback1]

#add chair 2
collision_obstacles += [chairseat2, chairback2]

#add closet 1 and 2
collision_obstacles += [closet1, closet2]

#[sphere1, sphere2]
# sphere1, sphere2, wall1, cylinder1
# TODO collision code can't check wall/cylinder shapes, only spheres 
# (obstacle.radius() doesnt work on BoxObstacle objects)

decorative_obstacles = [wall1, wall2, wall3, wall4]
