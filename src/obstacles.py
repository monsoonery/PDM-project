from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle


# toggles
block_middle = False # toggle extra obstacles
display_objects = True # toggle displaying obstacles in sim environment

collision_obstacles = []

# collision sphere colors (red with transparency)
alpha_value = 0.3 
color_spheres_squaretables = [1, 0, 0, alpha_value]
color_spheres_roundtables = [1,0,0, alpha_value]
color_spheres_chairs = [1, 0, 0, alpha_value]
color_spheres_closets = [1, 0, 0, alpha_value]

# obstacle colors
color_squaretables = [0.2, 0.1, 0.0, 1.0]
color_roundtables = [0.1, 0.3, 0.3, 1.0]
color_chairs = [0.2, 0.1, 0.3, 1.0]
color_closets = [0.3, 0.1, 0.4, 1.0]
color_lamps = [0.6, 0.6, 0, 1.0]

"""====================================================================================================
================================= Obstacle collision spheres ==========================================
===================================================================================================="""

# Table 1
#========================================================================
table1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [5,4,1], "radius": 1.5},
    "rgba": color_spheres_squaretables,
}
table1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [5,6,1], "radius": 1.5},
    "rgba": color_spheres_squaretables,
}
table1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [5,5,1], "radius": 1.5},
    "rgba": color_spheres_squaretables,
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

# Table 2
#========================================================================
table2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,-3,1], "radius": 1.5},
    "rgba": color_spheres_squaretables,
}
table2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-5,-3,1], "radius": 1.5},
    "rgba": color_spheres_squaretables,
}
table2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-6,-3,1], "radius": 1.5},
    "rgba": color_spheres_squaretables,
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

# Table 3
#========================================================================
table3_1_dict = {
    "type": "sphere",
    "geometry": {"position": [3,-6,0.5], "radius": 1.8},
    "rgba": color_spheres_roundtables,
}
#========================================================================
table3_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=table3_1_dict
)
#========================================================================

# Table 4
#========================================================================
table4_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-6,8,0.5], "radius": 1.8},
    "rgba": color_spheres_roundtables,
}
#========================================================================
table4_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=table4_1_dict
)
#========================================================================

# Chair 1
#========================================================================
chairsphere1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.5,-3,0.35], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
chairsphere1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.5,-3,1], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
chairsphere1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-2.5+0.4,-3,1.2], "radius": 0.7},
    "rgba": color_spheres_chairs,
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

# Chair 2
#========================================================================
chairsphere2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [6.5,-6,0.35], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
chairsphere2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [6.5,-6,1], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
chairsphere2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [6.5+0.4,-6,1.2], "radius": 0.7},
    "rgba": color_spheres_chairs,
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

# Closet 1
#========================================================================
closetsphere1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6-0.5,0.7], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6+0.5,0.7], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6-0.5,0.7+1.4], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere1_4_dict = {
    "type": "sphere",
    "geometry": {"position": [-4,6+0.5,0.7+1.4], "radius": 1},
    "rgba": color_spheres_closets,
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
#========================================================================

# Closet 2
#========================================================================
closetsphere2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-6-0.5,4,0.7], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-6+0.5,4,0.7], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-6-0.5,4,0.7+1.4], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere2_4_dict = {
    "type": "sphere",
    "geometry": {"position": [-6+0.5,4,0.7+1.4], "radius": 1},
    "rgba": color_spheres_closets,
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
#========================================================================

# Closet 3
#========================================================================
closetsphere3_1_dict = {
    "type": "sphere",
    "geometry": {"position": [2.5-0.5,0,0.7], "radius": 1},
    "rgba": color_spheres_closets,
}
closetsphere3_2_dict = {
    "type": "sphere",
    "geometry": {"position": [2.5+0.5,0,0.7], "radius": 1},
    "rgba": color_spheres_closets,
}
#========================================================================
closetsphere3_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere3_1_dict
)
closetsphere3_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=closetsphere3_2_dict
)
#========================================================================

# Chair 3
#========================================================================
chairsphere3_1_dict = {
    "type": "sphere",
    "geometry": {"position": [0,-5,0.35], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
chairsphere3_2_dict = {
    "type": "sphere",
    "geometry": {"position": [0,-5,1], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
chairsphere3_3_dict = {
    "type": "sphere",
    "geometry": {"position": [0,-5,1.35], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
#========================================================================
chairsphere3_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere3_1_dict
)
chairsphere3_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere3_2_dict
)
chairsphere3_3= SphereObstacle(
    name="Sphere1", 
    content_dict=chairsphere3_3_dict
)
#========================================================================

# Standing lamp 1
#========================================================================
lampsphere1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [5,0,0.7], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
lampsphere1_2_dict = {
    "type": "sphere",
    "geometry": {"position": [5,0,1.4], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
lampsphere1_3_dict = {
    "type": "sphere",
    "geometry": {"position": [5,0,1.4+0.4], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
#========================================================================
lampsphere1_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=lampsphere1_1_dict
)
lampsphere1_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=lampsphere1_2_dict
)
lampsphere1_3= SphereObstacle(
    name="Sphere1", 
    content_dict=lampsphere1_3_dict
)
#========================================================================

# Standing lamp 2
#========================================================================
lampsphere2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-1,-3.5,0.7], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
lampsphere2_2_dict = {
    "type": "sphere",
    "geometry": {"position": [-1,-3.5,1.4], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
lampsphere2_3_dict = {
    "type": "sphere",
    "geometry": {"position": [-1,-3.5,1.4+0.4], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
#========================================================================
lampsphere2_1 = SphereObstacle(
    name="Sphere1", 
    content_dict=lampsphere2_1_dict
)
lampsphere2_2 = SphereObstacle(
    name="Sphere1", 
    content_dict=lampsphere2_2_dict
)
lampsphere2_3= SphereObstacle(
    name="Sphere1", 
    content_dict=lampsphere2_3_dict
)
#========================================================================

# Hanging lamp 1
#========================================================================
hanglampsphere1_1_dict = {
    "type": "sphere",
    "geometry": {"position": [-5.0, -7.5, 2], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
#========================================================================
hanglampsphere1_1= SphereObstacle(
    name="Hanglamp Sphere1", 
    content_dict=hanglampsphere1_1_dict
)
#========================================================================

# Hanging lamp 2
#========================================================================
hanglampsphere2_1_dict = {
    "type": "sphere",
    "geometry": {"position": [0.0, 5.0, 2], "radius": 0.7},
    "rgba": color_spheres_chairs,
}
#========================================================================
hanglampsphere2_1= SphereObstacle(
    name="Hanglamp Sphere2", 
    content_dict=hanglampsphere2_1_dict
)
#========================================================================


"""====================================================================================================
============================================ Obstacle models ==========================================
===================================================================================================="""
# Room walls
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
#========================================================================
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
#========================================================================

# Table 1
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
    "rgba": color_squaretables,

}
table_leg1_1 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]+0.2-length_table/2, location_table1[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
}
table_leg1_2 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]+0.2-length_table/2, location_table1[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
}
table_leg1_3 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]-0.2+length_table/2, location_table1[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
}
table_leg1_4 = {
    'type': 'box',
    'geometry': {
        'position': [location_table1[0]-0.2+length_table/2, location_table1[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
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

# Table 2
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
        "rgba": color_squaretables,
}
table_leg2_1 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]+0.2-length_table/2, location_table2[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
}
table_leg2_2 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]+0.2-length_table/2, location_table2[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
}
table_leg2_3 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]-0.2+length_table/2, location_table2[1]-0.2+width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
}
table_leg2_4 = {
    'type': 'box',
    'geometry': {
        'position': [location_table2[0]-0.2+length_table/2, location_table2[1]+0.2-width_table/2, 0.5],  # Adjust the position as needed
        'width': 0.2,  # Adjust the size as needed
        'height': 1.0,
        'length': 0.2,
    },
        "rgba": color_squaretables,
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

# Table 3
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
    "rgba": color_roundtables,
}
roundtableleg1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable1[0], position_roundtable1[1], 0.5],
        "radius": 0.1,
        "height": 1,
    },
    "rgba": color_roundtables,
}
roundtabletop1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable1[0], position_roundtable1[1], 1],
        "radius": 1.5,
        "height": 0.1,
    },
    "rgba": color_roundtables,
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

# Table 4
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
    "rgba": color_roundtables,
}
roundtableleg2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable2[0], position_roundtable2[1], 0.5],
        "radius": 0.1,
        "height": 1,
    },
    "rgba": color_roundtables,
}
roundtabletop2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_roundtable2[0], position_roundtable2[1], 1],
        "radius": 1.5,
        "height": 0.1,
    },
    "rgba": color_roundtables,
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

# Chair 1
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
    "rgba": color_chairs,
}
chairback1 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair1[0]+0.4,location_chair1[1], 0.8],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 1.6,
        'length': 0.1,
    },
    "rgba": color_chairs,
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

# Chair 2
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
    "rgba": color_chairs,

}
chairback2 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair2[0]+0.4,location_chair2[1], 0.8],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 1.6,
        'length': 0.1,
    },
    "rgba": color_chairs,

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

# Chair 3
#========================================================================
location_chair3 = [0,-5]
chairseat3 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair3[0],location_chair3[1],0.3],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 0.6,
        'length': 0.7,
    },
    "rgba": color_chairs,

}
chairback3 = {
    'type': 'box',
    'geometry': {
        'position': [location_chair3[0]-0.4,location_chair3[1], 0.8],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 1.6,
        'length': 0.1,
    },
    "rgba": color_chairs,

}
#========================================================================
chairseat3 = BoxObstacle(
    name="Table1",
    content_dict=chairseat3
)
chairback3 = BoxObstacle(
    name="Table1",
    content_dict=chairback3
)
#========================================================================


# Closet 1
#========================================================================
closet1_dict = {
    'type': 'box',
    'geometry': {
        'position': [-4,6,1.5],  # Adjust the position as needed
        'width': 2,  # Adjust the size as needed
        'height': 3,
        'length': 0.7,
    },
    "rgba": color_closets,
}
#========================================================================
closet1 = BoxObstacle(
    name="Table1",
    content_dict=closet1_dict
)
#========================================================================

# Closet 2
#========================================================================
closet2_dict = {
    'type': 'box',
    'geometry': {
        'position': [-6,4,1.5],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 3,
        'length': 2,
    },
    "rgba": color_closets,
}
#========================================================================
closet2 = BoxObstacle(
    name="Table1",
    content_dict=closet2_dict
)
#========================================================================

# Closet 3
#========================================================================
closet3_dict = {
    'type': 'box',
    'geometry': {
        'position': [2.5,0,0.5],  # Adjust the position as needed
        'width': 0.7,  # Adjust the size as needed
        'height': 1,
        'length': 2,
    },
    "rgba": color_closets,
}
#========================================================================
closet3 = BoxObstacle(
    name="Table1",
    content_dict=closet3_dict
)
#========================================================================

# Standing lamp 1
#========================================================================
position_lamp1 = [5, 0]
lamp1_1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp1[0], position_lamp1[1], 0.05],
        "radius": 0.3,
        "height": 0.1,
    },
    "rgba": color_lamps,
}
lamp1_2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp1[0], position_lamp1[1], 1],
        "radius": 0.1,
        "height": 2,
    },
    "rgba": color_lamps,
}
lamp1_3_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp1[0], position_lamp1[1], 2],
        "radius": 0.5,
        "height": 0.6,
    },
    "rgba": color_lamps,
}
#========================================================================
lamp1_1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp1_1_dict
)
lamp1_2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp1_2_dict
)
lamp1_3 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp1_3_dict
)
#========================================================================

# Standing lamp 2
#========================================================================
position_lamp2 = [-1, -3.5]
lamp2_1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp2[0], position_lamp2[1], 0.05],
        "radius": 0.3,
        "height": 0.1,
    },
    "rgba": color_lamps,
}
lamp2_2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp2[0], position_lamp2[1], 1],
        "radius": 0.1,
        "height": 2,
    },
    "rgba": color_lamps,
}
lamp2_3_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp2[0], position_lamp2[1], 2],
        "radius": 0.5,
        "height": 0.6,
    },
    "rgba": color_lamps,
}
#========================================================================
lamp2_1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp2_1_dict
)
lamp2_2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp2_2_dict
)
lamp2_3 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp2_3_dict
)
#========================================================================

# Hanging lamp 1
#========================================================================
position_lamp3 = [-5, -7.5, 2]
lamp3_1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp3[0], position_lamp3[1], 2],
        "radius": 0.6,
        "height": 0.2,
    },
    "rgba": color_lamps,
}
lamp3_2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp3[0], position_lamp3[1], 2.2],
        "radius": 0.3,
        "height": 0.2,
    },
    "rgba": color_lamps,
}
lamp3_3_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp3[0], position_lamp3[1], 2.8],
        "radius": 0.05,
        "height": 1,
    },
    "rgba": color_lamps,
}
#========================================================================
lamp3_1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp3_1_dict
)
lamp3_2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp3_2_dict
)
lamp3_3 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp3_3_dict
)
#========================================================================

# Hanging lamp 2
#========================================================================
position_lamp4 = [0, 5, 2]
lamp4_1_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp4[0], position_lamp4[1], 2],
        "radius": 0.6,
        "height": 0.2,
    },
    "rgba": color_lamps,
}
lamp4_2_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp4[0], position_lamp4[1], 2.2],
        "radius": 0.3,
        "height": 0.2,
    },
    "rgba": color_lamps,
}
lamp4_3_dict = {
    "type": "cylinder",
    "movable": False,
    "geometry": {
        "position": [position_lamp4[0], position_lamp4[1], 2.8],
        "radius": 0.05,
        "height": 1,
    },
    "rgba": color_lamps,
}
#========================================================================
lamp4_1 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp4_1_dict
)
lamp4_2 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp4_2_dict
)
lamp4_3 = CylinderObstacle(
    name="Cylinder1",
    content_dict=lamp4_3_dict
)
#========================================================================

"""=====================================================================================================
#================================== Adding obstacle spheres to list ====================================
#===================================================================================================="""

# Add hanging lamp 1 en 2
collision_obstacles += [hanglampsphere1_1, hanglampsphere2_1]

# Add table 1
collision_obstacles += [table1_1, table1_2]
#collision_obstacles += [table1_1, table1_2, table1_3]

# Add table 2
collision_obstacles += [table2_1, table2_3]
#collision_obstacles += [table2_1, table2_2, table2_3]

# Add table 3
collision_obstacles += [table3_1]

# Add table 4
collision_obstacles += [table4_1]

# Add chair 1
collision_obstacles += [chairsphere1_1, chairsphere1_3]
#collision_obstacles += [chairsphere1_1, chairsphere1_2, chairsphere1_3]

# Add chair 2
collision_obstacles += [chairsphere2_1, chairsphere2_3]
#collision_obstacles += [chairsphere2_1, chairsphere2_2, chairsphere2_3]

# Add closet 1, 2 and 3
collision_obstacles += [closetsphere2_1, closetsphere2_2, closetsphere2_3, closetsphere2_4][::-1]
collision_obstacles += [closetsphere3_1, closetsphere3_2][::-1]
#collision_obstacles += [closetsphere1_1, closetsphere1_2, closetsphere1_3, closetsphere1_4][::-1]

# Add standing lamp 1
collision_obstacles += [lampsphere1_1, lampsphere1_3]
#collision_obstacles += [lampsphere1_1, lampsphere1_2, lampsphere1_3]

# Add extra chair and lamp
if block_middle:
    # Add chair 3
    collision_obstacles += [chairsphere3_1, chairsphere3_3]
    #collision_obstacles += [chairsphere3_1, chairsphere3_2, chairsphere3_3]

    # Add standing lamp 2
    collision_obstacles += [lampsphere2_1, lampsphere2_3]
    #collision_obstacles += [lampsphere2_1, lampsphere2_2, lampsphere2_3]


"""=====================================================================================================
#================================== Adding obstacle models to list =====================================
#===================================================================================================="""

# Add walls (always displayed)
decorative_obstacles = [wall1, wall2, wall3, wall4]

# Only add other obstacles if toggle set to True
if display_objects:
    # Add table 1, 2, 3, 4
    decorative_obstacles += [tabletop1, tableleg1_1, tableleg1_2, tableleg1_3, tableleg1_4]
    decorative_obstacles += [tabletop2, tableleg2_1, tableleg2_2, tableleg2_3, tableleg2_4]
    decorative_obstacles += [roundtablefoot1, roundtableleg1, roundtabletop1]
    decorative_obstacles += [roundtablefoot2, roundtableleg2, roundtabletop2]

    # Add chair 1 and 2
    decorative_obstacles += [chairseat1, chairback1]
    decorative_obstacles += [chairseat2, chairback2]

    # Add closet 1, 2 and 3
    decorative_obstacles += [closet2, closet3]
    #decorative_obstacles += [closet1, closet2, closet3]

    #add standing lamp 1
    decorative_obstacles += [lamp1_1, lamp1_2, lamp1_3]

    #add hanging lamp 1 and 2
    decorative_obstacles += [lamp3_1, lamp3_2, lamp3_3]
    decorative_obstacles += [lamp4_1, lamp4_2, lamp4_3]
    
    # Add extra obstacles
    if block_middle:
        # Add lamp 2
        decorative_obstacles += [lamp2_1, lamp2_2, lamp2_3]

        # Add chair 3
        decorative_obstacles += [chairseat3, chairback3]









