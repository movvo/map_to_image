"""
   Copyright 2022 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
   Contact: support.idi@ageve.net
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get config file
    package_name = 'map_to_image'
    script_path = os.path.dirname(os.path.realpath(__file__))
    root_path_full = os.path.dirname(script_path)
    config_file = os.path.join(root_path_full, "config", "params.yaml")

    # Node to launch
    node = Node(
            package=package_name,
            namespace='movvo',
            name='map_to_image',
            executable='map_to_image',
            arguments=['--ros-args','--log-level','info'],
            parameters=[config_file],
            output="screen",
            emulate_tty=True
            )   
            
    ld = LaunchDescription()
    ld.add_action(node)

    return ld