#!/usr/bin/env python

"""ROS2 Launch file use to populate tiago's robot_description configuration."""
from tiago_lfc.launch import (
    make_robot_description_from_tiago_description
)


def generate_launch_description():
    """Add the robot_description launch configuration using tiago's xacro."""
    return make_robot_description_from_tiago_description()
