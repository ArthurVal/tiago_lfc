#!/usr/bin/env python

"""Populate robot_description configuration from any xacro."""
from launch import LaunchDescription

from tiago_lfc.launch import (
    add_robot_description_from_xacro,
)


def generate_launch_description():
    """Add the robot_description launch configuration using tiago's xacro."""
    return LaunchDescription(add_robot_description_from_xacro())
