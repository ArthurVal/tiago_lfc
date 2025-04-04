#!/usr/bin/env python

"""Launch file use to load ANY ros2 control controller."""

from launch import LaunchDescription

from tiago_lfc.launch import (
    load_controllers,
)


def generate_launch_description():
    """Load controllers."""
    return LaunchDescription(load_controllers())
