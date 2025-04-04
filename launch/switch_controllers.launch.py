#!/usr/bin/env python

"""Launch file use to switch ANY ros2 control controller."""

from launch import LaunchDescription

from tiago_lfc.launch import (
    switch_controllers,
)


def generate_launch_description():
    """Load controllers."""
    return LaunchDescription(switch_controllers())
