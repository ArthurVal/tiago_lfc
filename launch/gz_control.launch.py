#!/usr/bin/env python

"""ROS2 launch file that interact with a given GZ world control."""
from launch import LaunchDescription

from tiago_lfc.launch import (
    gz_control,
)


def generate_launch_description():
    """Send a GZ WorldControl request (Pause/Play/Step/...)."""
    return LaunchDescription(gz_control())
