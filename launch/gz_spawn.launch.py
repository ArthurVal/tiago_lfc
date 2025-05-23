#!/usr/bin/env python

"""ROS2 launch file that spawn a GZ entity into an already running GZ world."""
from launch import LaunchDescription

from tiago_lfc.launch import (
    gz_spawn_entity,
)


def generate_launch_description():
    """Spawn a GZ entity model."""
    return LaunchDescription(gz_spawn_entity())
