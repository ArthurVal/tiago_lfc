#!/usr/bin/env python

"""ROS2 launch file that spawn a GZ entity into an already running GZ world."""

from tiago_lfc.launch import (
    gz_spawn_entity,
)


def generate_launch_description():
    """Spawn a GZ entity model."""
    return gz_spawn_entity()
