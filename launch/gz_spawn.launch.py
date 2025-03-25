#!/usr/bin/env python

"""ROS2 launch file that spawn a GZ entity into an already running GZ world."""

from tiago_sim.launch import (
    make_gz_spawn,
)


def generate_launch_description():
    """TODO."""
    return make_gz_spawn()
