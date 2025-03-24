#!/usr/bin/env python

"""ROS2 launch file that spawn a GZ entity into an already running GZ world."""

from tiago_lfc.launch import (
    make_gz_spawn,
)


def generate_launch_description():
    return make_gz_spawn()
