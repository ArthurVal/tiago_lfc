#!/usr/bin/env python

"""ROS2 launch file launching a GZ server."""

from tiago_lfc.launch import (
    make_gz_server,
)


def generate_launch_description():
    """Create a GZ server."""
    return make_gz_server()
