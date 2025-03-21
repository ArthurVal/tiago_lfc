#!/usr/bin/env python

"""TODO."""

from tiago_lfc.launch import (
    make_gz_server,
    make_gz_spawn,
    make_robot_description_from_tiago_description,
    make_robot_state_publisher,
)


def generate_launch_description():
    """Spawn a robot_state_publisher, using robot_description provided."""
    d = make_gz_server()

    d = make_robot_description_from_tiago_description(
        description=d
    )

    d = make_robot_state_publisher(
        description=d
    )

    return make_gz_spawn(description=d)
