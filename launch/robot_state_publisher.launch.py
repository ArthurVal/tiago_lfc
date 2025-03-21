#!/usr/bin/env python

"""TODO."""

from tiago_lfc.launch import (
    make_robot_description_from_tiago_description,
    make_robot_state_publisher,
)


def generate_launch_description():
    """Spawn a robot_state_publisher, using robot_description provided."""
    return make_robot_state_publisher(
        description=make_robot_description_from_tiago_description()
    )
