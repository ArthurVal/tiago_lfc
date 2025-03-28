#!/usr/bin/env python

"""Create an instance of robot_state_publisher from any xacro."""

from launch.substitutions import LaunchConfiguration

from tiago_sim.launch import (
    add_robot_description_from_xacro,
    run_robot_state_publisher,
)


def generate_launch_description():
    """Spawn a robot_state_publisher, using a xacro file."""
    description = add_robot_description_from_xacro()

    run_robot_state_publisher(
        robot_description=LaunchConfiguration('robot_description'),
        description=description
    )

    return description
