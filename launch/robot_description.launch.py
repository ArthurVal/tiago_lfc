#!/usr/bin/env python

"""ROS2 Launch file use to populate tiago's robot_description configuration."""

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from tiago_lfc.launch import (
    make_arguments_from_yaml,
    make_robot_description_from_xacro,
)


def generate_launch_description():
    """Add the robot_description launch configuration using tiago's xacro."""
    description, args_names = make_arguments_from_yaml(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'config',
            'tiago_configuration.yaml',
        ),
    )

    description = make_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings_config_names=args_names,
        description=description,
    )

    return description
