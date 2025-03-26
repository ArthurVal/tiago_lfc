#!/usr/bin/env python

"""ROS2 Launch file use to populate tiago's robot_description configuration."""

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    DeclareLaunchArgument,
)

from tiago_sim.launch import (
    add_robot_description_from_xacro,
    declare_arguments_from_yaml,
)
from tiago_sim.opaque_function import (
    get_configs,
)


def generate_launch_description():
    """Add the robot_description launch configuration using tiago's xacro."""
    description, args_names = declare_arguments_from_yaml(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'config',
            'tiago_configuration.yaml',
        ),
    )

    # use_sim_time is used in tiago_description but not inside
    # tiago_configuration.yaml. We add it by hand
    description.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            choices=['True', 'False'],
            default_value='False'
        )
    )
    args_names.append('use_sim_time')

    description = add_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings=get_configs(args_names, as_dict=True),
        description=description,
    )

    return description
