#!/usr/bin/env python

"""Create an instance of robot_state_publisher from tiago's xacro."""
from itertools import (
    chain,
)
from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from tiago_lfc.launch import (
    add_robot_description_from_xacro,
    all_arguments_from_yaml,
    evaluate_dict,
    run_robot_state_publisher,
)


def generate_launch_description():
    """Spawn a robot_state_publisher, using robot_description provided."""
    xacro_args = [
        # use_sim_time is used in tiago_description but not inside
        # tiago_configuration.yaml. We add it by hand.
        DeclareLaunchArgument(
            'use_sim_time',
            choices=['True', 'False'],
            default_value='False'
        )
    ] + list(
        all_arguments_from_yaml(
            file_path=Path(
                get_package_share_directory('tiago_description'),
                'config',
                'tiago_configuration.yaml',
            ),
        )
    )

    robot_description = add_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings=evaluate_dict(
            {
                arg.name: LaunchConfiguration(arg.name)
                for arg in xacro_args
            }
        ),
    )

    robot_state_publisher = run_robot_state_publisher(
        robot_description=LaunchConfiguration('robot_description'),
    )

    return LaunchDescription(
        chain(xacro_args, robot_description, robot_state_publisher)
    )
