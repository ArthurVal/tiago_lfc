#!/usr/bin/env python

"""TODO."""

from typing import (
    Text,
)

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node

from tiago_lfc.opaque_function import (
    do_format,
    get_configs,
    log,
    make_opaque_function_that,
)

from . import (
    logger,
)


def make_robot_state_publisher(
        *,
        description: LaunchDescription = LaunchDescription(),
        robot_description_config_name: Text = 'robot_description',
) -> LaunchDescription:
    """Spawn a robot_state_publisher, using robot_description config."""
    description.add_action(
        DeclareLaunchArgument(
            robot_description_config_name,
            description='Robot description used by the robot_state_publisher',
            default_value='',
        )
    )

    description.add_action(
        DeclareLaunchArgument(
            'namespace',
            description='Namespace used to populate nodes',
            default_value='',
        )
    )

    description.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
        )
    )

    description.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {
                    'robot_description': LaunchConfiguration(
                        robot_description_config_name
                    ),
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time'
                    ),
                }
            ],
        )
    )

    description.add_action(
        make_opaque_function_that(
            log(
                msg=do_format(
                    (
                        'robot_state_publisher spawned with:'
                        '\n - robot_description_config_name: "{name}"'
                        '\n - namespace: "{ns}"'
                        '\n - use_sim_time: {sim_time}'
                    ),
                    name=robot_description_config_name,
                    ns=get_configs('namespace'),
                    sim_time=get_configs('use_sim_time'),
                ),
                logger=logger,
            )
        )
    )
    return description
