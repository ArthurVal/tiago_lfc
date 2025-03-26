#!/usr/bin/env python

"""TODO."""

from typing import (
    Optional,
    Text,
    Union,
)

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node

from tiago_sim.opaque_function import (
    do_format,
    get_configs,
    log,
    make_opaque_function_that,
)

from .logging import (
    logger,
)


def run_robot_state_publisher(
        *,
        robot_description: Optional[Union[Text, LaunchConfiguration]] = None,
        namespace: Optional[Union[Text, LaunchConfiguration]] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Spawn a robot_state_publisher, using robot_description config."""
    if robot_description is None:
        description.add_action(
            DeclareLaunchArgument(
                'robot_description',
                description=(
                    'Robot description used by the robot_state_publisher'
                ),
            )
        )
    else:
        description.add_action(
            SetLaunchConfiguration(
                'robot_description',
                robot_description
            )
        )

    if namespace is None:
        description.add_action(
            DeclareLaunchArgument(
                'namespace',
                description=(
                    'Namespace used by the node'
                ),
                default_value='',
            )
        )
    else:
        description.add_action(
            SetLaunchConfiguration(
                'namespace', namespace
            )
        )

    description.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                {
                    'robot_description': LaunchConfiguration(
                        'robot_description'
                    ),
                    'use_sim_time': LaunchConfiguration(
                        'use_sim_time', default='False'
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
                        '\n - namespace: "{ns}"'
                        '\n - use_sim_time: {sim_time}'
                    ),
                    ns=get_configs('namespace'),
                    sim_time=get_configs('use_sim_time'),
                ),
                logger=logger,
            )
        )
    )
    return description
