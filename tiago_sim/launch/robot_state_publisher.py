#!/usr/bin/env python

"""Module providing robot_state_publisher utils launch functions."""

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
    """Spawn a robot_state_publisher Node.

    Parameters
    ----------
    robot_description: Optional[Union[Text, LaunchConfiguration]]
      The robot description used. If not provided, declare a mandatory launch
      argument for it
    namespace: Optional[Union[Text, LaunchConfiguration]]
      Namespace of the node. If not provided, declare a launch argument for it
      (default to '')
    description: LaunchDescription
      Optional LaunchDescription to use. Create a new one by default.

    Returns
    -------
    LaunchDescription
      The launch description used/created with the node in it
    """
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
