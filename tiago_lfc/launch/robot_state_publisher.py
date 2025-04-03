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

from .invoke import (
    Invoke,
    SubstitutionOr,
)
from .logging import (
    logger,
)


def run_robot_state_publisher(
        *,
        robot_description: Optional[SubstitutionOr[Text]] = None,
        namespace: Optional[SubstitutionOr[Text]] = None,
        use_sim_time: Optional[Union[bool, SubstitutionOr[Text]]] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Spawn a robot_state_publisher Node.

    Parameters
    ----------
    robot_description: Optional[SubstitutionOr[Text]]
      The robot description used. If not provided, declare a mandatory launch
      argument for it
    namespace: Optional[SubstitutionOr[Text]]
      Namespace of the node. If not provided, declare a launch argument for it
      (default to '')
    use_sim_time: Optional[bool | SubstitutionOr[Text]]
      Indicates if the clock comes from simulation or the normal OS wall clock
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
            Invoke(
                SetLaunchConfiguration,
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
            Invoke(
                SetLaunchConfiguration,
                'namespace',
                namespace
            )
        )

    if use_sim_time is None:
        description.add_action(
            DeclareLaunchArgument(
                'use_sim_time',
                choices=['True', 'False'],
                default_value='False',
            )
        )
    else:
        description.add_action(
            Invoke(
                SetLaunchConfiguration,
                'use_sim_time',
                str(use_sim_time)
                if isinstance(use_sim_time, bool)
                else use_sim_time
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
                        'use_sim_time'
                    ),
                }
            ],
        )
    )

    description.add_action(
        Invoke(
            (
                'robot_state_publisher spawned with:'
                '\n - namespace: "{ns}"'
                '\n - use_sim_time: {sim_time}'
            ).format,
            ns=LaunchConfiguration('namespace'),
            sim_time=LaunchConfiguration('use_sim_time'),
        ).and_then(
            logger.info,
        )
    )
    return description
