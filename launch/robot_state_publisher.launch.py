#!/usr/bin/env python

"""TODO."""

import os

from ament_index_python.packages \
    import get_package_share_directory as shared_dir

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import Node

# from tiago_lfc.opaque_function import (
#     get_configs,
#     make_opaque_function_that,
# )


def generate_launch_description():
    """Spawn a robot_state_publisher."""
    robot_description_launch_file_arg = DeclareLaunchArgument(
        'robot_description_launch_file',
        description='Launch file used to populate robot_description',
        default_value=os.path.join(
            shared_dir('tiago_lfc'),
            'launch',
            'robot_description.launch.py'
        ),
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace of the robot_state_publisher',
        default_value='',
    )

    return LaunchDescription(
        [
            robot_description_launch_file_arg,
            namespace_arg,

            LogInfo(
                msg=[
                    'Using robot_description launch file located @:\n',
                    LaunchConfiguration(robot_description_launch_file_arg.name),
                ]
            ),

            # Set the robot_description
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    LaunchConfiguration(robot_description_launch_file_arg.name)
                )
            ),

            # Spawn the node
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='both',
                namespace=LaunchConfiguration(namespace_arg.name),
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
        ]
    )
