#!/usr/bin/env python

"""Launch file to create an instance of robot_state_publisher for tiago."""

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration

from tiago_sim.launch import (
    add_robot_description_from_xacro,
    declare_arguments_from_yaml,
    run_robot_state_publisher,
)


def generate_launch_description():
    """Spawn a robot_state_publisher, using robot_description provided."""
    # NOTE: We don't use Include... from launch stuff because otherwise
    # arguments coming from included launch file don't appear when doing
    # `ros2 launch <pkg> <launch> -s`
    description, args_names = declare_arguments_from_yaml(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'config',
            'tiago_configuration.yaml',
        ),
    )

    description = add_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings_config_names=args_names,
        description=description,
    )

    return run_robot_state_publisher(
        robot_description=LaunchConfiguration('robot_description'),
        description=description
    )
