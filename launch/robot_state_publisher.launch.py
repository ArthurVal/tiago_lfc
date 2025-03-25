#!/usr/bin/env python

"""Launch file to create an instance of robot_state_publisher for tiago."""

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from tiago_sim.launch import (
    declare_arguments_from_yaml,
    make_robot_description_from_xacro,
    make_robot_state_publisher,
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

    description = make_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings_config_names=args_names,
        description=description,
    )

    return make_robot_state_publisher(
        description=description
    )
