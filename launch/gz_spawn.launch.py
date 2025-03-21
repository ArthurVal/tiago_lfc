#!/usr/bin/env python

"""TODO."""

from ament_index_python.packages \
    import get_package_share_directory as shared_dir

from launch import (
    LaunchContext,
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    # EnvironmentVariable,
    LaunchConfiguration,
)

def generate_launch_description():
    """Spawn tiago inside an already running GZ's world."""
    
    
    return LaunchDescription(
        [
            # TODO
        ]
    )
