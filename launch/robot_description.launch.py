#!/usr/bin/env python

"""TODO.

Description.
"""
from pathlib import Path

from ament_index_python.packages \
    import get_package_share_directory as shared_dir

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    # EnvironmentVariable,
    LaunchConfiguration,
)

from launch_param_builder import (
    load_yaml,
)

from tiago_lfc.opaque_function import (
    from_xacro,
    get_configs,
    make_opaque_function_that,
    set_config,
)


def generate_launch_description():
    """Spawn tiago inside an already running GZ's world."""
    # FIXME: maybe use args to select xacro location and its args ?
    # This for a depedency on tiago_description package otherwise...
    tiago_xacro = Path(shared_dir('tiago_description'))
    tiago_xacro /= 'robots'
    tiago_xacro /= 'tiago.urdf.xacro'

    # There is a file in tiago_description that define all xacro mappings using
    # the launch DeclareArgument format in yaml...
    tiago_xacro_mappings = Path(shared_dir('tiago_description'))
    tiago_xacro_mappings /= 'config'
    tiago_xacro_mappings /= 'tiago_configuration.yaml'

    tiago_xacro_mappings_args = [
        DeclareLaunchArgument(name, **params)
        for name, params in load_yaml(tiago_xacro_mappings).items()
    ]

    force_use_sim_time = SetLaunchConfiguration('use_sim_time', 'True')

    set_tiago_xacro = SetLaunchConfiguration(
        'tiago_xacro',
        str(tiago_xacro),
    )

    set_tiago_xacro_mappings = SetLaunchConfiguration(
        'tiago_xacro_mappings',
        str(tiago_xacro_mappings),
    )

    log_tiago_xacro = LogInfo(
        msg=[
            'Tiago model used:',
            '\n - Model   : ', LaunchConfiguration('tiago_xacro'),
            '\n - Mappings: ', LaunchConfiguration('tiago_xacro_mappings'),
        ]
    )

    set_robot_description = make_opaque_function_that(
        set_config(
            name='robot_description',
            value=from_xacro(
                file_path=get_configs('tiago_xacro', transform=Path),
                mappings=get_configs(
                    *[arg.name for arg in tiago_xacro_mappings_args],
                    as_dict=True,
                )
            ),
        ),
    )

    return LaunchDescription(
        tiago_xacro_mappings_args +
        [
            # Config updates
            force_use_sim_time,
            set_tiago_xacro,
            set_tiago_xacro_mappings,
            log_tiago_xacro,
            set_robot_description,
        ]
    )
