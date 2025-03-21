#!/usr/bin/env python

"""TODO."""

from pathlib import Path
from pprint import (
    pformat,
)

from ament_index_python.packages \
    import get_package_share_directory as shared_dir

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
)

from launch_param_builder import (
    load_yaml,
)

from tiago_lfc.opaque_function import (
    do_format,
    from_xacro,
    get_configs,
    log,
    make_opaque_function_that,
    set_config,
    apply,
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

    tiago_xacro_mappings_values = get_configs(
        [arg.name for arg in tiago_xacro_mappings_args],
        as_dict=True,
    )

    log_args = make_opaque_function_that(
        log(
            msg=do_format(
                (
                    '\nUsing xacro model from:'
                    '\n{xacro}'
                    '\nMappings:'
                    '\n- Exported from:'
                    '\n{xacro_config}'
                    '\n- Values (can be updated with <name>:=<value>):'
                    '\n{xacro_mappings}'
                ),
                xacro=get_configs(set_tiago_xacro.name),
                xacro_config=get_configs(set_tiago_xacro_mappings.name),
                xacro_mappings=apply(pformat, tiago_xacro_mappings_values)
            )
        ),
    )

    set_robot_description = make_opaque_function_that(
        set_config(
            name='robot_description',
            value=from_xacro(
                file_path=get_configs(set_tiago_xacro.name, transform=Path),
                mappings=tiago_xacro_mappings_values
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
            log_args,
            set_robot_description,
        ]
    )
