#!/usr/bin/env python

"""TODO."""

import logging

from pathlib import (
    Path,
)
from pprint import (
    pformat,
)
from typing import (
    Optional,
)

from launch import (
    LaunchDescription,
)
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from tiago_lfc.opaque_function import (
    apply,
    do_format,
    get_configs,
    get_envs,
    log,
    make_opaque_function_that,
    set_config,
)

from . import (
    logger,
)


def make_gz_server(
        *,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Create/update a description to launch a gz server in the background."""
    description.add_action(
        SetLaunchConfiguration(
            'use_sim_time',
            'True',
        )
    )

    description.add_action(
        DeclareLaunchArgument(
            'world_file',
            description='sdf file of the world we wish to create',
            default_value='empty.sdf'
        )
    )

    all_env_arguments = {
        'resource_path': (
            'world/models sdf files',
            'GZ_SIM_RESOURCE_PATH'
        ),
        'system_plugin_path': (
            'system plugins',
            'GZ_SIM_SYSTEM_PLUGIN_PATH'
        ),
        'server_config_path': (
            'server configurations',
            'GZ_SIM_SERVER_CONFIG_PATH'
        ),
        'gui_plugin_path': (
            'GUI plugins',
            'GZ_SIM_GUI_PLUGIN_PATH'
        ),
        'gui_resource_path': (
            'GUI resource files (config files)',
            'GZ_SIM_GUI_RESOURCE_PATH'
        ),
    }

    for arg_name, details in all_env_arguments.items():
        descr, env_var = details

        description.add_action(
            DeclareLaunchArgument(
                arg_name,
                description=(
                    'Contains paths to {descr}. Will be appended to {env_var}.'
                ).format(
                    descr=descr,
                    env_var=env_var,
                ),
                default_value='',
            )
        )

        description.add_action(
            AppendEnvironmentVariable(
                env_var,
                LaunchConfiguration(arg_name),
            )
        )

    description.add_action(
        make_opaque_function_that(
            log(
                msg=do_format(
                    (
                        'Creating GZ server:'
                        '\n - World file used: {world_file}'
                        '\n - Path lookup env variable:'
                        '\n{env}'
                    ),
                    world_file=get_configs('world_file'),
                    env=apply(
                        pformat,
                        get_envs(
                            (
                                env_name
                                for _, env_name in all_env_arguments.values()
                            ),
                            as_dict=True
                        )
                    )
                ),
                logger=logger,
            )
        )
    )

    description.add_action(
        ExecuteProcess(
            cmd=[
                'gz', 'sim', LaunchConfiguration('world_file'), '-s'
            ],
        )
    )

    return description


def make_gz_spawn(
        *,
        description: LaunchDescription = LaunchDescription(),
        model_path: Optional[Path] = None,
) -> LaunchDescription:
    """TODO."""
    if model_path is None:
        description.add_action(
            DeclareLaunchArgument(
                'model_path',
                description='Path of the entity model file we wish to spawn',
                # default_value=LaunchConfiguration('model_path'),
            )
        )
    else:
        description.add_action(
            SetLaunchConfiguration('model_path', str(model_path))
        )

    description.add_action(
        DeclareLaunchArgument(
            'entity_name',
            description='Name of the entity spawn inside gz',
            default_value='tiago',
        )
    )

    description.add_action(
        DeclareLaunchArgument(
            'world',
            description='Name of the world we wish to spawn the entity into',
            default_value='empty',
        )
    )

    description.add_action(
        DeclareLaunchArgument(
            'service_timeout',
            description='Timeout associated to the gz request (in ms)',
            default_value='1000',
        )
    )

    description.add_action(
        make_opaque_function_that(
            set_config(
                name='service_name',
                value=do_format(
                    '/world/{name}/create',
                    name=get_configs('world'),
                )
            ),
            set_config(
                name='service_request',
                value=do_format(
                    'sdf_filename: "{file_name}", name: "{entity_name}"',
                    file_name=get_configs('model_path'),
                    entity_name=get_configs('entity_name'),
                )
            ),
            log(
                msg=do_format(
                    'Try to spawn entity "{entity_name}" into "{world_name}"',
                    entity_name=get_configs('entity_name'),
                    world_name=get_configs('world'),
                ),
                logger=logger,
            ),
            log(
                msg=do_format(
                    'Using GZ args:'
                    '\n - service name: {name}'
                    '\n - request: {req}',
                    name=get_configs('service_name'),
                    req=get_configs('service_request')
                ),
                logger=logger,
                level=logging.DEBUG,
            )
        )
    )

    description.add_action(
        ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', LaunchConfiguration('service_name'),
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', LaunchConfiguration('service_timeout'),
                '--req', LaunchConfiguration('service_request'),
            ],
        )
    )

    return description
