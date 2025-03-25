#!/usr/bin/env python

"""Add utils launch function managing GZ stuff."""

from logging import (
    DEBUG,
)
from pathlib import (
    Path,
)
from typing import (
    Any,
    Dict,
    Optional,
    Text,
    Union,
)

from launch import (
    LaunchDescription,
)
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from tiago_sim.opaque_function import (
    apply,
    do_format,
    get_configs,
    get_envs,
    log,
    make_opaque_function_that,
    set_config,
)

from .logging import (
    logger,
)
from .utils import (
    dict_to_string,
)


def gz_server(
        *,
        world: Optional[Path] = None,
        gui: Optional[bool] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Create/update a description to launch a gz sim server.

    Parameters
    ----------
    world: Optional[Path]
      If not None, correspond to the sdf file use to spawn the server with.
      When None, declare a LaunchArgument for it (default to 'empty.sdf').
    gui: Optional[bool]
      If not None, indicates if we wish to spawn the UI or only the server in
      background. When None, declare a LaunchArgument for it (default to True).
    description: Optional[LaunchDescription]
      LaunchDescription to use instead of creating a new one

    Returns
    -------
    LaunchDescription
      The launch description with that launch the gz sim server accordingly
    """
    if world is None:
        description.add_action(
            DeclareLaunchArgument(
                'world',
                description='sdf file of the world we wish to create',
                default_value='empty.sdf'
            )
        )
        world = get_configs('world')

    if gui is None:
        description.add_action(
            DeclareLaunchArgument(
                'gui',
                description=(
                    'Set to false if you wish to disable the GUI and only '
                    'launch the server in background'
                ),
                default_value='True',
                choices=['False', 'True'],
            )
        )
        gui = get_configs(
            'gui',
            transform=lambda txt:
            True if txt == 'True' else False
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
                        'Creating GZ sim server using:'
                        '\n- With gui ?: {gui}'
                        '\n- World: {world}'
                        '\n- With env:'
                        '\n{env}'
                    ),
                    gui=gui,
                    world=world,
                    env=apply(
                        dict_to_string,
                        get_envs(
                            (
                                env_name
                                for _, env_name in all_env_arguments.values()
                            ),
                            as_dict=True
                        ),
                        kv_header='--> ',
                    )
                ),
                logger=logger,
            ),
            set_config(
                name='gz_sim_args',
                value=apply(
                    # This appends '-s' when gui is False
                    lambda world, have_gui:
                    world if have_gui else world + ' -s',
                    world, gui,
                )
            ),
        )
    )

    description.add_action(
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('gz_sim_args')],
            # shell=True is mandatory when adding whitespaces inside cmd
            shell=True,
        )
    )

    return description


def __get_sdf_type_from(value: Text) -> Text:
    value_path = Path(value)
    if value_path.exists() and (value_path.suffix in ('.sdf', '.urdf')):
        return 'sdf_filename'
    else:
        return 'sdf'


def gz_spawn_entity(
        *,
        model: Optional[Union[Path, Text]] = None,
        name: Optional[Text] = None,
        world: Optional[Text] = None,
        timeout: Optional[int] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Spawn a model, with a given name, into an already running GZ server.

    Parameters
    ----------
    model: Optional[Union[Path, Text]]
      If not None, the model we wish to spawn. It may be either a file
      (.sdf/.urdf) or directly a string.
      When None, declare a LaunchArgument for it.
    name: Optional[Text]
      If not None, the name of the entity spawned inside gz.
      When None, declare a LaunchArgument for it (default to 'tiago').
    world: Optional[Path]
      If not None, the GZ world we wish to spawn our model into.
      When None, declare a LaunchArgument for it (default to 'empty').
    timeout: Optional[int]
      If not None, the timeout in ms associated to the gz service request.
      When None, declare a LaunchArgument for it (default to 1000).
    description: Optional[LaunchDescription]
      LaunchDescription to use instead of creating a new one

    Returns
    -------
    LaunchDescription
      The launch description with that spanw the model into a gz server
    """
    if model is None:
        description.add_action(
            DeclareLaunchArgument(
                'model',
                description=(
                    'The model to spawn. '
                    'Expecting either an sdf or urdf file path (checking '
                    'files extensions) OR a raw SDF string model.'
                ),
                # default_value=LaunchConfiguration('model_path'),
            )
        )
        model = get_configs('model')

    if name is None:
        description.add_action(
            DeclareLaunchArgument(
                'name',
                description='Name of the entity to spawn inside gz',
                default_value='tiago',
            )
        )
        name = get_configs('name')

    if world is None:
        description.add_action(
            DeclareLaunchArgument(
                'world',
                description=(
                    'Name of the world we wish to spawn the entity into'
                ),
                default_value='empty',
            )
        )
        world = get_configs('world')

    if timeout is None:
        description.add_action(
            DeclareLaunchArgument(
                'timeout',
                description='Timeout associated to the gz request (in ms)',
                default_value='1000',
            )
        )
        timeout = get_configs('timeout', transform=int)

    description.add_action(
        make_opaque_function_that(
            set_config(
                'service_args',
                do_format(
                    (
                        '-s /world/{world}/create'
                        ' --reqtype gz.msgs.EntityFactory'
                        ' --reptype gz.msgs.Boolean'
                        ' --timeout {timeout}'
                        ' --req \'name: "{name}", {model_type}: "{model}"\''
                    ),
                    world=world,
                    timeout=timeout,
                    name=name,
                    model_type=apply(
                        __get_sdf_type_from,
                        model,
                    ),
                    model=model,
                )
            ),
            log(
                msg=do_format(
                    'Try to spawn entity "{name}" into "{world}"',
                    name=name,
                    world=world,
                ),
                logger=logger,
            ),
            log(
                msg=do_format(
                    'Using command:'
                    '\n- gz service {args}',
                    args=get_configs('service_args'),
                ),
                logger=logger,
                level=DEBUG,
            )
        )
    )

    description.add_action(
        ExecuteProcess(
            cmd=[
                'gz', 'service', LaunchConfiguration('service_args'),
            ],
            shell=True,
        )
    )

    return description
