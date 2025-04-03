#!/usr/bin/env python

"""Add utils launch function managing GZ stuff."""
from collections.abc import (
    Generator,
    Mapping,
)
from pathlib import (
    Path,
)
from typing import (
    Optional,
    Text,
)

from launch import (
    Action,
)
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)

from launch_ros.actions import (
    Node,
)

from .invoke import (
    Invoke,
    SubstitutionOr,
    evaluate_dict,
)
from .logging import (
    logger,
)
from .utils import (
    dict_to_string,
)


def __log_then_forward_cmd(cmd):
    logger.debug(
        'Command: `{}`'.format(
            ' '.join(cmd)
        )
    )
    return cmd


def __make_sim_cmd(
        gui: bool,
        world: Text,
        envs: Mapping[Text, Text],
):
    logger.info(
        (
            'Creating GZ sim server using:'
            '\n- Gui: {gui}'
            '\n- World: {world}'
            '\n- Envs:'
            '\n{envs}'
        ).format(
            gui=gui,
            world=world,
            envs=dict_to_string(
                envs,
                fmt='--> {k}: {v}',
            ),
        )
    )

    cmd = [
        'gz',
        'sim',
        world,
    ]

    if not gui:
        cmd.append('-s')

    return cmd


def gz_server(
        *,
        world: Optional[SubstitutionOr[Path]] = None,
        gui: Optional[SubstitutionOr[bool]] = None,
) -> Generator[Action]:
    """Create/update a description to launch a gz sim server.

    Parameters
    ----------
    world: Optional[SubstitutionOr[Path]]
      If not None, correspond to the sdf file use to spawn the server with.
      When None, declare a LaunchArgument for it (default to 'empty.sdf').
    gui: Optional[SubstitutionOr[bool]]
      If not None, indicates if we wish to spawn the UI or only the server in
      background. When None, declare a LaunchArgument for it (default to True).
    description: Optional[LaunchDescription]
      LaunchDescription to use instead of creating a new one

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if world is None:
        yield DeclareLaunchArgument(
            'world',
            description='sdf file of the world we wish to create',
            default_value='empty.sdf'
        )
        world = LaunchConfiguration('world')

    if gui is None:
        yield DeclareLaunchArgument(
            'gui',
            description=(
                'Set to false if you wish to disable the GUI and only '
                'launch the server in background'
            ),
            default_value='True',
            choices=['False', 'True'],
        )
        gui = Invoke(
            lambda txt: True if txt == 'True' else False,
            LaunchConfiguration('gui'),
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
        yield DeclareLaunchArgument(
            arg_name,
            description=(
                'Contains paths to {descr}. Will be appended to {env_var}.'
            ).format(
                descr=descr,
                env_var=env_var,
            ),
            default_value='',
        )

        yield AppendEnvironmentVariable(
            env_var,
            LaunchConfiguration(arg_name),
        )

    yield Invoke(
        __make_sim_cmd,
        gui=gui,
        world=world,
        envs=evaluate_dict(
            {
                name: EnvironmentVariable(name)
                for _, name in all_env_arguments.values()
            }
        ),
    ).and_then(
        __log_then_forward_cmd,
    ).and_then_with_key(
        'cmd',
        ExecuteProcess
    )

    # # FIXME: Is this needed ?
    yield Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


def __make_spawn_cmd(
        world: Text,
        name: Text,
        model_path: Path,
        timeout_ms: int,
):
    logger.info(
        (
            "Spawning '{name}' from '{path}'"
            '\n- Into world: {world}'
            '\n- Timeout: {timeout}ms'
        ).format(
            name=name,
            world=world,
            path=model_path,
            timeout=timeout_ms,
        )
    )

    return [
        'gz',
        'service',
        '-s', '/world/{}/create'.format(world),
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '{}'.format(timeout_ms),
        '--req',
        'name: "{name}", sdf_filename: "{path}"'.format(
            name=name,
            path=model_path
        )
    ]


def gz_spawn_entity(
        *,
        model_path: Optional[SubstitutionOr[Path]] = None,
        name: Optional[SubstitutionOr[Text]] = None,
        world: Optional[SubstitutionOr[Text]] = None,
        timeout_ms: Optional[SubstitutionOr[int]] = None,
) -> Generator[Action]:
    """Spawn a model, with a given name, into an already running GZ server.

    Parameters
    ----------
    model_path: Optional[SubstitutionOr[Path]]
      If not None, the model we wish to spawn. It may be either a
      .sdf or .urdf file.
      When None, declare a LaunchArgument for it.
    name: Optional[SubstitutionOr[Text]]
      If not None, the name of the entity spawned inside gz.
      When None, declare a LaunchArgument for it (default to 'tiago').
    world: Optional[SubstitutionOr[Path]]
      If not None, the GZ world we wish to spawn our model into.
      When None, declare a LaunchArgument for it (default to 'empty').
    timeout: Optional[SubstitutionOr[int]]
      If not None, the timeout in ms associated to the gz service request.
      When None, declare a LaunchArgument for it (default to 1000).

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if model_path is None:
        yield DeclareLaunchArgument(
            'model_path',
            description=(
                'The model file to spawn. '
                'Expecting either an sdf or urdf file path (checking '
                'files extensions).'
            ),
        )
        model_path = Invoke(
            Path,
            LaunchConfiguration('model_path')
        )

    if name is None:
        yield DeclareLaunchArgument(
            'name',
            description='Name of the entity to spawn inside gz',
            default_value='tiago',
        )
        name = LaunchConfiguration('name')

    if world is None:
        yield DeclareLaunchArgument(
            'world',
            description=(
                'Name of the world we wish to spawn the entity into'
            ),
            default_value='empty',
        )
        world = LaunchConfiguration('world')

    if timeout_ms is None:
        yield DeclareLaunchArgument(
            'timeout_ms',
            description='Timeout associated to the gz request (in ms)',
            default_value='1000',
        )
        timeout_ms = Invoke(
            int,
            LaunchConfiguration('timeout_ms'),
        )

    yield Invoke(
        __make_spawn_cmd,
        world,
        name,
        model_path,
        timeout_ms,
    ).and_then(
        __log_then_forward_cmd
    ).and_then_with_key(
        'cmd',
        ExecuteProcess,
    )
