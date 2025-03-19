#!/usr/bin/env python

"""TODO.

Description.
"""
from collections.abc import (
    Callable,
    Mapping,
)
from pathlib import Path
from typing import (
    Any,
    List,
    Optional,
    TypeAlias,
    TypeVar,
    Union,
)

from ament_index_python.packages \
    import get_package_share_directory as shared_dir

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_context import LaunchContext
from launch.substitutions import (
    # EnvironmentVariable,
    LaunchConfiguration,
)

from launch_param_builder import (
    load_xacro,
    load_yaml,
)

from launch_ros.actions import Node


def generate_launch_description():
    """Spawn tiago inside an already running GZ's world."""
    # FIXME: maybe use args to select xacro location and its args ?
    # This for a depedency on tiago_description package otherwise...
    tiago_xacro = Path(shared_dir('tiago_description'))
    tiago_xacro /= 'robots'
    tiago_xacro /= 'tiago.urdf.xacro'

    tiago_xacro_mappings = Path(shared_dir('tiago_description'))
    tiago_xacro_mappings /= 'config'
    tiago_xacro_mappings /= 'tiago_configuration.yaml'

    tiago_xacro_mappings_args = [
        DeclareLaunchArgument(name, **config)
        for name, config in load_yaml(tiago_xacro_mappings).items()
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

    set_robot_description = __make_opaque_function_that(
        __set_config(
            name='robot_description',
            value=__load_xacro(
                file_path=__from_config('tiago_xacro', convert_to=Path),
                mappings=__dict_from_configs(
                    names=[arg.name for arg in tiago_xacro_mappings_args]
                )
            ),
        ),

    )

    # Add the robot_state_publisher for ros2_control
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {
                'robot_description': LaunchConfiguration('robot_description'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # Extra args for GZ
    # gz_arguments = [
    #     DeclareLaunchArgument(
    #         'world_name',
    #         description='World name use to spawn Tiago into',
    #         default_value='',
    #     ),
    #     DeclareLaunchArgument(
    #         'entity_name',
    #         description='Name used as entity_name in the GZ world',
    #         default_value='tiago',
    #     ),
    #     DeclareLaunchArgument(
    #         '',
    #         description='todo',
    #         default_value='',
    #     ),
    # ]

    return LaunchDescription(
        tiago_xacro_mappings_args +
        [
            # Config updates
            force_use_sim_time,
            set_tiago_xacro,
            set_tiago_xacro_mappings,
            log_tiago_xacro,
            set_robot_description,

            # Node spawn
            start_robot_state_publisher,
        ]
    )


T = TypeVar('T')
LaunchWrapper: TypeAlias = Callable[[LaunchContext], T]
LaunchWrapperOr: TypeAlias = Union[LaunchWrapper[T], T]


def __eval[T](
        obj: LaunchWrapperOr[T],
        context: LaunchContext
) -> T:
    return obj(context) if callable(obj) else obj


def __from_config[T](
        name: str,
        convert_to: Optional[Callable[[str], T]] = None
) -> LaunchWrapper[Union[str, T]]:
    def __wrapper(context: LaunchContext) -> Union[str, T]:
        value = LaunchConfiguration(name).perform(context)
        return value if convert_to is None else convert_to(value)

    return __wrapper


def __set_config(
        name: str,
        value: LaunchWrapperOr[str]
) -> LaunchWrapper[SetLaunchConfiguration]:
    def __wrapper(context: LaunchContext) -> SetLaunchConfiguration:
        return SetLaunchConfiguration(
            name=name,
            value=__eval(value, context)
        )

    return __wrapper


def __dict_from_configs(
        names: List[str]
) -> LaunchWrapper[Mapping[str, str]]:
    def __wrapper(context: LaunchContext) -> Mapping[str, str]:
        return {
            name: LaunchConfiguration(name).perform(context)
            for name in names
        }

    return __wrapper


def __load_xacro(
        file_path: LaunchWrapperOr[Path],
        mappings: Optional[LaunchWrapperOr[Mapping[str, str]]] = None
) -> LaunchWrapper[str]:
    def __wrapper(context: LaunchContext) -> str:
        return load_xacro(
            file_path=__eval(file_path, context),
            mappings=__eval(mappings, context),
        )

    return __wrapper


def __make_opaque_function_that(
        *wrappers: List[LaunchWrapper[Any]]
) -> OpaqueFunction:
    def __wrapper(context: LaunchContext()):
        return [__eval(wrapper, context) for wrapper in wrappers]

    return OpaqueFunction(function=__wrapper)
