#!/usr/bin/env python

"""TODO."""

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetLaunchConfiguration,
)
# from launch.substitutions import (
#     LaunchConfiguration,
# )

from launch_ros.actions import (
    Node,
)

from tiago_lfc.opaque_function import (
    do_format,
    get_configs,
    log,
    make_opaque_function_that,
)

from . import (
    logger,
)


def make_gz_server(
        *,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """TODO."""
    description.add_action(
        SetLaunchConfiguration(
            'use_sim_time',
            'True',
        )
    )

    description.add_action(
        make_opaque_function_that(
            log(
                msg=do_format(
                    (
                        'Spawning GZ server: TODO'
                    ),
                ),
                logger=logger,
            )
        )
    )

    description.add_action(
        ExecuteProcess(
            cmd=[
                'ls',
            ],
        )
    )

    # TODO

    return description


def make_gz_spawn(
        *,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Spawn a robot_state_publisher, using robot_description provided."""
    description.add_action(
        DeclareLaunchArgument(
            'entity_name',
            description='Name of the entity spawn inside gz',
            default_value='tiago',
        )
    )

    description.add_action(
        make_opaque_function_that(
            log(
                msg=do_format(
                    (
                        'Spawning GZ entity:'
                        '\n - name: {name}'
                    ),
                    name=get_configs('entity_name'),
                ),
                logger=logger,
            )
        )
    )

    return description
