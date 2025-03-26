#!/usr/bin/env python

"""TODO."""

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from tiago_sim.launch import (
    add_robot_description_from_xacro,
    declare_arguments_from_yaml,
    gz_server,
    gz_spawn_entity,
    run_robot_state_publisher,
)
from tiago_sim.opaque_function import (
    apply,
    get_configs,
    make_opaque_function_that,
    set_config,
)


def generate_launch_description():
    """TODO."""
    description, args_names = declare_arguments_from_yaml(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'config',
            'tiago_configuration.yaml',
        ),
    )

    description.add_action(
        SetLaunchConfiguration(
            'use_sim_time',
            'True',
        ),
    )

    # 'use_sim_time' it not included in the .yaml file
    # We add it by hand.
    args_names.append('use_sim_time')

    # TODO: create a tmp file ?
    tiago_urdf_file = Path(
        get_package_share_directory('tiago_description'),
        'robots',
        'tiago.urdf',
    )

    add_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings=get_configs(args_names, as_dict=True),
        output_file=tiago_urdf_file,
        description=description,
    )

    run_robot_state_publisher(
        robot_description=LaunchConfiguration('robot_description'),
        use_sim_time=LaunchConfiguration('use_sim_time'),
        description=description
    )

    gz_server(
        description=description,
    )

    # We reuse the world of gz_server declared as argument
    # and update it (only use the stem) for gz_spawn_entity
    world = get_configs('world')
    description.add_action(
        make_opaque_function_that(
            set_config(
                'world',
                apply(
                    lambda w: Path(w).stem,
                    world
                )
            )
        )
    )

    gz_spawn_entity(
        model=tiago_urdf_file,
        name='tiago',
        world=world,
        timeout_ms=1000,
        description=description,
    )

    return description
