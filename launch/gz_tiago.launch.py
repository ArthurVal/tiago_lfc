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
    get_configs,
)


def generate_launch_description():
    """TODO."""
    # This must be first, since make_gz_server force use_sim_time to True
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

    # gz_spawn needs an URDF file, we use this this location (TBD)
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
        namespace='',
        description=description
    )

    gz_server(
        world='empty.sdf',
        gui=True,
        description=description,
    )

    gz_spawn_entity(
        model=tiago_urdf_file,
        name='tiago',
        world='empty',
        timeout_ms=1000,
        description=description,
    )

    return description
