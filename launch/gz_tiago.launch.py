#!/usr/bin/env python

"""Completely launch tiago inside GZ in one go."""
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
    Invoke,
    add_robot_description_from_xacro,
    declare_arguments_from_yaml,
    gz_server,
    gz_spawn_entity,
    run_robot_state_publisher,
)


def generate_launch_description():
    """Launch tiago within GZ."""
    tiago_xacro_file = Path(
        get_package_share_directory('tiago_description'),
        'robots',
        'tiago.urdf.xacro',
    )

    tiago_xacro_mappgins_yaml_description = Path(
        get_package_share_directory('tiago_description'),
        'config',
        'tiago_configuration.yaml',
    )

    description, xacro_mappings_argument_names = declare_arguments_from_yaml(
        file_path=tiago_xacro_mappgins_yaml_description,
    )

    # Since we are simulating, use_sim_time is FORCED here
    description.add_action(
        SetLaunchConfiguration(
            'use_sim_time',
            'True',
        ),
    )

    # 'use_sim_time' it not included in the yaml file but used inside the xacro
    # as a mapping to select Gazebo stuff...
    # We add it by hand.
    xacro_mappings_argument_names.append('use_sim_time')

    # FIXME: We shouldn't create a file at this location...
    tiago_urdf_file = tiago_xacro_file.with_suffix('.urdf')
    # Some possible work around TBD:
    # - Create a tmp file ?
    # - Use a fifo (not portable in windows and I don't even know if it works
    #   well with gz_spawn...) ?
    # - Find out how to forward a valid value using 'sdf' param instead of
    #   'sdf_filename' in gz service /world/create (EntityFactory msg) ?
    # - ... ?

    add_robot_description_from_xacro(
        file_path=tiago_xacro_file,
        mappings={
            names: LaunchConfiguration(names)
            for names in xacro_mappings_argument_names
        },
        # TODO: Remove this
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

    # NOTE:
    # gz_server declare 'world' as an argument, but require a '.sdf' file.
    # Then, when calling GZ services associated to that world, the world is
    # mentionned without its file extension.
    # This is this 'world' (without the file extension) that is used by
    # gz_spawn_entity below.
    world = LaunchConfiguration('world')

    # This remove the file extension from 'world'
    description.add_action(
        Invoke(
            lambda v: Path(v).stem,
            world
        ).and_then_with_key(
            'value',
            SetLaunchConfiguration,
            name='world'
        )
    )

    gz_spawn_entity(
        model_path=tiago_urdf_file,
        name='tiago',
        world=world,
        timeout_ms=1000,
        description=description,
    )

    return description
