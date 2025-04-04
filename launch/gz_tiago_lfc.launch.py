#!/usr/bin/env python

"""Completely launch tiago inside GZ in one go."""
from itertools import (
    chain,
)
from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
)
from launch.actions import (
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from tiago_lfc.launch import (
    Invoke,
    add_robot_description_from_xacro,
    all_arguments_from_yaml,
    evaluate_dict,
    gz_server,
    gz_spawn_entity,
    load_controllers,
    run_robot_state_publisher,
    # switch_controllers,
)


def generate_launch_description():
    """Launch tiago within GZ."""
    xacro_file = Path(
        get_package_share_directory('tiago_description'),
        'robots',
        'tiago.urdf.xacro',
    )

    xacro_args = list(
        all_arguments_from_yaml(
            Path(
                get_package_share_directory('tiago_description'),
                'config',
                'tiago_configuration.yaml',
            )
        )
    )

    # FIXME: We shouldn't create a file at this location...
    urdf_file = xacro_file.with_suffix('.urdf')
    # Some possible work around TBD:
    # - Create a tmp file ?
    # - Use a fifo (not portable in windows and I don't even know if it works
    #   well with gz_spawn...) ?
    # - Find out how to forward a valid value using 'sdf' param instead of
    #   'sdf_filename' in gz service /world/create (EntityFactory msg) ?
    # - ... ?

    spawn_robot_state_publisher = chain(
        add_robot_description_from_xacro(
            file_path=xacro_file,
            mappings=evaluate_dict(
                {
                    arg.name: LaunchConfiguration(arg.name)
                    for arg in xacro_args
                } | {
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ),
            output_file=urdf_file,
        ),
        run_robot_state_publisher(
            robot_description=LaunchConfiguration('robot_description'),
            use_sim_time=LaunchConfiguration('use_sim_time'),
        ),
    )

    start_gz = chain(
        gz_server(),
        gz_spawn_entity(
            model_path=urdf_file,
            name='tiago',
            world=Invoke(
                # Remove the file extension from world defined by gz_server()
                lambda v: Path(v).stem,
                LaunchConfiguration('world'),
            ),
            timeout_ms=1000,
        )
    )

    load_lfc_jse = load_controllers(
        controllers=('lfc', 'jse'),
        param_file=Path(
            get_package_share_directory('tiago_lfc'),
            'config',
            'lfc_parameters.yaml',
        ),
        activate=False,
        controller_manager='/controller_manager',
    )

    return LaunchDescription(
        chain(
            (
                # Since we are simulating, use_sim_time is FORCED here
                SetLaunchConfiguration(
                    'use_sim_time',
                    'True',
                ),
            ),
            xacro_args,
            spawn_robot_state_publisher,
            start_gz,
            load_lfc_jse,
        )
    )
