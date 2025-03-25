#!/usr/bin/env python

"""TODO."""

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import (
    Node,
)

from tiago_sim.launch import (
    make_arguments_from_yaml,
    make_gz_server,
    make_gz_spawn,
    make_robot_description_from_xacro,
    make_robot_state_publisher,
)
from tiago_sim.opaque_function import (
    apply,
    get_configs,
    make_opaque_function_that,
    set_config,
)


def generate_launch_description():
    """TODO."""
    # This must be first, since make_gz_server force use_sim_time to True
    description = make_gz_server()

    description.add_action(
        make_opaque_function_that(
            # Force set the world used by gz_spawn to the one used to populate
            # the server
            set_config(
                'world',
                apply(
                    lambda v: Path(v).stem,
                    get_configs('world_file')
                )
            )
        )
    )

    description, args_names = make_arguments_from_yaml(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'config',
            'tiago_configuration.yaml',
        ),
        description=description,
    )

    # 'use_sim_time' it not included in the .yaml file
    # We add it by hand (forced to true by make_gz_server)
    args_names.append('use_sim_time')

    # gz_spawn needs an URDF file, we use this this location (TBD)
    tiago_urdf_file = Path(
        get_package_share_directory('tiago_description'),
        'robots',
        'tiago.urdf',
    )

    description = make_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings_config_names=args_names,
        output_file=tiago_urdf_file,
        description=description,
    )

    description = make_robot_state_publisher(
        description=description
    )

    # FIXME: Is this needed ?
    description.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )
    )

    return make_gz_spawn(
        model_path=tiago_urdf_file,
        description=description,
    )
