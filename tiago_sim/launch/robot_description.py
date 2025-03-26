#!/usr/bin/env python

"""Provide utils function to populate decription with robot_description."""

from collections.abc import (
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
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_param_builder import (
    load_xacro,
)

from tiago_sim.opaque_function import (
    Substituable,
    apply,
    do_format,
    get_configs,
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


def __write_to_file(
        file_path: Path,
        txt: Text,
        *,
        logger: type(logger) = logger,
) -> None:
    if file_path != '':
        logger.info(f'Dumping robot_description to {file_path}')
        with open(file_path, 'w') as f:
            f.write(txt)


def add_robot_description_from_xacro(
        *,
        file_path: Optional[Substituable[Path]] = None,
        mappings: Optional[Substituable[Mapping[Text, Text]]] = None,
        output_file: Optional[Substituable[Path]] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Create a Configuration with the robot_description from a xacro.

    Parameters
    ----------
    file_path: Optional[Substituable[Path]]
      Path to the xacro file. If None, declare a mandatory Launch argument
      for it
    mappings: Optional[Substituable[Mapping[Text, Text]]]
      Mappings of the XACRO. If None, declare a launch argument for it
      (default = {})
    description: LaunchDescription
      If defined, use this description instead of creating a new one
    output_file: Optional[Substituable[Path]]
      If given, will write the content of robot_description to the given file

    Returns
    -------
    LaunchDescription
      The launch description populated with robot_description
    """
    if file_path is None:
        description.add_action(
            DeclareLaunchArgument(
                'file_path',
                description=(
                    'XACRO file path use to create the robot_description URDF'
                ),
            )
        )
        file_path = LaunchConfiguration('file_path')

    if mappings is None:
        raise NotImplementedError(
            (
                'TODO: Need to implement a launch argument parser to transform'
                ' a string to a dict OR a way to introspect xacro arguments'
                ' a posteriori'
            )
        )

    if output_file is None:
        description.add_action(
            DeclareLaunchArgument(
                'output_file',
                description=(
                    'If not empty, a valid file name (will be created) used to'
                    ' write the robot_description into'
                ),
                default_value='',
            )
        )
        output_file = LaunchConfiguration('output_file')

    description.add_action(
        make_opaque_function_that(
            log(
                msg=do_format(
                    (
                        'Creating robot_description:'
                        '\n- From XACRO {file_path}'
                        '\n- Using mappings:'
                        '\n{mappings}'
                    ),
                    file_path=file_path,
                    mappings=apply(
                        dict_to_string,
                        value=mappings,
                        kv_header='--> ',
                    )
                ),
                logger=logger,
            ),
            set_config(
                name='robot_description',
                value=apply(
                    load_xacro,
                    file_path=file_path,
                    mappings=mappings,
                ),
            ),
            apply(
                __write_to_file,
                file_path=output_file,
                txt=get_configs('robot_description'),
                logger=logger,
            )
        )
    )

    return description
