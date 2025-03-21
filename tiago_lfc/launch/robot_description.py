#!/usr/bin/env python

"""Provide utils function to populate decription with robot_description."""

from collections.abc import (
    Iterable,
)
from pathlib import (
    Path,
)
from pprint import (
    pformat,
)
from typing import (
    Text,
)

from launch import (
    LaunchDescription,
)

from tiago_lfc.opaque_function import (
    apply,
    do_format,
    from_xacro,
    get_configs,
    log,
    make_opaque_function_that,
    set_config,
)

from . import (
    logger,
    make_arguments_from_yaml,
)


def make_robot_description_from_xacro(
        file_path: Path,
        mappings_config_names: Iterable[Text],
        *,
        description: LaunchDescription = LaunchDescription(),
        name: Text = 'robot_description',
) -> LaunchDescription:
    """Create a Description with the robot_description set from a xacro.

    Parameters
    ----------
    file_path: Path
      Path to the xacro file
    mappings_config_names: Iterable[Text]
      List of launch configuration names defining all xacro mappings used
    description: LaunchDescription
      If defined, use this description instead of creating a new one
    name: Text
      The robot_description name used when setting the launch config

    Returns
    -------
    LaunchDescription
      The launch description populated with robot_description
    """
    logger.debug('Will populate "robot_description" from [XACRO]')
    logger.debug('{file_path}'.format(file_path=file_path))
    logger.debug(
        'Using mappings from launch configuration:\n{names}'.format(
            names=mappings_config_names
        )
    )
    all_mappings_args_value = get_configs(mappings_config_names, as_dict=True)
    description.add_action(
        make_opaque_function_that(
            log(
                msg=do_format(
                    (
                        'Xacro Mappings arguments evaluated to:'
                        '\n{mappings}'
                    ),
                    mappings=apply(pformat, all_mappings_args_value)
                ),
                logger=logger,
            ),
            set_config(
                name=name,
                value=from_xacro(
                    file_path=file_path,
                    mappings=all_mappings_args_value
                ),
            ),
        )
    )
    return description


def make_robot_description_from_tiago_description(
        *,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Create a Description with robot_description from 'tiago_description'.

    Parameters
    ----------
    description: LaunchDescription
      If defined, use this description instead of creating a new one

    Returns
    -------
    LaunchDescription
      The launch description populated with robot_description
    """
    from ament_index_python.packages import get_package_share_directory

    description, args_names = make_arguments_from_yaml(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'config',
            'tiago_configuration.yaml',
        ),
        description=description,
    )

    description = make_robot_description_from_xacro(
        file_path=Path(
            get_package_share_directory('tiago_description'),
            'robots',
            'tiago.urdf.xacro',
        ),
        mappings_config_names=args_names,
        description=description,
    )

    return description
