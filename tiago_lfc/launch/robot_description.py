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
    Optional,
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
)


def __write_to_file(file_path: Path, txt: Text) -> None:
    with open(file_path, 'w') as f:
        f.write(txt)


def make_robot_description_from_xacro(
        file_path: Path,
        mappings_config_names: Iterable[Text],
        *,
        description: LaunchDescription = LaunchDescription(),
        config_name: Text = 'robot_description',
        output_file: Optional[Path] = None,
) -> LaunchDescription:
    """Create a Configuration with the robot_description from a xacro.

    Parameters
    ----------
    file_path: Path
      Path to the xacro file
    mappings_config_names: Iterable[Text]
      List of launch configuration names defining all xacro mappings used
    description: LaunchDescription
      If defined, use this description instead of creating a new one
    config_name: Text
      The robot_description name used when setting the launch config
    output_file: Optional[Path]
      If given, will write the content of robot_description to the given file

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
                name=config_name,
                value=from_xacro(
                    file_path=file_path,
                    mappings=all_mappings_args_value
                ),
            ),
        )
    )

    if output_file is not None:
        description.add_action(
            make_opaque_function_that(
                log(
                    f'Dumping {config_name} to file {output_file}',
                    logger=logger,
                ),
                apply(
                    __write_to_file,
                    file_path=output_file,
                    txt=get_configs(config_name),
                )
            )
        )

    return description
