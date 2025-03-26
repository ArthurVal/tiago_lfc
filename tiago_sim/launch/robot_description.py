#!/usr/bin/env python

"""Provide utils function to populate decription with robot_description."""

from collections.abc import (
    Iterable,
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

from tiago_sim.opaque_function import (
    Substituable,
    apply,
    do_format,
    from_xacro,
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


def __write_to_file(file_path: Path, txt: Text) -> None:
    with open(file_path, 'w') as f:
        f.write(txt)


def add_robot_description_from_xacro(
        file_path: Path,
        mappings_config_names: Iterable[Text],
        *,
        output_file: Optional[Substituable[Path]] = None,
        description: LaunchDescription = LaunchDescription(),
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
    output_file: Optional[Substituable[Path]]
      If given, will write the content of robot_description to the given file

    Returns
    -------
    LaunchDescription
      The launch description populated with robot_description
    """
    logger.debug('Will populate "robot_description" from [XACRO]')
    logger.debug(f'{file_path}')
    logger.debug(
        f'Using mappings from launch configuration:\n{mappings_config_names}'
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
                    mappings=apply(
                        dict_to_string,
                        all_mappings_args_value,
                        kv_header='--> ',
                    )
                ),
                logger=logger,
            ),
            set_config(
                name='robot_description',
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
                    f'Dumping robot_description to {output_file}',
                    logger=logger,
                ),
                apply(
                    __write_to_file,
                    file_path=output_file,
                    txt=get_configs('robot_description'),
                )
            )
        )

    return description
