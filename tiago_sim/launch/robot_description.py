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
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_param_builder import (
    load_xacro,
)

from .invoke import (
    Invoke,
    SubstitutionOr,
)
from .logging import (
    logger,
)
from .utils import (
    dict_to_string,
)


def __write_and_forward(
        value: Text,
        *,
        file_path: Optional[Path] = None,
) -> Text:
    if file_path is not None:
        logger.info('Dumping robot_description to {}'.format(file_path))
        with open(file_path, 'w') as f:
            f.write(value)

    return value


def __load_xacro(file_path: Path, **mappings: [Text]) -> Text:
    logger.info(
        (
            'Loading XACRO:'
            '\n- File "{file_path}"'
            '\n- Using mappings:'
            '\n{mappings}'
        ).format(
            file_path=file_path,
            mappings=dict_to_string(
                mappings,
                kv_header='--> ',
            )
        )
    )

    return load_xacro(file_path, mappings=mappings)


def add_robot_description_from_xacro(
        *,
        file_path: Optional[SubstitutionOr[Path]] = None,
        mappings: Optional[Mapping[Text, SubstitutionOr[Text]]] = None,
        output_file: Optional[SubstitutionOr[Path]] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Create a Configuration with the robot_description from a xacro.

    Parameters
    ----------
    file_path: Optional[MaybeSubstituable[Path]]
      Path to the xacro file. If None, declare a mandatory Launch argument
      for it
    mappings: Optional[Mapping[Text, MaybeSubstituable[Text]]]
      Mappings of the XACRO.
    description: LaunchDescription
      If defined, use this description instead of creating a new one
    output_file: Optional[MaybeSubstituable[Path]]
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
        # TODO: mappings string parser from launch argument ?
        # e.g. mappings:='toto:1, tata:2' -> {'toto': '1', 'tata': '2'}
        mappings = {}

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
        output_file = Invoke(
            lambda txt: None if txt == '' else Path(txt),
            LaunchConfiguration('output_file'),
        )

    description.add_action(
        Invoke(
            __load_xacro,
            file_path=file_path,
            **mappings,
        ).and_then(
            __write_and_forward,
            file_path=output_file,
        ).and_then_with_key(
            'value',
            SetLaunchConfiguration,
            name='robot_description',
        )
    )

    return description
