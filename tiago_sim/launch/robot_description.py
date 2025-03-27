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
    MaybeSubstituable,
    evaluate_as_dict,
)
from .logging import (
    logger,
)
from .utils import (
    dict_to_string,
)


def __not_equals(v: Text):
    return lambda x: x != v


def __write_to_file_when_path(predicate, *, logger=logger):
    def impl(file_path: Path, txt: Text):
        if predicate(file_path):
            logger.info(f'Dumping robot_description to {file_path}')
            with open(file_path, 'w') as f:
                f.write(txt)

        return txt

    return impl


def add_robot_description_from_xacro(
        *,
        file_path: Optional[MaybeSubstituable[Path]] = None,
        mappings: Optional[Mapping[Text, MaybeSubstituable[Text]]] = None,
        output_file: Optional[MaybeSubstituable[Path]] = None,
        description: LaunchDescription = LaunchDescription(),
) -> LaunchDescription:
    """Create a Configuration with the robot_description from a xacro.

    Parameters
    ----------
    file_path: Optional[MaybeSubstituable[Path]]
      Path to the xacro file. If None, declare a mandatory Launch argument
      for it
    mappings: Optional[Mapping[Text, MaybeSubstituable[Text]]]
      Mappings of the XACRO. If None, declare a launch argument for it
      (default = {})
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
        evaluate_as_dict(**mappings).and_then(
            dict_to_string,
            kv_header='--> ',
        ).and_then_with_key(
            'mappings',
            (
                'Creating robot_description:'
                '\n- From XACRO {file_path}'
                '\n- Using mappings:'
                '\n{mappings}'
            ).format,
            file_path=file_path,
        ).and_then(
            logger.info
        ),
    )

    description.add_action(
        evaluate_as_dict(**mappings).and_then_with_key(
            'mappings',
            load_xacro,
            file_path=file_path,
        ).and_then_with_key(
            'txt',
            __write_to_file_when_path(__not_equals('')),
            file_path=output_file,
        ).and_then_with_key(
            'value',
            SetLaunchConfiguration,
            name='robot_description',
        )
    )

    return description
