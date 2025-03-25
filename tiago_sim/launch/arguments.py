#!/usr/bin/env python

"""Provide utils launch function handling DeclareArgument."""

from pathlib import (
    Path,
)
from typing import (
    List,
    Text,
    Tuple,
)

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
)

from . import logger


def make_arguments_from_yaml(
        file_path: Path,
        *,
        description: LaunchDescription = LaunchDescription(),
) -> Tuple[LaunchDescription, List[Text]]:
    """Create a Descrition with arguments directly imported from a yaml.

    Parameters
    ----------
    file_path: Path
      Path to the yaml file
    description: LaunchDescription
      If defined, use this description instead of creating a new one

    Returns
    -------
    Tuple[LaunchDescription, List[Text]]
      The launch description populated with arguments from yaml and the list of
      args name added
    """
    from launch_param_builder import load_yaml

    arg_names = []

    logger.debug('Adding new launch arguments from yaml:')
    logger.debug('{path}'.format(path=file_path))
    for name, params in load_yaml(file_path).items():
        logger.debug('- {name}'.format(name=name))
        arg_names.append(name)
        description.add_action(
            DeclareLaunchArgument(name, **params)
        )

    return (description, arg_names)
