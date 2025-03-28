#!/usr/bin/env python

"""Provide utils launch function handling DeclareArgument."""

from collections.abc import (
    Generator,
)
from pathlib import (
    Path,
)

from launch.actions import (
    DeclareLaunchArgument,
)


def all_arguments_from_yaml(
        file_path: Path,
) -> Generator[DeclareLaunchArgument]:
    """Declare arguments directly imported from a yaml.

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
    return (
        DeclareLaunchArgument(name, **params)
        for name, params in load_yaml(file_path).items()
    )
