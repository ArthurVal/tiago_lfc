#!/usr/bin/env python

"""tiago_lfc launch module main entry points."""
from launch.logging import get_logger

logger = get_logger('launch.user')

from .arguments import (
    make_arguments_from_yaml,
)
from .robot_description import (
    make_robot_description_from_tiago_description,
    make_robot_description_from_xacro,
)
from .robot_state_publisher import (
    make_robot_state_publisher
)
__all__ = [
    'logger',

    # arguments
    'make_arguments_from_yaml',

    # robot_description
    'make_robot_description_from_tiago_description',
    'make_robot_description_from_xacro',

    # robot_state_publisher
    'make_robot_state_publisher',
]
