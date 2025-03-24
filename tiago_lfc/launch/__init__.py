#!/usr/bin/env python

"""tiago_lfc launch module main entry points."""
from launch.logging import get_logger

logger = get_logger('launch.user')

from .arguments import (
    make_arguments_from_yaml,
)
from .gz import (
    make_gz_server,
    make_gz_spawn,
)
from .robot_description import (
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
    'make_robot_description_from_xacro',

    # robot_state_publisher
    'make_robot_state_publisher',

    # gz
    'make_gz_server',
    'make_gz_spawn',
]
