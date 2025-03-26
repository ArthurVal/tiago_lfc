#!/usr/bin/env python

"""opaque_function module providing tools to generate ROS2 OpaqueFunctions."""
from .config import (
    get_configs,
    get_envs,
    set_config,
)
from .functional import (
    invoke,
)
from .log import (
    do_format,
    log,
)
from .make_opaque_function_that import (
    make_opaque_function_that,
)
from .substitute import (
    FunctionSubstitution,
    Substituable,
    substitute,
)

__all__ = [
    # substitute
    'FunctionSubstitution',
    'Substituable',
    'substitute',

    # functional
    'invoke',

    # make_opaque_function_that
    'make_opaque_function_that',

    # config
    'get_envs',
    'get_configs',
    'set_config',

    # log
    'do_format',
    'log',
]
