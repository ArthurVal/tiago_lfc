#!/usr/bin/env python

"""opaque_function module providing tools to generate ROS2 OpaqueFunctions."""
from .config import (
    get_configs,
    get_envs,
    set_config,
)
from .context_value import (
    FunctionSubstitution,
    Substituable,
    apply,
    as_const,
    substitue,
)
from .log import (
    do_format,
    log,
)
from .make_opaque_function_that import (
    make_opaque_function_that,
)
from .urdf import (
    from_xacro,
)

__all__ = [
    # context_value
    'apply',
    'FunctionSubstitution',
    'Substituable',
    'substitute',
    'as_const',

    # make_opaque_function_that
    'make_opaque_function_that',

    # config
    'get_envs',
    'get_configs',
    'set_config',

    # log
    'do_format',
    'log',

    # urdf
    'from_xacro',
]
