#!/usr/bin/env python

"""opaque_function module providing tools to generate ROS2 OpaqueFunctions."""
from .algorithms import (
    duplicate,
    for_each,
    reduce,
    transform,
)
from .config import (
    get_configs,
    get_envs,
    set_config,
)
from .context_value import (
    ContextValue,
    ContextValueOr,
    apply,
    as_const,
    perform_substitution,
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
    'ContextValue',
    'ContextValueOr',
    'perform_substitution',
    'as_const',

    # make_opaque_function_that
    'make_opaque_function_that',

    # algorithms
    'duplicate',
    'for_each',
    'reduce',
    'transform',

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
