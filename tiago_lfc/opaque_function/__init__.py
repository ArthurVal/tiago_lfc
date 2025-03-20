#!/usr/bin/env python

"""opaque_function module providing tools to generate ROS2 OpaqueFunctions."""

from .algorithms import (
    duplicate,
    for_each,
    reduce,
    transform,
)
from .compare import (
    equals,
    greater_or_equals,
    greater_than,
    less_or_equals,
    less_than,
    not_equals,
)
from .config import (
    get_configs,
    set_config,
)
from .context_value import (
    ContextValue,
    ContextValueOr,
    as_const,
    from_context,
)
from .log import (
    log
)
from .make_opaque_function_that import (
    make_opaque_function_that,
)
from .urdf import (
    from_xacro,
)

__all__ = [
    # context_value
    'ContextValue',
    'ContextValueOr',
    'from_context',
    'as_const',

    # make_opaque_function_that
    'make_opaque_function_that',

    # algorithms
    'duplicate',
    'for_each',
    'reduce',
    'transform',

    # compare
    'equals',
    'greater_or_equals',
    'greater_than',
    'less_or_equals',
    'less_than',
    'not_equals',

    # config
    'get_configs',
    'set_config',

    # log
    'log',

    # urdf
    'from_xacro',
]
