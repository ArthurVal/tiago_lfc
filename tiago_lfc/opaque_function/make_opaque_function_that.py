#!/usr/bin/env python

"""TODO.

Description.
"""
from collections.abc import (
    Iterable,
)
from typing import (
    Any,
    Type,
)

from launch import (
    LaunchContext,
    LaunchDescriptionEntity,
)
from launch.actions import (
    OpaqueFunction,
)

from .context_value import (
    ContextValueOr,
    from_context,
)


def make_opaque_function_that(
        *values: Iterable[ContextValueOr[Any]]
) -> OpaqueFunction:
    """Create an OpaqueFunction from the given context's values.

    This is the main entry point of the opaque_function_helper module.

    Notes
    -----
      - Values returning LaunchDescriptionEntity subclasses will be forwarded
        to the OpaqueFunction, otherwise they are filtered out
    """
    def __is_subclass_of(base: Type):
        return lambda v: (v is not None) and issubclass(type(v), base)

    def __wrapper(context: LaunchContext):
        return filter(
            __is_subclass_of(LaunchDescriptionEntity),
            (from_context(context, v) for v in values)
        )

    return OpaqueFunction(function=__wrapper)
