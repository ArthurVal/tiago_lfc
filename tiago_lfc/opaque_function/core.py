#!/usr/bin/env python

"""TODO.

Description.
"""
from collections.abc import (
    Callable,
    Iterable,
)

from typing import (
    Any,
    List,
    TypeAlias,
    TypeVar,
    Union,
)

from launch import (
    LaunchContext,
    LaunchDescriptionEntity,
)
from launch.actions import (
    OpaqueFunction,
)

T = TypeVar('T')

ContextValue: TypeAlias = Callable[[LaunchContext], T]
ContextValueOr: TypeAlias = Union[ContextValue[T], T]


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
    def __is_subclass_of(base: Any):
        return lambda v: (v is not None) and issubclass(v, base)

    def __wrapper(context: LaunchContext()):
        return filter(
            __is_subclass_of(LaunchDescriptionEntity),
            (from_context(context, v) for v in values)
        )

    return OpaqueFunction(function=__wrapper)
