#!/usr/bin/env python

"""Declare main types used by opaque_function."""

from collections.abc import (
    Callable,
)
from typing import (
    TypeAlias,
    TypeVar,
    Union,
)

from launch import (
    LaunchContext,
    Substitution,
)

T = TypeVar('T')

# Custom substitution using functional programming style (used by the
# opaque_function module)
FunctionSubstitution: TypeAlias = Callable[[LaunchContext], T]

# Argument type of functions from the opaque_function module that wish to be
# plugged to other opaque_function outputs.
Substituable: TypeAlias = Union[
    Substitution,
    FunctionSubstitution[T],
    T
]


def substitute(
        context: LaunchContext,
        value: Substituable[T]
) -> T:
    """Perform the substitution of the given value, if relevant."""
    if isinstance(value, Substitution):
        return value.perform(context)
    elif callable(value):
        return value(context)
    else:
        return value
