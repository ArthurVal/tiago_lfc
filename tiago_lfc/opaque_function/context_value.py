#!/usr/bin/env python

"""TODO.

Description.
"""
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
)

T = TypeVar('T')

ContextValue: TypeAlias = Callable[[LaunchContext], T]
ContextValueOr: TypeAlias = Union[ContextValue[T], T]


def from_context(context: LaunchContext, v: ContextValueOr[T]) -> T:
    """Return the result of v(context) when v is callable, otherwise v."""
    return v(context) if callable(v) else v


def as_const(v: T) -> ContextValue[T]:
    """Return v as constant, independently of the context."""
    return lambda _: v


def no_opt() -> ContextValue[None]:
    """Do nothing."""
    return as_const(None)
