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
