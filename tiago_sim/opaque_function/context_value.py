#!/usr/bin/env python

"""TODO.

Description.
"""
from collections.abc import (
    Callable,
    Iterable,
    Mapping,
)
from typing import (
    Any,
    Text,
    TypeAlias,
    TypeVar,
    Union,
)

from launch import (
    LaunchContext,
)

T = TypeVar('T')

# Value that is meant to be evaluated later on, given a LaunchContext.
# It is the main return type of all function in the opaque_function module that
# is meant to be plugged into each other.
ContextValue: TypeAlias = Callable[[LaunchContext], T]

# Argument type of function from the opaque_function module that wish to be
# plugged to other opaque_function outputs.
ContextValueOr: TypeAlias = Union[ContextValue[T], T]


def perform_substitution(context: LaunchContext, v: ContextValueOr[T]) -> T:
    """Return the result of v(context) when v is callable, otherwise v."""
    return v(context) if callable(v) else v


def as_const(v: T) -> ContextValue[T]:
    """Create a ContextValue that always return v."""

    def impl(*args, **kwargs) -> T:
        """Return v as constant, independently of the context."""
        return v

    return impl


def no_opt() -> ContextValue[None]:
    """Do nothing."""
    return as_const(None)


def apply(
        f: Callable[[Iterable[Any], Mapping[Text, Any]], T],
        *args: Iterable[ContextValueOr[Any]],
        **kwargs: Mapping[Text, ContextValueOr[Any]],
) -> ContextValue[T]:
    """Call the function f with args and kwargs after evaluation.

    This should be used to call traditional functions on unevaluated arguments
    coming from the LaunchConfiguration context.

    Parameters
    ----------
    f: Callable[[Iterable[Any], Mapping[Text, Any]], T]
      Any callable responsible for transforming the evaluated values.
    args: ContextValueOr[Iterable[Any]]
      C-values evaluated and then forward to f
    kwargs: ContextValueOr[Mapping[Text, Any]]
      Mapping of key/c-values evaluated and then forward to f

    Returns
    -------
    ContextValue[T]
      A callable returning the result of calling call f

    """
    def impl(context: LaunchContext) -> T:
        return f(
            *[perform_substitution(context, arg) for arg in args],
            **{k: perform_substitution(context, v) for k, v in kwargs.items()}
        )

    return impl
