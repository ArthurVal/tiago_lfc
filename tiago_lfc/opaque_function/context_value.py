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


def apply(
        f: Callable[[Iterable[Any], Mapping[Text, Any]], T],
        *args: Iterable[ContextValueOr[Any]],
        **kwargs: Mapping[Text, ContextValueOr[Any]],
) -> ContextValue[T]:
    """Call the function f with *args and **kwargs after evaluation.

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
            *[from_context(context, arg) for arg in args],
            **{k: from_context(context, v) for k, v in kwargs.items()}
        )

    return impl
