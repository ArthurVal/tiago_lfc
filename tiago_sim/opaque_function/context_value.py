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


def perform_substitution(
        context: LaunchContext,
        value: Substituable[T]
) -> T:
    """Perform the substitution of the given value."""
    if isinstance(value, Substitution):
        return value.perform(context)
    elif callable(value):
        return value(context)
    else:
        return value


def as_const(v: T) -> FunctionSubstitution[T]:
    """Create a FunctionSubstitution that always return v."""

    def impl(*args, **kwargs) -> T:
        """Return {value}, independently of the context."""
        return v

    impl.__doc__ = impl.__doc__.format(value=v)
    return impl


def apply(
        f: Callable[[Iterable[Any], Mapping[Text, Any]], T],
        *args: Iterable[Substituable[Any]],
        **kwargs: Mapping[Text, Substituable[Any]],
) -> FunctionSubstitution[T]:
    """Call the function f with args and kwargs after substitution.

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
