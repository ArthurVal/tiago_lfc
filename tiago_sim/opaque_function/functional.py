#!/usr/bin/env python

"""TODO."""
from collections.abc import (
    Callable,
    Iterable,
    Mapping,
)
from typing import (
    Any,
    Text,
)

from .substitute import (
    FunctionSubstitution,
    LaunchContext,
    Substituable,
    T,
    substitute,
)


def invoke(
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
    args: Substituable[Iterable[Any]]
      C-values evaluated and then forward to f
    kwargs: Substituable[Mapping[Text, Any]]
      Mapping of key/c-values evaluated and then forward to f

    Returns
    -------
    FunctionSubstitution[T]
      A callable returning the result of calling call f

    """
    def impl(context: LaunchContext) -> T:
        return f(
            *[substitute(context, arg) for arg in args],
            **{k: substitute(context, v) for k, v in kwargs.items()}
        )

    return impl
