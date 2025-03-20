#!/usr/bin/env python

"""Define."""
from collections.abc import (
    Callable,
    Generator,
    Iterable,
    Mapping,
)
from typing import (
    Any,
    Text,
)

from .context_value import (
    ContextValue,
    ContextValueOr,
    LaunchContext,
    T,
    from_context,
)


def duplicate(
        value: ContextValueOr[T],
        N: int,
) -> ContextValue[Generator[T]]:
    """Duplicate the evaluated value N times.

    Parameters
    ----------
    value: ContextValueOr[T]
      Context value evaluated from the LaunchContext
    N: int
      Number of duplication

    Returns
    -------
    ContextValue[Generator[T]]
      An iterable with value listed N times
    """
    def impl(context: LaunchContext) -> Generator[T]:
        v = from_context(context, value)
        return (v for _ in range(N))

    return impl


def for_each(
        f: Callable[[Any], None],
        values: ContextValueOr[Iterable[Any]],
) -> ContextValue[None]:
    """Call f() for each values contain within values.

    Parameters
    ----------
    f: Callable[[Any], None]
      Any callable transforming taking the values
    values: ContextValueOr[Iterable[Any]]
      Context values evaluated from the LaunchContext containing an iterable

    Returns
    -------
    ContextValue[Generator[Any]]
      An iterable with the result of calling f() for each values
    """
    def impl(context: LaunchContext) -> None:
        [f(v) for v in from_context(context, values)]
        return None

    return impl


def apply(
        f: Callable[[Iterable[Any], Mapping[Text, Any]], T],
        args: ContextValueOr[Iterable[Any]],
        kwargs: ContextValueOr[Mapping[Text, Any]],
) -> ContextValue[T]:
    """Todo.

    Parameters
    ----------
    f: Callable[[Iterable[Any], Mapping[Text, Any]], T]
      Todo.
    args: ContextValueOr[Iterable[Any]]
      Context values evaluated from the LaunchContext containing an iterable
    kwargs: ContextValueOr[Mapping[Text, Any]]
      Context values evaluated from the LaunchContext containing a Mapping

    Returns
    -------
    ContextValue[T]
      TODO
    """
    def impl(context: LaunchContext) -> Generator[Any]:
        return f(*from_context(context, args), **from_context(context, kwargs))

    return impl


def transform(
        f: Callable[[Any], Any],
        values: ContextValueOr[Iterable[Any]],
) -> ContextValue[Generator[Any]]:
    """Transform the iterable values into a Generator containing f(v).

    Correspond to the following:
    VALUES  --> RESULT
    [V_0]   --> [R_0 = f(V_0)]
    [V_1]   --> [R_1 = f(V_1)]
    [...]   --> [... = f(...)]
    [V_N]   --> [V_N = f(V_N)]

    Parameters
    ----------
    f: Callable[[Any], Any]
      Any callable transforming the value
    values: ContextValueOr[Iterable[Any]]
      Context values evaluated from the LaunchContext containing an iterable

    Returns
    -------
    ContextValue[Generator[Any]]
      An iterable with the result of calling f() for each values
    """
    def impl(context: LaunchContext) -> Generator[Any]:
        return (
            f(v) for v in from_context(context, values)
        )

    return impl


def reduce(
        f: Callable[[T, Any], T],
        init: ContextValueOr[T],
        values: ContextValueOr[Iterable[Any]],
) -> ContextValue[T]:
    """Reduce the given values using init = f(init, v) for each values.

    Correspond to the following:
    RESULT = f(f(f(f(init, V_0), V_1), ...), V_N)

    Or:
    VALUES [V_0]  [V_1]  [...]  [V_N]
             |      |      |      |
             V      V      V      V
    init -> f() -> f() -> f() -> f() -> RESULT

    Parameters
    ----------
    f: Callable[[T, Any], T]
      Any callable taking init, evaluated value and returning something
      assigned to init
    init: ContextValueOr[T]
      init value
    values: ContextValueOr[Iterable[Any]]
      values evaluated from the LaunchContext containing an iterable

    Returns
    -------
    ContextValue[T]
      The content of init after reduction
    """
    def impl(context: LaunchContext) -> T:
        out = from_context(context, init)
        for v in from_context(context, values):
            out = f(out, v)
        return out

    return impl
