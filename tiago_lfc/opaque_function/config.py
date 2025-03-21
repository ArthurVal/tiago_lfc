#!/usr/bin/env python

"""Define helpers working on launch configurations."""

from collections.abc import (
    Callable,
    Generator,
    Iterable,
    Mapping,
)
from typing import (
    Text,
    Union,
)

from launch.actions import (
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from .context_value import (
    ContextValue,
    ContextValueOr,
    LaunchContext,
    T,
    from_context,
)


def __do_nothing(x: Text) -> Text:
    return x


def get_configs(
        names: Union[Text, Iterable[Text]],
        *,
        transform: Callable[[Text], T] = __do_nothing,
        as_dict: bool = False,
) -> ContextValue[Union[T, Generator[T], Mapping[Text, T]]]:
    """Retreive a LaunchConfiguration(name)'s value(s).

    Parameters
    ----------
    names: Union[Text, Iterable[Text]]
      The launch configuration's names
    transform: Callable[[Text], T] (default: lambda x: x)
      A Callable that transform the launch configuration's value before storing
      it inside the container (default to identity).
    as_dict: bool (default: False)
      Set to True if you wish to output a Mapping[Text, T]

    Returns
    -------
    ContextValue[..]
      A ContextValue that create either:
        - T the transformed launch config's value only when names is a Text
        - Mapping[Text, T] when as_dict is true
        - Generator[T] otherwise
    """
    def impl(context: LaunchContext) -> Union[
            T,                 # names is Text
            Generator[T],      # len(names) > 1 and as_dict = 0
            Mapping[Text, T],  # len(names) > 1 and as_dict = 1
    ]:
        if isinstance(names, str):
            return transform(LaunchConfiguration(names).perform(context))
        else:
            if as_dict:
                return {
                    name: transform(LaunchConfiguration(name).perform(context))
                    for name in names
                }
            else:
                return (
                    transform(LaunchConfiguration(name).perform(context))
                    for name in names
                )

    return impl


def set_config(
        name: Text,
        value: ContextValueOr[Text]
) -> ContextValue[SetLaunchConfiguration]:
    """Set the LaunchConfiguration(name)'s value.

    Parameters
    ----------
    name: Text
      The launch configuration's name
    value: ContextValueOr[Text]
      The launch configuration's value we wish to set

    Returns
    -------
    ContextValue[SetLaunchConfiguration]
      A ContextValue that set he launch config's value and returns None
    """
    def impl(context: LaunchContext) -> SetLaunchConfiguration:
        return SetLaunchConfiguration(
            name,
            from_context(context, value)
        )

    return impl
