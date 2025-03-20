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


def get_configs(
        *names: Iterable[Text],
        transform: Callable[[Text], T] = lambda x: x,
        as_dict: bool = False,
) -> ContextValue[
    Union[
        T,                 # len(names) == 1
        Generator[T],      # len(names) > 1 and as_dict = false
        Mapping[Text, T],  # len(names) > 1 and as_dict = true
    ]
]:
    """Output LaunchConfiguration(name)'s values.

    Parameters
    ----------
    names: Iterable[Text]
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
        - T the transformed launch config's value when only 1 name is given;
        - Mapping[Text, T] when as_dict is true
        - Generator[T] otherwise
    """
    def impl(context: LaunchContext) -> Union[
            T,                 # len(names) == 1
            Generator[T],      # len(names) > 1 and as_dict = 0
            Mapping[Text, T],  # len(names) > 1 and as_dict = 1
    ]:
        if len(names) == 1:
            return transform(LaunchConfiguration(names[0]).perform(context))
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
