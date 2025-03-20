#!/usr/bin/env python

"""TODO.

Description.
"""
from collections.abc import (
    Callable,
    Iterator,
    Mapping,
)
from pathlib import Path
from typing import (
    Any,
    List,
    Text,
    TypeAlias,
    TypeVar,
    Union,
)

from launch import (
    LaunchContext,
    LaunchDescriptionEntity,
)
from launch.actions import (
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    # EnvironmentVariable,
    LaunchConfiguration,
)


T = TypeVar('T')

ContextValue: TypeAlias = Callable[[LaunchContext], T]
ContextValueOr: TypeAlias = Union[ContextValue[T], T]


def __eval(
        obj: ContextValueOr[T],
        context: LaunchContext
) -> T:
    """Return obj(context) when obj is callable, otherwise forward obj."""
    return obj(context) if callable(obj) else obj


def const(v: T) -> ContextValue[T]:
    """Return v as constant, independently of the context."""
    def __wrapper(context: LaunchContext) -> T:
        return v

    return __wrapper




def get_configs(
        *names: List[Text],
        transform: Callable[[Text], T] = lambda x: x,
        as_dict: bool = False,
) -> ContextValue[
    Union[
        T,                 # len(names) == 1
        Iterator[T],       # len(names) > 1 and as_dict = false
        Mapping[Text, T],  # len(names) > 1 and as_dict = true
    ]
]:
    """Output LaunchConfiguration(name)'s values.

    Parameters
    ----------
    names: List[Text]
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
        - Iterator[T] otherwise
    """
    def __wrapper(context: LaunchContext) -> Union[
            T,                 # len(names) == 1
            Iterator[T],       # len(names) > 1 and as_dict = 0
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

    return __wrapper


def set_config(
        name: Text,
        value: ContextValueOr[Text]
) -> ContextValue[None]:
    """Set the LaunchConfiguration(name)'s value.

    Parameters
    ----------
    name: Text
      The launch configuration's name
    value: ContextValueOr[Text]
      The launch configuration's value we wish to set

    Returns
    -------
    ContextValue[None]
      A ContextValue that set he launch config's value and returns None
    """
    def __wrapper(context: LaunchContext) -> None:
        SetLaunchConfiguration(name, __eval(value, context)).execute(context)
        return None

    return __wrapper


def load_xacro(
        file_path: ContextValueOr[Path],
        mappings: ContextValueOr[Mapping[Text, Text]] = {}
) -> ContextValue[Text]:
    """Return an URDF string from the given xacro file.

    Parameters
    ----------
    file_path: ContextValueOr[Path]
      A valid file name path evaluated from the context
    mappings: ContextValueOr[Mapping[Text, Text]] (default: {})
      The xacro mappings dictionnary with arguments values in it

    Returns
    -------
    ContextValue[Text]
      A ContextValue that create an urdf string from the xacro file_path
    """
    def __wrapper(context: LaunchContext) -> Text:
        from launch_param_builder import load_xacro
        return load_xacro(
            file_path=__eval(file_path, context),
            mappings=__eval(mappings, context),
        )

    return __wrapper


def make_opaque_function_that(
        *wrappers: List[ContextValueOr[Any]]
) -> OpaqueFunction:
    """Todo."""
    def __is_subclass_of(base: Any):
        return lambda v: (v is not None) and issubclass(v, base)

    def __wrapper(context: LaunchContext()):
        return filter(
            __is_subclass_of(LaunchDescriptionEntity),
            (__eval(wrapper, context) for wrapper in wrappers)
        )

    return OpaqueFunction(function=__wrapper)
