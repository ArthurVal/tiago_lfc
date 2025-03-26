#!/usr/bin/env python

"""Generic URDF manipulation helpers."""

from collections.abc import (
    Mapping,
)
from pathlib import (
    Path,
)
from typing import (
    Text,
)

from launch_param_builder import load_xacro

from .apply import (
    FunctionSubstitution,
    Substituable,
    apply,
)


def from_xacro(
        file_path: Substituable[Path],
        mappings: Substituable[Mapping[Text, Text]] = {}
) -> FunctionSubstitution[Text]:
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
    return apply(load_xacro, file_path=file_path, mappings=mappings)
