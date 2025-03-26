#!/usr/bin/env python

"""Define helpers use to log things."""

from collections.abc import (
    Iterable,
    Mapping,
)
from logging import (
    INFO,
    Logger,
)
from typing import (
    Any,
    Text,
)

from launch.logging import (
    get_logger,
)

from .functional import (
    FunctionSubstitution,
    Substituable,
    invoke,
)


def do_format(
        fmt: Text,
        *args: Iterable[Substituable[Any]],
        **kwargs: Mapping[Text, Substituable[Any]],
) -> FunctionSubstitution[Text]:
    """Call fmt.format(*args, **kwargs) after evaluating args/kwargs.

    Parameters
    ----------
    fmt: Text
      Fmt string used by the log
    *args: Iterable[Substituable[Any]]
      List of Substituable forwarded to the .format() function
    **kwargs: Mapping[Text, Substituable[Any]]
      Map of Keys/Substituable forwarded to the .format() function

    Returns
    -------
    FunctionSubstitution[Text]
      A FunctionSubstitution calling fmt.format()
    """
    return invoke(str.format, fmt, *args, **kwargs)


def log(
        msg: Substituable[Text],
        *args,
        level: Substituable[int] = INFO,
        logger: Logger = get_logger('launch.user'),
        **kwargs,
) -> FunctionSubstitution[None]:
    """Log.

    Parameters
    ----------
    msg: Substituable[Text]
      Msg to log
    level: int
      Log level used
    logger: logging.Logger
      logging.Logger used to log to

    Returns
    -------
    FunctionSubstitution[None]
      A FunctionSubstitution simply logging things after context' evaluation
    """
    return invoke(Logger.log, logger, level, msg, *args, **kwargs)
