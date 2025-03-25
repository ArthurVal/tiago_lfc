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

from .context_value import (
    ContextValue,
    ContextValueOr,
    apply,
)


def do_format(
        fmt: Text,
        *args: Iterable[ContextValueOr[Any]],
        **kwargs: Mapping[Text, ContextValueOr[Any]],
) -> ContextValue[Text]:
    """Call fmt.format(*args, **kwargs) after evaluating args/kwargs.

    Parameters
    ----------
    fmt: Text
      Fmt string used by the log
    *args: Iterable[ContextValueOr[Any]]
      List of ContextValueOr forwarded to the .format() function
    **kwargs: Mapping[Text, ContextValueOr[Any]]
      Map of Keys/ContextValueOr forwarded to the .format() function

    Returns
    -------
    ContextValue[Text]
      A ContextValue calling fmt.format()
    """
    return apply(str.format, fmt, *args, **kwargs)


def log(
        msg: ContextValueOr[Text],
        *args,
        level: ContextValueOr[int] = INFO,
        logger: Logger = get_logger('launch.user'),
        **kwargs,
) -> ContextValue[None]:
    """Log.

    Parameters
    ----------
    msg: ContextValueOr[Text]
      Msg to log
    level: int
      Log level used
    logger: logging.Logger
      logging.Logger used to log to

    Returns
    -------
    ContextValue[None]
      A ContextValue simply logging things after context' evaluation
    """
    return apply(Logger.log, logger, level, msg, *args, **kwargs)
