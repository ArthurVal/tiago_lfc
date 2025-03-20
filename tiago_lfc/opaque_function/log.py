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
    LaunchContext,
    from_context,
)


def do_format(
        fmt: Text,
        *args: Iterable[ContextValueOr[Any]],
        **kwargs: Mapping[Text, ContextValueOr[Any]],
) -> ContextValue[Text]:
    """Log.

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
    def impl(context: LaunchContext) -> Text:
        return fmt.format(
            *[from_context(context, arg) for arg in args],
            **{k: from_context(context, v) for k, v in kwargs.items()},
        )

    return impl


def log(
        msg: ContextValueOr[Text],
        *,
        level: ContextValueOr[int] = INFO,
        logger: Logger = get_logger('launch.user'),
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
    def impl(context: LaunchContext) -> None:
        logger.log(
            level=from_context(context, level),
            msg=from_context(context, msg)
        )
    return impl
