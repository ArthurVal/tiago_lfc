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


def log(
        fmt: Text,
        *args: Iterable[ContextValueOr[Any]],
        logger: Logger = get_logger('launch.user'),
        level: int = INFO,
        **kwargs: Mapping[Text, ContextValueOr[Any]],
) -> ContextValue[None]:
    """Log.

    Parameters
    ----------
    fmt: Text
      TODO
    *args: Iterable[ContextValueOr[Any]]
      TODO
    logger: logging.Logger
      TODO
    level: int
      TODO
    **kwargs: Mapping[Text, ContextValueOr[Any]]
      TODO

    Returns
    -------
    ContextValue[None]
      A ContextValue simply logging things after context' evaluation
    """
    def impl(context: LaunchContext) -> None:
        logger.log(
            level=level,
            msg=fmt.format(
                *[from_context(context, v) for v in args],
                **{k: from_context(context, v) for k, v in kwargs.items()}
            )
        )
    return impl
