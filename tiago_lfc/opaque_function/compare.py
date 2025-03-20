#!/usr/bin/env python

"""Define generic comparison helpers."""

from .context_value import (
    ContextValue,
    ContextValueOr,
    from_context,
)


def equals(
        lhs: ContextValueOr[bool],
        rhs: ContextValueOr[bool]
) -> ContextValue[bool]:
    """Check that lhs == rhs."""
    return lambda context: \
        from_context(context, lhs) == from_context(context, rhs)


def not_equals(
        lhs: ContextValueOr[bool],
        rhs: ContextValueOr[bool]
) -> ContextValue[bool]:
    """Check that lhs != rhs."""
    return lambda context: \
        from_context(context, lhs) != from_context(context, rhs)


def less_than(
        lhs: ContextValueOr[bool],
        rhs: ContextValueOr[bool]
) -> ContextValue[bool]:
    """Check that lhs < rhs."""
    return lambda context: \
        from_context(context, lhs) < from_context(context, rhs)


def greater_than(
        lhs: ContextValueOr[bool],
        rhs: ContextValueOr[bool]
) -> ContextValue[bool]:
    """Check that lhs > rhs."""
    return lambda context: \
        from_context(context, lhs) > from_context(context, rhs)


def greater_or_equals(
        lhs: ContextValueOr[bool],
        rhs: ContextValueOr[bool]
) -> ContextValue[bool]:
    """Check that lhs >= rhs."""
    return lambda context: \
        from_context(context, lhs) >= from_context(context, rhs)


def less_or_equals(
        lhs: ContextValueOr[bool],
        rhs: ContextValueOr[bool]
) -> ContextValue[bool]:
    """Check that lhs <= rhs."""
    return lambda context: \
        from_context(context, lhs) <= from_context(context, rhs)
