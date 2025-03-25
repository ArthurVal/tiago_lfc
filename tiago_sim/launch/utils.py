#!/usr/bin/env python

"""Add utils function."""

from typing import (
    Any,
    Dict,
    Text,
)


def dict_to_string(
        value: Dict[Any, Any],
        *,
        items_separator: Text = '\n',
        kv_separator: Text = ': ',
        kv_header: Text = '',
        kv_footer: Text = '',
) -> Text:
    """Create a simple string version of a dictionnary."""
    return items_separator.join(
        (
            '{header}{k}{sep}{v}{footer}'.format(
                header=kv_header,
                k=k,
                sep=kv_separator,
                v=v,
                footer=kv_footer,
            )
            for k, v in value.items()
        )
    )
