#!/usr/bin/env python

"""Module definig Invoke launch utils."""

from collections.abc import (
    Callable,
)
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Protocol,
    Text,
    TypeAlias,
    TypeVar,
    Union,
    runtime_checkable,
)

from launch import (
    Action,
    LaunchContext,
    LaunchDescriptionEntity,
    Substitution,
)

T = TypeVar('T')
U = TypeVar('U')


# Define the FunctionSubstitution traits (callable on LaunchContext)
@runtime_checkable
class FunctionSubstitution(Protocol[T]):
    def __call__[T](self, context: LaunchContext) -> T: ...


Substituable: TypeAlias = Union[
    Substitution,
    FunctionSubstitution[T],
]

MaybeSubstituable: TypeAlias = Union[Substituable[T], T]


def substitute(
        context: LaunchContext,
        value: MaybeSubstituable[T]
) -> T:
    """Perform the substitution of the given Substituable, if relevant."""
    if isinstance(value, Substitution):
        return value.perform(context)
    elif isinstance(value, FunctionSubstitution):
        return value(context)
    else:
        return value


class Invoke[T](Action):
    """Invoke the given function with arguments evaluated when needed.

    This is meant to be used as replacement of OpaqueFunction.

    Any arguments provided that are **not Substituable** will be directly
    forwarded to the function.
    Substituable arguments (launch.Substitution or FunctionSubstitution) will
    first be evaluated, given a LaunchContext, and then their values will be
    forwarded to the function call.

    You can obtain the Invoke result through the __call__(LaunchContext) -> T
    interface.
    When executing `.execute()` from the launch.Action API, this effectively
    calls __call__ but filter out the result T if it's not a
    LaunchDescriptionEntity.

    Additionnally, Invoke provide a 'Chainable-like' interface (`.and_then()`),
    which enable users to call an other function, with the result of the
    previously function call.
    By default, the result of the previous function call will be forwarded as
    the first unnamed argument to the new function, unless the
    'forward_previous_result_as' is set to a function keyword

    Examples:
    - Invoke(foo, 'a', 1).and_then(bar, 'b', 3):
      -> bar(foo('a', 1), 'b', 3)
    - Invoke(foo, 1, toto='a').and_then(bar, 'b', titi=3):
      -> bar(foo(1, toto='a'), 'b', titi=3)
    - Invoke(foo).and_then_with_key('s', bar, 1):
      -> bar(1, s=foo())
    """

    def __init__(
            self,
            f: Callable[..., T],
            *args: MaybeSubstituable[Any],
            **kwargs: MaybeSubstituable[Any],
    ) -> None:
        """Construct the Invoke action from the given function/args.

        Parameters
        ----------
        f: Callable[..., T]
          Fun called later on, with arguments evaluated from a LaunchContext
        args: MaybeSubstituable[Any]
          Arguments forwarded to the function call, after evaluation
        kwargs: MaybeSubstituable[Any]
          Keywords arguments forwarded to the function call, after evaluation
        """
        super().__init__()
        self.__f = f
        self.__args = args
        self.__kwargs = kwargs

    def and_then(
            self,
            f: Callable[[...], U],
            *args: MaybeSubstituable[Any],
            **kwargs: MaybeSubstituable[Any],
    ) -> 'Invoke[U]':
        """Call a new function f with previous results forwared as 1rst arg.

        Parameters
        ----------
        f: Callable[..., T]
          Fun called later on, with arguments evaluated from a LaunchContext
        args: MaybeSubstituable[Any]
          Arguments forwarded to the function call, after evaluation
        kwargs: MaybeSubstituable[Any]
          Keywords arguments forwarded to the function call, after evaluation

        Returns
        -------
        Invoke[U]
          A new Invoke instance containing the previous Invoke call as function
          arguments
        """
        return Invoke(
            f,
            self,
            *args,
            **kwargs,
        )

    def and_then_with_key(
            self,
            key: Text,
            f: Callable[[...], U],
            *args: MaybeSubstituable[Any],
            **kwargs: MaybeSubstituable[Any],
    ) -> 'Invoke[U]':
        """Call a new function f with previous call result forwared with a key.

        Parameters
        ----------
        key: Text
          Key used to forward the previous result to in the next function call
        f: Callable[..., T]
          Fun called later on, with arguments evaluated from a LaunchContext
        args: MaybeSubstituable[Any]
          Arguments forwarded to the function call, after evaluation
        kwargs: MaybeSubstituable[Any]
          Keywords arguments forwarded to the function call, after evaluation

        Returns
        -------
        Invoke[U]
          A new Invoke instance containing the previous Invoke call as function
          arguments
        """
        kwargs[key] = self
        return Invoke[U](
            f,
            *args,
            **kwargs,
        )

    def describe_args(self, *, separator: Text = ', ') -> Text:
        """Create a repr string of the arguments only."""
        args_repr = separator.join(
            (
                repr(arg)
                for arg in self.__args
            )
        )

        args_repr += separator.join(
            (
                '{k}={v}'.format(k=k, v=repr(v))
                for k, v in self.__kwargs.items()
            )
        )

        return args_repr

    def describe(
            self,
            *other_args: Any,
            fmt: Text = '{f}({args})',
            **other_kwargs: Any
    ) -> Text:
        """Create a repr of the given invoke, using the provided fmt string.

        Parameters
        ----------
        fmt: Text
          Format string to use (default to '{f}({args})').
          The following named format identifier are used:
          - f: Correspond to the repr of the function;
          - args: Correspond to the output of describe_args()
        other_args: Any
          Arguments forwarded to describe_args()
        other_kwargs: Any
          Keyword arguments forwarded to describe_args()

        Returns
        -------
        Text:
          A describe of the Invoke action
        """
        return fmt.format(
            f=repr(self.__f),
            args=self.describe_args(*other_args, *other_kwargs),
        )

    def __repr__(self) -> Text:
        """Return describe()."""
        return self.describe()

    def execute(
            self,
            context: LaunchContext
    ) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action following launch.Action API.

        Note
        ----
        Same a calling __call__ except that the output is filtered to match the
        expected Action API

        Parameters
        ----------
        context: LaunchContext
          Launch context use to perform substitution, if relevant

        Returns
        -------
        Optional[List[LaunchDescriptionEntity]]
          The result of calling the action, with arguments evaluated
        """
        r = self.__call__(context)

        if isinstance(r, LaunchDescriptionEntity):
            return [r]
        elif isinstance(r, list) and all(
                isinstance(item, LaunchDescriptionEntity)
                for item in r
        ):
            return r
        else:
            return None

    def __call__(self, context: LaunchContext) -> T:
        """Perform the postpone args evaluation and function call.

        Parameters
        ----------
        context: LaunchContext
          Launch context use to perform substitution, if relevant

        Returns
        -------
        T
          The result of calling the function, with arguments evaluated
        """
        return self.__f(
            *[substitute(context, arg) for arg in self.__args],
            **{k: substitute(context, v) for k, v in self.__kwargs.items()}
        )


def evaluate(
        arg: MaybeSubstituable[Any],
        *others: MaybeSubstituable[Any],
) -> Invoke[Union[Any, List[Any]]]:
    """Create an Invoke instance that only evaluate un-named args.

    Note
    ----
    If only one arg is provided, the result will be this arg
    evaluated. Otherwise, the result will be a List of args.

    Parameters
    ----------
    arg: MaybeSubstituable[Any]
      A single arg to evaluate
    others: MaybeSubstituable[Any]
      Any other arguments to evaluate

    Returns
    -------
    Invoke[Union[Any, List[Any]]]
      An Invoke instance that will return the arguments evaluated
    """
    return Invoke(
        lambda arg, *others:
        arg if len(others) == 0 else [arg] + others
    )


def evaluate_as_dict(
        **kwargs: MaybeSubstituable[Any],
) -> Invoke[Dict[Text, Any]]:
    """Create an Invoke instance that only evaluate un-named KEYWORD args.

    Parameters
    ----------
    kwargs: MaybeSubstituable[Any]
      Arguments map with value to evaluate

    Returns
    -------
    Invoke[Dict[Text, Any]]
      An Invoke instance that will return the input dict with all its values
      evaluated
    """
    return Invoke(
        lambda **kwargs: kwargs,
        **kwargs,
    )
