"""Small logging helpers shared across the package.

Library code never configures the root logger; it only obtains named
loggers via :func:`get_logger`. Applications (the demos, or a downstream
user) opt in to a sensible default handler by calling
:func:`configure_logging` once at start-up.
"""

from __future__ import annotations

import logging
import os

_DEFAULT_FORMAT = "%(asctime)s %(levelname)-8s [%(name)s] %(message)s"
_DEFAULT_DATEFMT = "%H:%M:%S"


def get_logger(name: str) -> logging.Logger:
    """Return a package-scoped logger.

    Args:
        name: Usually ``__name__`` of the calling module.

    Returns:
        A :class:`logging.Logger` namespaced under ``autopilot``.
    """
    return logging.getLogger(name)


def configure_logging(level: int | str | None = None) -> None:
    """Attach a stream handler to the package logger (idempotent).

    This is intended to be called by applications, not by library code.
    The level can be overridden with the ``MPC_LOG_LEVEL`` environment
    variable (e.g. ``MPC_LOG_LEVEL=DEBUG``).

    Args:
        level: Logging level as an int or name. Defaults to the
            ``MPC_LOG_LEVEL`` env var, or ``INFO`` if unset.
    """
    root = logging.getLogger("autopilot")
    if root.handlers:  # already configured
        return

    if level is None:
        level = os.environ.get("MPC_LOG_LEVEL", "INFO")

    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(_DEFAULT_FORMAT, _DEFAULT_DATEFMT))
    root.addHandler(handler)
    root.setLevel(level)
    root.propagate = False
