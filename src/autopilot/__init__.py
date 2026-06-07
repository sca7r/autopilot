"""autopilot: an iterative MPC trajectory-tracking controller built on CVXPY."""

from __future__ import annotations

from .controller import MPC

__all__ = ["MPC", "__version__"]
__version__ = "0.2.0"
