"""Shared pytest fixtures for the autopilot test suite."""

from __future__ import annotations

import numpy as np
import pytest

from autopilot import MPC
from autopilot.resources import MPC_CONFIG
from autopilot.utils import compute_path_from_wp, get_ref_trajectory


@pytest.fixture(scope="session")
def straight_path() -> np.ndarray:
    """A simple gently curving path as a (3, N) array."""
    return compute_path_from_wp([0, 3, 6, 10], [0, 0, 2, 4], step=0.05)


@pytest.fixture(scope="session")
def mpc() -> MPC:
    """A controller built from the bundled config, short horizon for speed."""
    return MPC(MPC_CONFIG, horizon_time=2.0)


@pytest.fixture
def reference(mpc: MPC, straight_path: np.ndarray) -> np.ndarray:
    """A reference trajectory for the controller horizon."""
    state = np.array([0.0, 0.0, 0.0, 0.0])
    return get_ref_trajectory(
        state,
        straight_path,
        target_v=1.0,
        T=mpc.control_horizon * mpc.dt,
        DT=mpc.dt,
    )
