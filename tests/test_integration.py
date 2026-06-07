"""Tests for path bookkeeping helpers and controller edge behavior."""

from __future__ import annotations

import numpy as np
import pytest

from autopilot import MPC
from autopilot.utils import (
    compute_path_arc_lengths,
    compute_path_from_wp,
    update_path_obstacles,
)


def test_arc_length_monotonic_and_total():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.1)
    cdist, total = compute_path_arc_lengths(path)
    # Cumulative distance starts at 0 and is non-decreasing.
    assert cdist[0] == pytest.approx(0.0)
    assert np.all(np.diff(cdist) >= -1e-9)
    # Total length of a ~straight 10 m path should be close to 10.
    assert total == pytest.approx(10.0, abs=0.1)
    assert cdist[-1] == pytest.approx(total)


def test_update_path_obstacles_static_stays_put():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.05)
    obstacles = [{"distance": 2.0, "speed": 0.0, "radius": 0.3}]
    first = update_path_obstacles(obstacles, path, dt=0.1)
    second = update_path_obstacles(obstacles, path, dt=0.1)
    # A static obstacle (speed 0) should not move between updates.
    assert first[0][0] == pytest.approx(second[0][0])
    assert first[0][1] == pytest.approx(second[0][1])
    # Velocity components should be zero.
    assert first[0][3] == pytest.approx(0.0)
    assert first[0][4] == pytest.approx(0.0)


def test_update_path_obstacles_moving_advances():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.05)
    obstacles = [{"distance": 2.0, "speed": 0.5, "radius": 0.3}]
    start = obstacles[0]["distance"]
    update_path_obstacles(obstacles, path, dt=1.0)
    # The obstacle's path-distance should advance by speed * dt.
    assert obstacles[0]["distance"] == pytest.approx(start + 0.5, abs=1e-6)


def test_update_path_obstacles_returns_five_tuple():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 1, 1, 0], step=0.05)
    obstacles = [{"distance": 1.0, "speed": 0.2, "radius": 0.4}]
    result = update_path_obstacles(obstacles, path, dt=0.1)
    assert len(result[0]) == 5  # x, y, radius, vx, vy
    assert result[0][2] == pytest.approx(0.4)  # radius preserved


def test_emergency_braking_on_infeasible(mpc, reference):
    """An impossible obstacle (engulfing the vehicle) triggers safe braking."""
    initial = np.array([0.0, 0.0, 1.0, 0.0])
    # Obstacle centered on the vehicle with a huge radius -> infeasible avoid.
    obstacle = (0.0, 0.0, 50.0, 0.0, 0.0)
    x_opt, u_opt = mpc.solve(initial, reference, obstacle=obstacle)
    # Controller must still return a control command (braking), never crash.
    assert u_opt is not None
    assert u_opt.shape == (2, mpc.control_horizon)
    assert np.all(np.isfinite(u_opt))


def test_fresh_controllers_agree(straight_path):
    """Two fresh controllers given identical first inputs agree on the command.

    The controller is stateful (it warm-starts from the previous solve), so
    determinism only holds for the *first* solve of a fresh instance.
    """
    from autopilot.resources import MPC_CONFIG
    from autopilot.utils import get_ref_trajectory

    initial = np.array([0.0, 0.0, 0.0, 0.0])

    def first_command() -> np.ndarray:
        m = MPC(MPC_CONFIG, horizon_time=2.0)
        ref = get_ref_trajectory(
            initial, straight_path, 1.0, m.control_horizon * m.dt, m.dt
        )
        _, u = m.solve(initial, ref, obstacle=None)
        return u[:, 0]

    assert np.allclose(first_command(), first_command(), atol=1e-6)
