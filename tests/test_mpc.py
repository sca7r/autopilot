"""Tests for the MPC controller: construction, validation, and solving."""

from __future__ import annotations

import numpy as np
import pytest

from autopilot import MPC
from autopilot.resources import MPC_CONFIG


def _base_config() -> dict:
    """A minimal valid config dict for constructing an MPC."""
    return {
        "model": {
            "vehicle": {
                "wheelbase": 0.3,
                "width": 0.16,
                "max_speed": 1.5,
                "max_acc": 1.0,
                "max_d_acc": 1.0,
                "max_steer": 0.38,
                "max_d_steer": 0.52,
            }
        },
        "controller": {
            "prediction": {"horizon_time": 2.0, "timestep": 0.2},
            "weights": {
                "state_cost": [10, 50, 30, 30],
                "final_state_cost": [10, 50, 30, 30],
                "input_cost": [10, 10],
                "input_rate_cost": [10, 10],
            },
            "obstacle": {"safety_margin": 0.15, "slack_penalty": 1e5},
        },
    }


def test_construct_from_bundled_yaml():
    mpc = MPC(MPC_CONFIG, horizon_time=2.0)
    assert mpc.control_horizon == int(2.0 / mpc.dt)
    assert mpc.q_matrix.shape == (4, 4)
    assert mpc.r_matrix.shape == (2, 2)


def test_construct_from_dict():
    mpc = MPC(_base_config())
    assert mpc.control_horizon == 10  # 2.0 / 0.2


def test_missing_config_file_raises():
    with pytest.raises(FileNotFoundError):
        MPC("definitely_not_a_real_config.yaml")


def test_invalid_state_cost_length_raises():
    cfg = _base_config()
    cfg["controller"]["weights"]["state_cost"] = [1, 2, 3]  # wrong length
    with pytest.raises(ValueError):
        MPC(cfg)


def test_invalid_input_cost_length_raises():
    cfg = _base_config()
    cfg["controller"]["weights"]["input_cost"] = [1, 2, 3]  # wrong length
    with pytest.raises(ValueError):
        MPC(cfg)


def test_override_weights_via_kwargs():
    mpc = MPC(
        _base_config(),
        state_cost=[1, 1, 1, 1],
        final_state_cost=[2, 2, 2, 2],
    )
    assert np.allclose(np.diag(mpc.q_matrix), [1, 1, 1, 1])
    assert np.allclose(np.diag(mpc.qf_matrix), [2, 2, 2, 2])


def test_solve_returns_correct_shapes(mpc, reference):
    initial = np.array([0.0, 0.0, 0.0, 0.0])
    x_opt, u_opt = mpc.solve(initial, reference, obstacle=None)
    assert x_opt is not None
    assert x_opt.shape == (4, mpc.control_horizon + 1)
    assert u_opt.shape == (2, mpc.control_horizon)


def test_solve_respects_control_limits(mpc, reference):
    initial = np.array([0.0, 0.0, 0.0, 0.0])
    _, u_opt = mpc.solve(initial, reference, obstacle=None)
    # Acceleration and steering must stay within configured bounds (small tol).
    assert np.all(np.abs(u_opt[0, :]) <= mpc.max_acc + 1e-3)
    assert np.all(np.abs(u_opt[1, :]) <= mpc.max_steer + 1e-3)


def test_solve_wrong_initial_state_dim_raises(mpc, reference):
    with pytest.raises(AssertionError):
        mpc.solve(np.array([0.0, 0.0, 0.0]), reference)  # 3 != 4


def test_solve_with_obstacle_runs(mpc, reference):
    initial = np.array([0.0, 0.0, 0.0, 0.0])
    obstacle = (2.0, 0.1, 0.4, 0.0, 0.0)  # x, y, r, vx, vy
    x_opt, u_opt = mpc.solve(initial, reference, obstacle=obstacle)
    assert u_opt.shape == (2, mpc.control_horizon)


def test_warm_start_consistency(mpc, reference):
    """A second solve from the same state should remain feasible."""
    initial = np.array([0.0, 0.0, 0.0, 0.0])
    mpc.solve(initial, reference, obstacle=None)
    x_opt, u_opt = mpc.solve(initial, reference, obstacle=None)
    assert x_opt is not None
    assert np.all(np.isfinite(u_opt))
