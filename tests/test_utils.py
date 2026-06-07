"""Unit tests for the geometry / path utilities in autopilot.utils."""

from __future__ import annotations

import numpy as np
import pytest

from autopilot.utils import (
    compute_errors,
    compute_path_from_wp,
    detect_obstacle_camera,
    ego_to_global,
    get_nn_idx,
    get_ref_trajectory,
)


def test_compute_path_shape_and_heading():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.1)
    assert path.shape[0] == 3
    assert path.shape[1] > 10
    # A perfectly horizontal path should have ~0 heading everywhere.
    assert np.allclose(path[2, 1:-1], 0.0, atol=1e-2)


def test_compute_path_spacing_is_uniform():
    path = compute_path_from_wp([0, 3, 6, 9], [0, 1, 1, 0], step=0.05)
    seg = np.hypot(np.diff(path[0]), np.diff(path[1]))
    # Arc-length resampling should keep segment lengths close to `step`.
    assert np.allclose(seg, 0.05, atol=1e-2)


def test_get_nn_idx_returns_forward_point():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.1)
    state = np.array([5.0, 0.0, 0.0, 0.0])
    idx = get_nn_idx(state, path)
    assert 0 <= idx < path.shape[1]
    # The closest point should be near x=5.
    assert abs(path[0, idx] - 5.0) < 0.2


def test_ego_to_global_identity_at_origin():
    state = np.array([0.0, 0.0, 0.0, 0.0])
    traj = np.array([[1.0, 2.0], [0.0, 0.0]])
    out = ego_to_global(state, traj)
    assert np.allclose(out, traj[:2, :])


def test_ego_to_global_translation_and_rotation():
    # Heading 90 deg, positioned at (1, 1): ego +x maps to global +y.
    state = np.array([1.0, 1.0, 0.0, np.pi / 2])
    traj = np.array([[1.0], [0.0]])  # one point, 1m ahead in ego frame
    out = ego_to_global(state, traj)
    assert np.allclose(out[:, 0], [1.0, 2.0], atol=1e-9)


def test_compute_errors_zero_on_path():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.1)
    # On the path, heading aligned -> both errors ~0.
    state = np.array([5.0, 0.0, 1.0, 0.0])
    cte, heading_err = compute_errors(state, path)
    assert abs(cte) < 1e-2
    assert abs(heading_err) < 1e-2


def test_compute_errors_sign_of_cross_track():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 0, 0, 0], step=0.1)
    left = np.array([5.0, 0.5, 1.0, 0.0])
    right = np.array([5.0, -0.5, 1.0, 0.0])
    cte_left, _ = compute_errors(left, path)
    cte_right, _ = compute_errors(right, path)
    # The two sides must have opposite sign.
    assert np.sign(cte_left) == -np.sign(cte_right)
    assert abs(cte_left) == pytest.approx(0.5, abs=1e-2)


def test_get_ref_trajectory_shape():
    path = compute_path_from_wp([0, 3, 6, 10], [0, 1, 2, 0], step=0.05)
    state = np.array([0.0, 0.0, 0.0, 0.0])
    T, DT = 2.0, 0.2
    ref = get_ref_trajectory(state, path, target_v=1.0, T=T, DT=DT)
    assert ref.shape == (4, int(T / DT) + 1)


def test_detect_obstacle_in_fov():
    # Obstacle directly ahead within range -> detected.
    obstacles = [(5.0, 0.0, 0.4, 0.0, 0.0)]
    found = detect_obstacle_camera(
        obstacles, robot_x=0.0, robot_y=0.0, robot_heading=0.0, max_range=10.0
    )
    assert found is not None


def test_detect_obstacle_behind_not_detected():
    # Obstacle directly behind, narrow FOV -> not detected.
    obstacles = [(-5.0, 0.0, 0.4, 0.0, 0.0)]
    found = detect_obstacle_camera(
        obstacles,
        robot_x=0.0,
        robot_y=0.0,
        robot_heading=0.0,
        max_range=10.0,
        fov_degrees=60.0,
    )
    assert found is None


def test_detect_obstacle_out_of_range():
    obstacles = [(50.0, 0.0, 0.4, 0.0, 0.0)]
    found = detect_obstacle_camera(
        obstacles, robot_x=0.0, robot_y=0.0, robot_heading=0.0, max_range=10.0
    )
    assert found is None
