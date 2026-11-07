from types import SimpleNamespace
import importlib

import numpy as np
import pytest

from bluerov2 import parameters
from bluerov2.lbts import monte_carlo_path


def result_with_covariances(total, estimation_error):
    p = parameters()
    return {
        "success": True, "nominalTrajectory": np.zeros((12, 2)),
        "controlTrajectory": np.zeros((6, 1)), "time": np.array([0.0, .01]),
        "model": p.model, "world": p.world,
        "feedbackGains": [np.zeros((6, 12))],
        "kalmanGains": [np.zeros((12, 12))],
        "measurementMatrices": [np.eye(12)],
        "measurementCovariances": [np.zeros((12, 12))],
        "options": SimpleNamespace(process_noise_qc=np.zeros((12, 12))),
        "covarianceSequence": [total],
        "estimationCovarianceSequence": [estimation_error],
    }


def test_zero_estimation_error_means_true_and_estimated_initial_states_match():
    result = result_with_covariances(.001*np.eye(12), np.zeros((12, 12)))
    mc = monte_carlo_path(result, {"num_runs": 8, "verbose": False})
    np.testing.assert_array_equal(mc["true_trajectories"][:, 0], mc["estimated_trajectories"][:, 0])
    assert np.any(mc["estimated_trajectories"][:, 0] != 0)


def test_zero_estimate_covariance_keeps_initial_estimate_at_nominal():
    covariance = .001*np.eye(12)
    result = result_with_covariances(covariance, covariance)
    mc = monte_carlo_path(result, {"num_runs": 8, "verbose": False})
    np.testing.assert_array_equal(mc["estimated_trajectories"][:, 0], np.zeros((12, 8)))
    assert np.any(mc["true_trajectories"][:, 0] != 0)


def test_initial_noise_uses_matlab_covariance_definitions():
    result = result_with_covariances(.009*np.eye(12), .004*np.eye(12))
    runs, seed = 8, 17
    rng = np.random.default_rng(seed)
    estimated = np.sqrt(.005)*rng.standard_normal((12, runs))
    true = estimated + np.sqrt(.004)*rng.standard_normal((12, runs))
    mc = monte_carlo_path(result, {"num_runs": runs, "rng_seed": seed, "verbose": False})
    np.testing.assert_allclose(mc["estimated_trajectories"][:, 0], estimated, atol=1e-14)
    np.testing.assert_allclose(mc["true_trajectories"][:, 0], true, atol=1e-14)


def test_numerical_divergence_is_not_reported_as_physical_collision(monkeypatch):
    monkeypatch.setattr(importlib.import_module("bluerov2.lbts"), "rk4_step",
                        lambda *args: np.full(12, np.nan))
    result = result_with_covariances(.001*np.eye(12), .0006*np.eye(12))
    with pytest.raises(FloatingPointError, match="finer plan time grid"):
        monte_carlo_path(result, {"num_runs": 2, "verbose": False})
