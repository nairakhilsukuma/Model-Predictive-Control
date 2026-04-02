from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal
import numpy as np
from requests import options
import yaml
from scipy.linalg import expm
from mpc_core.helpers import load_model_params , MPCModelParams, MPCParams


def initial_state(params: MPCModelParams) -> np.ndarray:
    return np.array(
        [params.h1_0, params.h2_0, params.h3_0, params.h4_0],
        dtype=float,
    )


def nominal_flows(params: MPCModelParams) -> np.ndarray:
    return np.array([params.qa, params.qb], dtype=float)


def time_constants(params: MPCModelParams) -> np.ndarray:
    return np.array(
        [
            params.A1 / params.a1 * np.sqrt(2.0 * params.h1_0 / params.g),
            params.A2 / params.a2 * np.sqrt(2.0 * params.h2_0 / params.g),
            params.A3 / params.a3 * np.sqrt(2.0 * params.h3_0 / params.g),
            params.A4 / params.a4 * np.sqrt(2.0 * params.h4_0 / params.g),
        ],
        dtype=float,
    )


def continuous_state_space(
    params: MPCModelParams,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    T1, T2, T3, T4 = time_constants(params)

    A = np.array(
        [
            [-1.0 / T1, 0.0, params.A3 / (params.A1 * T3), 0.0],
            [0.0, -1.0 / T2, 0.0, params.A4 / (params.A2 * T4)],
            [0.0, 0.0, -1.0 / T3, 0.0],
            [0.0, 0.0, 0.0, -1.0 / T4],
        ],
        dtype=float,
    )

    # Tank 2 input denominator should be A2 to match the nonlinear model.
    B = np.array(
        [
            [params.gamma1 / params.A1, 0.0],
            [0.0, params.gamma2 / params.A2],
            [0.0, (1.0 - params.gamma2) / params.A3],
            [(1.0 - params.gamma1) / params.A4, 0.0],
        ],
        dtype=float,
    )

    C = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    D = np.zeros((2, 2), dtype=float)
    return A, B, C, D


def discrete_state_space(
    A: np.ndarray,
    B: np.ndarray,
    delta_t: float,
    options: Literal["exact", "zoh"] = "zoh",
) -> tuple[np.ndarray, np.ndarray]:
    if options == "zoh":
        n_states = A.shape[0]
        n_inputs = B.shape[1]

        augmented = np.zeros((n_states + n_inputs, n_states + n_inputs), dtype=float)
        augmented[:n_states, :n_states] = A
        augmented[:n_states, n_states:] = B

        disc = expm(augmented * delta_t)
        Phi = disc[:n_states, :n_states]
        Gamma = disc[:n_states, n_states:]

    if options == "exact":
        Phi = expm(A * delta_t)
        Gamma = np.linalg.solve(A, (Phi - np.eye(A.shape[0])) @ B)
    return Phi, Gamma