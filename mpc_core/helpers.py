from __future__ import annotations
from pathlib import Path
import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import numpy as np
import yaml
from scipy.linalg import expm
import streamlit as st

# Utility helpers
def load_setup_config(config_path: str | Path | None = None) -> dict:
    path = Path(config_path) if config_path else project_root() / "setup.yaml"

    if not path.exists():
        raise FileNotFoundError(f"Could not find config file: {path}")

    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if config is None:
        raise ValueError(f"YAML file is empty or invalid: {path}")

    return config
    

def load_model_params(
    config_path: str | Path | None = None,
    section: str = "mpc_model"
):

    if config_path is None:
        raise ValueError("config_path must be provided")
    path = Path(config_path) if config_path else project_root() / "setup.yaml"
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    config = load_setup_config(path)

    if section:
        if section not in config:
            raise KeyError(f"Section '{section}' was not found in {path}")
        
    # Extracting as required
    data = config[section]
    if not isinstance(data, dict):
        raise ValueError(
        f"Section '{section}' is invalid. Expected dict, got: {type(data)} → {data}"
        )


    if section == "mpc_model":
        return MPCModelParams(**data)

    elif section == "mpc_parameters":
        return MPCParams(**data)


    
def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


@dataclass
class MPCModelParams:

    a1: float
    a2: float
    a3: float
    a4: float

    A1: float
    A2: float
    A3: float
    A4: float

    h1_0: float
    h2_0: float
    h3_0: float
    h4_0: float

    k1: float
    k2: float
    g: float

    gamma1: float
    gamma2: float

    v1: float
    v2: float

    @property
    def qa(self) -> float:
        return self.v1 * self.k1

    @property
    def qb(self) -> float:
        return self.v2 * self.k2
    

@dataclass
class MPCParams:
    P: int
    M: int
    N: int
    t_f: float
    delta_t: float
    w1: float
    w2: float
    setpoint_y1: float
    setpoint_y2: float
    umin: float
    umax: float
    plant_max_step: float