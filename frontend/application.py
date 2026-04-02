import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))
CONFIG_PATH = Path(__file__).resolve().parent.parent / "setup.yaml"

from mpc_core.setup_mpc import (
    build_lsq_dmc_engine,
    run_lsq_dmc_simulation
)

st.set_page_config(layout="wide")

st.title("DMC Controller Simulator (Quad Tank)")

st.sidebar.header("MPC Control Panel")

st.sidebar.subheader("Model Duration")
t_f  = st.sidebar.slider("Simulation Time (s)", 20.0, 2000.0, 60.0)

st.sidebar.subheader(" Output Setpoints")
y1_sp = st.sidebar.slider("y1 Setpoint", 5.0, 25.0, 8.0)
y2_sp = st.sidebar.slider("y2 Setpoint", 5.0, 25.0, 7.0)

st.sidebar.subheader(" Output Setpoint change times")
y1_sp_change_time = st.sidebar.slider("Setpoint Change Time (s)", 20.0, 2000.0, 60.0)
y2_sp_change_time = st.sidebar.slider("Setpoint Change Time (s)", 10.0, 2000.0, 60.0)

st.sidebar.subheader(" Aggressiveness Weights")

w1 = st.sidebar.slider("Aggressiveness y1", 0.0001, 10.0, 0.01)
w2 = st.sidebar.slider("Aggressiveness y2", 0.0001, 10.0, 0.01)

run_button = st.sidebar.button("Run Simulation")

y1_moves=[{"time": y1_sp_change_time, "value": y1_sp}]
y2_moves=[{"time": y2_sp_change_time, "value": y2_sp}]

# if run_button:
#     with st.spinner("Running DMC simulation..."):
engine = build_lsq_dmc_engine(
        simulation_time=t_f,
        y1_moves=y1_moves,
        y2_moves=y2_moves,
        w1=w1,
        w2=w2,
)
result = run_lsq_dmc_simulation(
    engine,
)

df = result["df"]



st.subheader("Outputs vs Setpoints")

fig1, ax1 = plt.subplots()

ax1.plot(df["time_s"], df["y1"], label="y1")
ax1.plot(df["time_s"], df["sp_y1"], "--", color="blue", label="SP y1")

ax1.plot(df["time_s"], df["y2"], label="y2")
ax1.plot(df["time_s"], df["sp_y2"], "--", color="orange", label="SP y2")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Outputs")
ax1.legend()
ax1.grid()

st.pyplot(fig1)


st.subheader("Control Inputs")

fig2, ax2 = plt.subplots()

ax2.plot(df["time_s"], df["u1"], color="red", label="u1")
ax2.plot(df["time_s"], df["u2"], color="green", label="u2")

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Inputs")
ax2.legend()
ax2.grid()

st.pyplot(fig2)



st.subheader("Performance")

col1, col2 = st.columns(2)

col1.metric("MSE y1", f"{result['mse1']:.4f}")
col2.metric("MSE y2", f"{result['mse2']:.4f}")