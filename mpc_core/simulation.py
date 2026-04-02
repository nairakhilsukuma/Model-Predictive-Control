from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt 
from scipy.integrate import solve_ivp
from mpc_core.helpers import MPCModelParams
from mpc_core.construct_controller import initial_state, nominal_flows

def quad_tank_odesys(
    t: float,
    state: np.ndarray,
    params: MPCModelParams,
    flows: np.ndarray | None = None,
) -> np.ndarray:
    qa, qb = nominal_flows(params) if flows is None else np.asarray(flows, dtype=float)
    h1, h2, h3, h4 = np.maximum(np.asarray(state, dtype=float), 0.0)

    dh1dt = (
        -params.a1 / params.A1 * np.sqrt(2.0 * params.g * h1)
        + params.a3 / params.A1 * np.sqrt(2.0 * params.g * h3)
        + params.gamma1 * qa / params.A1
    )
    dh2dt = (
        -params.a2 / params.A2 * np.sqrt(2.0 * params.g * h2)
        + params.a4 / params.A2 * np.sqrt(2.0 * params.g * h4)
        + params.gamma2 * qb / params.A2
    )
    dh3dt = (
        -params.a3 / params.A3 * np.sqrt(2.0 * params.g * h3)
        + (1.0 - params.gamma2) * qb / params.A3
    )
    dh4dt = (
        -params.a4 / params.A4 * np.sqrt(2.0 * params.g * h4)
        + (1.0 - params.gamma1) * qa / params.A4
    )

    return np.array([dh1dt, dh2dt, dh3dt, dh4dt], dtype=float)



def simulate(
    params: MPCModelParams,
    time_span: tuple[float, float] = (0.0, 300.0),
    initial_conditions: np.ndarray | None = None,
    flows: np.ndarray | None = None,
    max_step: float = 0.1,
    num_points: int | None = None,
    system: function = quad_tank_odesys,
    plot: bool = False,
):
    y0 = initial_state(params) if initial_conditions is None else np.asarray(initial_conditions, dtype=float)

    t_eval = None
    if num_points is not None:
        t_eval = np.linspace(time_span[0], time_span[1], num_points)

    sol =  solve_ivp(
        fun=lambda t, y: system(t, y, params, flows),
        t_span=time_span,
        y0=y0,
        max_step=max_step,
        t_eval=t_eval,
    )
    if plot:
    #Plotting the results
      plt.plot(sol.t, sol.y[0], label='$h_{1}$')
      plt.plot(sol.t, sol.y[1], label='$h_{2}$')
      plt.plot(sol.t, sol.y[2], label='$h_{3}$')
      plt.plot(sol.t, sol.y[3], label='$h_{4}$')
      plt.xlabel('Time (s)')
      plt.ylabel('Tank height (cm)')
      plt.legend()
      plt.show()
      print(sol.y[0][-1],sol.y[1][-1],sol.y[2][-1],sol.y[3][-1])

    return sol



