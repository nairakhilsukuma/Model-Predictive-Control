from __future__ import annotations
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.optimize import minimize
from scipy.linalg import expm
from numpy.linalg import multi_dot
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.signal import dstep
import cvxpy as cp
from mpc_core.helpers import load_model_params, MPCModelParams, MPCParams
from mpc_core.construct_controller import initial_state, nominal_flows, continuous_state_space, discrete_state_space  
from mpc_core.simulation import quad_tank_odesys, simulate

# USER SETTINGS
DEFAULT_DELTA_T = 0.1
DEFAULT_SIMULATION_TIME = 300.0
DEFAULT_MAX_STEP = 0.1
DEFAULT_MODEL_DURATION = 5000
CONFIG_PATH = Path(__file__).resolve().parent.parent / "setup.yaml"

def setup_odesim(
    config_path: str | None = None,
    simulation_time: float = DEFAULT_SIMULATION_TIME,
    max_step: float = DEFAULT_MAX_STEP,
):
    params = load_model_params(config_path)
    mpc_params = load_model_params(config_path, section="mpc_parameters")   
    delta_t = mpc_params.delta_t
    x0 = initial_state(params)
    u0 = nominal_flows(params)

    nominal_residual = quad_tank_odesys(0.0, x0, params, u0)


    num_points = int(round(simulation_time / delta_t)) + 1
    solution = simulate(
        params=params,
        time_span=(0.0, simulation_time),
        initial_conditions=x0,
        flows=u0,
        max_step=max_step,
        num_points=num_points,
        system = quad_tank_odesys,
        plot = False,
    )

    if not solution.success:
        raise RuntimeError(f"Simulation failed: {solution.message}")

    return {
        "params": params,
        "x0": x0,
        "u0": u0,
        "nominal_residual": nominal_residual,
        "simulation": solution,
    }

def setup_statematrices(
    config_path: str | None = None,
):
    params = load_model_params(config_path)
    mpc_params = load_model_params(config_path, section="mpc_parameters")
    delta_t = mpc_params.delta_t

    A, B, C, D = continuous_state_space(params)
    Phi, Gamma = discrete_state_space(A, B, delta_t)

    return {
        "A": A,
        "B": B,
        "C": C,
        "D": D,
        "Phi": Phi,
        "Gamma": Gamma,
    }

def setup_step_response(
    config_path: str | None = None,
    plot: bool = False,):

    mpc_params = load_model_params(config_path, section="mpc_parameters")
    N = mpc_params.N
    delta_t = mpc_params.delta_t

    matrices = setup_statematrices(config_path)
    sysd = (matrices["Phi"], matrices["Gamma"], matrices["C"], matrices["D"], delta_t)

    #Step response
    t_step, y_step = dstep(sysd, n=N+1) # This will give a (2x51x2) shape tuple

    #To convert the tuple into an array
    y_step_ar = np.asarray(y_step, dtype=float) 

    y1_u1 = y_step_ar[0][:,0]
    y2_u1 = y_step_ar[0][:,1]
    y1_u2 = y_step_ar[1][:,0]
    y2_u2 = y_step_ar[1][:,1]

    responses = {
        "t_step": np.asarray(t_step, dtype=float),
        "y1_u1": y_step_ar[0][:, 0],
        "y2_u1": y_step_ar[0][:, 1],
        "y1_u2": y_step_ar[1][:, 0],
        "y2_u2": y_step_ar[1][:, 1],
    }

    if plot:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

        axes[0, 0].step(t_step, y1_u1)
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel("Output y1 response to step in u$_1$")

        axes[0, 1].step(t_step, y1_u2)
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("Output y1 response to step in u$_2$")

        axes[1, 0].step(t_step, y2_u1)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Output y2 response to step in u$_1$")

        axes[1, 1].step(t_step, y2_u2)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Output y2 response to step in u$_2$")

        for ax in axes.flat:
            ax.set_xlabel("Time (s)")
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()
    else:
        fig = None

    return fig , responses

def build_lsq_dmc_engine( 
        config_path: str | Path = CONFIG_PATH,
        plot: bool = False,
        simulation_time=None,
        y1_moves=None,
        y2_moves=None,
        w1=None,
        w2=None,
    ):
    
    params = load_model_params(config_path, section="mpc_model")
    mpc_params = load_model_params(config_path, section="mpc_parameters")
    
    if simulation_time is not None:
        mpc_params.t_f = simulation_time

    if w1 is not None:
        mpc_params.w1 = w1

    if w2 is not None:
        mpc_params.w2 = w2

    x0 = initial_state(params)
    u0 = nominal_flows(params)

    #Matrices
    matrices = setup_statematrices(config_path)
    A = matrices["A"]
    B = matrices["B"]   
    C = matrices["C"]
    D = matrices["D"]
    Phi = matrices["Phi"]
    Gamma = matrices["Gamma"]

    # Step response
    _, step_response = setup_step_response(config_path)

    # Constructing the dynamic matrix for DMC
    N = mpc_params.N    
    P = mpc_params.P
    M = mpc_params.M
    w1 = mpc_params.w1
    w2 = mpc_params.w2

    y11 = step_response["y1_u1"]
    y12 = step_response["y1_u2"]
    y21 = step_response["y2_u1"]
    y22 = step_response["y2_u2"]
    t_step = step_response["t_step"]

    Sf11 = np.zeros((P, M), dtype=float)
    Sf12 = np.zeros((P, M), dtype=float)
    Sf21 = np.zeros((P, M), dtype=float)
    Sf22 = np.zeros((P, M), dtype=float)

    for i in range(M):
        Sf11[i:, i] = y11[1 : P + 1 - i]
        Sf12[i:, i] = y12[1 : P + 1 - i]
        Sf21[i:, i] = y21[1 : P + 1 - i]
        Sf22[i:, i] = y22[1 : P + 1 - i]

    Spast11 = np.zeros((P, N - 2), dtype=float)
    Spast12 = np.zeros((P, N - 2), dtype=float)
    Spast21 = np.zeros((P, N - 2), dtype=float)
    Spast22 = np.zeros((P, N - 2), dtype=float)

    for i in range(P):
        Spast11[i, 0 : N - 2 - i] = y11[2 + i : N]
        Spast12[i, 0 : N - 2 - i] = y12[2 + i : N]
        Spast21[i, 0 : N - 2 - i] = y21[2 + i : N]
        Spast22[i, 0 : N - 2 - i] = y22[2 + i : N]

    Sf = np.block(
        [
            [Sf11, Sf12],
            [Sf21, Sf22],
        ]
    )

    Spast = np.block(
        [
            [Spast11, Spast12],
            [Spast21, Spast22],
        ]
    )

    SN = np.array(
        [
            [y11[N], y12[N]],
            [y21[N], y22[N]],
        ],
        dtype=float,
    )

    W = np.diag([w1] * M + [w2] * M)


    return {
        "y1_moves": y1_moves or [],
        "y2_moves": y2_moves or [],
        "params": params,
        "mpc": mpc_params,
        "x0": x0,
        "u0": u0,
        "A": A,
        "B": B,
        "C": C,
        "D": D,
        "Phi": Phi,
        "Gamma": Gamma,
        "t_step": t_step,
        "y1_u1": y11,
        "y2_u1": y21,
        "y1_u2": y12,
        "y2_u2": y22,
        "Sf11": Sf11,
        "Sf12": Sf12,
        "Sf21": Sf21,
        "Sf22": Sf22,
        "Spast11": Spast11,
        "Spast12": Spast12,
        "Spast21": Spast21,
        "Spast22": Spast22,
        "Sf": Sf,
        "Spast": Spast,
        "SN": SN,
        "W": W,
    }

def setup_initial_states(
    engine: dict[str, object]
):
    params = engine["params"]
    mpc = engine["mpc"]
    n_steps = int(round(mpc.t_f / mpc.delta_t)) + 1
    u0 = np.asarray(engine["u0"], dtype=float)
    x0 = np.asarray(engine["x0"], dtype=float)

    # Initialize variables - generalize for the loop
    t = np.arange(n_steps) * mpc.delta_t #t = np.linspace(0.0, total_time, n_steps)
    x = np.zeros((4, n_steps), dtype=float)
    y = np.zeros((2, n_steps), dtype=float)
    y_hat = np.zeros((2, n_steps), dtype=float)
    u = np.zeros((2, n_steps), dtype=float)
    error = np.zeros((2, n_steps), dtype=float)
    d_hat = np.zeros((2, mpc.P, n_steps), dtype=float)

    x[:, 0] = x0
    y[:, 0] = x0[:2]
    y_hat[:, 0] = x0[:2]
    u[:, 0] = u0

    u1_past = np.full(mpc.N - 1, u0[0], dtype=float)
    u2_past = np.full(mpc.N - 1, u0[1], dtype=float)


    sp_y1 = build_setpoint_profile(
        n_steps,
        mpc.delta_t,
        mpc.setpoint_y1,
        engine.get("y1_moves", [])
    )

    sp_y2 = build_setpoint_profile(
        n_steps,
        mpc.delta_t,
        mpc.setpoint_y2,
        engine.get("y2_moves", [])
    )
    
    error[0, 0] = sp_y1[0] - y[0, 0]
    error[1, 0] = sp_y2[0] - y[1, 0]


    return {
        "t": t,
        "x": x,
        "y": y,
        "u": u,
        "u1_past": u1_past,
        "u2_past": u2_past,
        "d_hat": d_hat,
        "y_hat": y_hat,
        "error": error,
        "sp_y1": sp_y1,
        "sp_y2": sp_y2,
    }

def build_setpoint_profile(
    n_steps: int,
    delta_t: float,
    initial_value: float,
    moves: list[dict[str, float]] | None = None,
) -> np.ndarray:
    profile = np.full(n_steps, initial_value, dtype=float)

    for move in moves or []:
        move_time = float(move["time"])
        move_value = float(move["value"])
        move_index = int(round(move_time / delta_t))
        move_index = max(0, min(move_index, n_steps - 1))
        profile[move_index:] = move_value

    return profile


def horizon_reference(profile: np.ndarray, k: int, P: int) -> np.ndarray:
    window = profile[k + 1 : k + 1 + P]
    if window.size < P:
        window = np.pad(window, (0, P - window.size), constant_values=profile[-1])
    return window.reshape(P, 1)

def simulate_plant_step(
    state_k: np.ndarray,
    input_k: np.ndarray,
    params,
    delta_t: float,
    max_step: float = 0.01,
) -> np.ndarray:
    sol = solve_ivp(
        fun=lambda t, x: quad_tank_odesys(t, x, params, input_k),
        t_span=(0.0, delta_t),
        y0=state_k,
        t_eval=[delta_t],
        max_step=min(max_step, delta_t),
    )
    if not sol.success:
        raise RuntimeError(f"Plant integration failed: {sol.message}")
    return sol.y[:, -1]

def results_to_dataframe(result: dict[str, object]) -> pd.DataFrame:
    return pd.DataFrame(
        {
            "time_s": result["t"],
            "y1": result["y"][0],
            "y2": result["y"][1],
            "y1_hat": result["y_hat"][0],
            "y2_hat": result["y_hat"][1],
            "u1": result["u"][0],
            "u2": result["u"][1],
            "sp_y1": result["sp_y1"],
            "sp_y2": result["sp_y2"],
            "error1": result["error"][0],
            "error2": result["error"][1],
        }
    )

def run_lsq_dmc_simulation(
    engine: dict[str, object],
    input_limits: tuple[float, float] | None = None,
) -> dict[str, object]:
    params = engine["params"]
    mpc = engine["mpc"]
    C = np.asarray(engine["C"], dtype=float)
    D = np.asarray(engine["D"], dtype=float)

    sim = setup_initial_states(engine)

    P = mpc.P
    M = mpc.M
    n_steps = sim["x"].shape[1]

    umin = mpc.umin
    umax = mpc.umax
    if input_limits is not None:
        umin, umax = input_limits

    for k in range(n_steps - 1):
        delta_u1_past = sim["u1_past"][0 : mpc.N - 2] - sim["u1_past"][1 : mpc.N - 1]
        delta_u2_past = sim["u2_past"][0 : mpc.N - 2] - sim["u2_past"][1 : mpc.N - 1]
        delta_u_past = np.concatenate((delta_u1_past, delta_u2_past)).reshape(-1, 1)

        u1_p = np.flip(sim["u1_past"])[:P].reshape(P, 1)
        u2_p = np.flip(sim["u2_past"])[:P].reshape(P, 1)

        steady_term = np.vstack(
            (
                engine["SN"][0, 0] * u1_p + engine["SN"][0, 1] * u2_p,
                engine["SN"][1, 0] * u1_p + engine["SN"][1, 1] * u2_p,
            )
        )

        d_hat_k = np.vstack(
            (
                sim["d_hat"][0, :, k].reshape(P, 1),
                sim["d_hat"][1, :, k].reshape(P, 1),
            )
        )

        free_response = engine["Spast"] @ delta_u_past + steady_term + d_hat_k

        r1 = horizon_reference(sim["sp_y1"], k, P)
        r2 = horizon_reference(sim["sp_y2"], k, P)
        r = np.vstack((r1, r2))
        E = r - free_response

        delta_u_f = np.linalg.solve(
            engine["Sf"].T @ engine["Sf"] + engine["W"],
            engine["Sf"].T @ E,
        )

        delta_u1_f = delta_u_f[:M]
        delta_u2_f = delta_u_f[M:]

        u1_prev = engine["u0"][0] if k == 0 else sim["u"][0, k - 1]
        u2_prev = engine["u0"][1] if k == 0 else sim["u"][1, k - 1]

        sim["u"][0, k] = np.clip(u1_prev + float(delta_u1_f[0, 0]), umin, umax)
        sim["u"][1, k] = np.clip(u2_prev + float(delta_u2_f[0, 0]), umin, umax)

        u_k = sim["u"][:, k]

        sim["x"][:, k + 1] = simulate_plant_step(
            state_k=sim["x"][:, k],
            input_k=u_k,
            params=params,
            delta_t=mpc.delta_t,
            max_step=mpc.plant_max_step,
        )

        sim["y"][:, k + 1] = C @ sim["x"][:, k + 1] + D @ u_k

        y1_hat_next = (
            engine["Sf11"][0, :] @ delta_u1_f[:, 0]
            + engine["Sf12"][0, :] @ delta_u2_f[:, 0]
            + engine["Spast11"][0, :] @ delta_u1_past
            + engine["Spast12"][0, :] @ delta_u2_past
            + engine["SN"][0, 0] * float(u1_p[0, 0])
            + engine["SN"][0, 1] * float(u2_p[0, 0])
        )

        y2_hat_next = (
            engine["Sf21"][0, :] @ delta_u1_f[:, 0]
            + engine["Sf22"][0, :] @ delta_u2_f[:, 0]
            + engine["Spast21"][0, :] @ delta_u1_past
            + engine["Spast22"][0, :] @ delta_u2_past
            + engine["SN"][1, 0] * float(u1_p[0, 0])
            + engine["SN"][1, 1] * float(u2_p[0, 0])
        )

        sim["y_hat"][0, k + 1] = y1_hat_next
        sim["y_hat"][1, k + 1] = y2_hat_next

        sim["d_hat"][0, :, k + 1] = sim["y"][0, k + 1] - y1_hat_next
        sim["d_hat"][1, :, k + 1] = sim["y"][1, k + 1] - y2_hat_next

        sim["u1_past"] = np.hstack((sim["u"][0, k], sim["u1_past"][0 : mpc.N - 2]))
        sim["u2_past"] = np.hstack((sim["u"][1, k], sim["u2_past"][0 : mpc.N - 2]))

        sim["error"][0, k + 1] = sim["sp_y1"][k + 1] - sim["y"][0, k + 1]
        sim["error"][1, k + 1] = sim["sp_y2"][k + 1] - sim["y"][1, k + 1]

    if n_steps > 1:
        sim["u"][:, -1] = sim["u"][:, -2]

    result = {
        **engine,
        **sim,
        "mse1": float(np.mean(sim["error"][0] ** 2)),
        "mse2": float(np.mean(sim["error"][1] ** 2)),
    }
    result["df"] = results_to_dataframe(result)
    return result


def main() -> None:
    engine = build_lsq_dmc_engine()
    result = run_lsq_dmc_simulation(
        engine,
        y1_moves=[{"time": 20.0, "value": 8.0}],
        y2_moves=[{"time": 20.0, "value": 7.0}],
        simulation_time=1000,
    )

    print(f"MSE1 = {result['mse1']:.6f}")
    print(f"MSE2 = {result['mse2']:.6f}")
    print(result["df"].head())


if __name__ == "__main__":
    main()
