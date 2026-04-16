from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp

from .params import VehicleParams
from .plant import build_disturbance_plant, nonlinear_rhs


def _require_control():
    try:
        import control as control_module
    except ImportError as exc:
        raise RuntimeError(
            "python-control is required for linear simulations. "
            "Install it with `pip install control`."
        ) from exc
    return control_module


@dataclass(slots=True)
class SimResult:
    t: np.ndarray
    v: np.ndarray
    u: np.ndarray
    v_ref: np.ndarray
    e: np.ndarray
    label: str = ""
    model: str = ""
    scenario: str = ""
    controller_key: str = ""
    metadata: dict[str, float | str] = field(default_factory=dict)


def run_linear(
    *,
    vp: VehicleParams,
    plant_tf,
    controller_tf,
    t_span: tuple[float, float],
    v_ref_func,
    dt: float = 0.1,
    label: str = "",
    scenario: str = "",
    controller_key: str = "",
    theta_func=None,
):
    control_module = _require_control()
    t = np.arange(t_span[0], t_span[1] + dt, dt)
    r_dev = np.array([float(v_ref_func(ti)) - vp.v0 for ti in t], dtype=float)

    loop_tf = controller_tf * plant_tf
    closed_loop_ref = control_module.feedback(loop_tf)
    response_ref = control_module.forced_response(closed_loop_ref, T=t, U=r_dev)
    v_dev = np.asarray(response_ref.outputs, dtype=float)

    if theta_func is not None:
        theta = np.array([float(theta_func(ti)) for ti in t], dtype=float)
        unity = control_module.tf([1.0], [1.0])
        disturbance_tf = build_disturbance_plant(vp) * control_module.feedback(unity, loop_tf)
        response_dist = control_module.forced_response(disturbance_tf, T=t, U=theta)
        v_dev = v_dev + np.asarray(response_dist.outputs, dtype=float)

    error_dev = r_dev - v_dev
    u_dev_resp = control_module.forced_response(controller_tf, T=t, U=error_dev)
    u_dev = np.asarray(u_dev_resp.outputs, dtype=float)

    v = v_dev + vp.v0
    v_ref = r_dev + vp.v0
    u = u_dev + vp.u0

    return SimResult(
        t=t,
        v=v,
        u=u,
        v_ref=v_ref,
        e=v_ref - v,
        label=label,
        model="linear",
        scenario=scenario,
        controller_key=controller_key,
    )


def run_nonlinear(
    *,
    vp: VehicleParams,
    controller,
    t_span: tuple[float, float],
    v_ref_func,
    theta_func,
    dt: float = 0.1,
    label: str = "",
    scenario: str = "",
    controller_key: str = "",
):
    t_eval = np.arange(t_span[0], t_span[1] + dt, dt)
    controller_initial_state = controller.initial_state()
    y0 = np.concatenate(([vp.v0], controller_initial_state))

    sol = solve_ivp(
        fun=nonlinear_rhs,
        t_span=t_span,
        y0=y0,
        method="RK45",
        t_eval=t_eval,
        max_step=min(0.1, dt),
        args=(vp, controller, v_ref_func, theta_func),
    )

    if not sol.success:
        raise RuntimeError(f"ODE solver failed: {sol.message}")

    v = np.asarray(sol.y[0], dtype=float)
    controller_state = np.asarray(sol.y[1:], dtype=float)
    v_ref = np.array([float(v_ref_func(ti)) for ti in sol.t], dtype=float)
    u = np.zeros_like(sol.t, dtype=float)

    for idx, ti in enumerate(sol.t):
        state_slice = controller_state[:, idx] if controller_state.size else np.zeros(0)
        u[idx], _ = controller.compute(
            error=float(v_ref[idx] - v[idx]),
            state=np.asarray(state_slice, dtype=float),
            measurement=float(v[idx]),
            v_ref=float(v_ref[idx]),
            t=float(ti),
        )

    return SimResult(
        t=np.asarray(sol.t, dtype=float),
        v=v,
        u=u,
        v_ref=v_ref,
        e=v_ref - v,
        label=label,
        model="nonlinear",
        scenario=scenario,
        controller_key=controller_key,
    )
