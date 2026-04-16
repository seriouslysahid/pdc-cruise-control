from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import numpy as np

from .params import VehicleParams

if TYPE_CHECKING:
    import control as ct
else:
    ct = None


def _require_control():
    try:
        import control as control_module
    except ImportError as exc:
        raise RuntimeError(
            "python-control is required for linear simulations. "
            "Install it with `pip install control` or `pip install -r requirements.txt`."
        ) from exc
    return control_module


def build_linear_plant(vp: VehicleParams):
    control_module = _require_control()
    return control_module.tf([vp.K], [vp.tau, 1.0])


def build_disturbance_plant(vp: VehicleParams):
    control_module = _require_control()
    return control_module.tf([-vp.K_d], [vp.tau, 1.0])


def nonlinear_rhs(
    t: float,
    y: Sequence[float],
    vp: VehicleParams,
    controller,
    v_ref_func,
    theta_func,
) -> list[float]:
    v = max(float(y[0]), 0.0)
    controller_state = np.asarray(y[1:], dtype=float)

    v_ref = float(v_ref_func(t))
    theta = float(theta_func(t))
    error = v_ref - v

    u, controller_state_dot = controller.compute(
        error=error,
        state=controller_state,
        measurement=v,
        v_ref=v_ref,
        t=t,
    )

    F_drag = 0.5 * vp.rho * vp.C_d * vp.A * v**2
    F_roll = vp.C_rr * vp.m * vp.g * np.cos(theta)
    F_grade = vp.m * vp.g * np.sin(theta)
    dv_dt = (u - F_drag - F_roll - F_grade) / vp.m

    return [dv_dt, *np.asarray(controller_state_dot, dtype=float).tolist()]
