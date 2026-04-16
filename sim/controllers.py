from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import minimize

from .params import ControllerParams, VehicleParams

if TYPE_CHECKING:
    import control as ct
else:
    ct = None


def _require_control():
    try:
        import control as control_module
    except ImportError as exc:
        raise RuntimeError(
            "python-control is required for controller transfer functions. "
            "Install it with `pip install control`."
        ) from exc
    return control_module


@dataclass(frozen=True, slots=True)
class ControllerSpec:
    key: str
    kind: str
    params: ControllerParams
    notes: str = ""


class StatefulController:
    state_size: int = 0

    def __init__(
        self,
        cp: ControllerParams,
        vp: VehicleParams,
        anti_windup: str = "back_calc",
    ) -> None:
        self.cp = cp
        self.vp = vp
        self.anti_windup = anti_windup
        self.u_min = vp.u_min
        self.u_max = vp.u_max
        self.u0 = vp.u0

    def initial_state(self) -> np.ndarray:
        return np.zeros(self.state_size, dtype=float)

    def _apply_saturation(self, u_unsat: float) -> tuple[float, bool]:
        u_sat = float(np.clip(u_unsat, self.u_min, self.u_max))
        return u_sat, not np.isclose(u_sat, u_unsat)


class PControllerODE(StatefulController):
    state_size = 0

    def compute(
        self,
        *,
        error: float,
        state: np.ndarray,
        measurement: float,
        v_ref: float,
        t: float,
    ) -> tuple[float, np.ndarray]:
        del state, measurement, v_ref, t
        u_unsat = self.u0 + self.cp.Kp * error
        u, _ = self._apply_saturation(u_unsat)
        return u, np.zeros(0, dtype=float)


class PIControllerODE(StatefulController):
    state_size = 1

    @property
    def Ki(self) -> float:
        return 0.0 if self.cp.tau_I is None else self.cp.Kp / self.cp.tau_I

    @property
    def Kt(self) -> float:
        return 0.0 if self.cp.tau_I is None else 1.0 / self.cp.tau_I

    def compute(
        self,
        *,
        error: float,
        state: np.ndarray,
        measurement: float,
        v_ref: float,
        t: float,
    ) -> tuple[float, np.ndarray]:
        del measurement, v_ref, t
        x_i = float(state[0]) if len(state) else 0.0
        u_unsat = self.u0 + self.cp.Kp * error + x_i
        u, saturated = self._apply_saturation(u_unsat)

        if self.anti_windup == "none":
            dx_i = self.Ki * error
        elif self.anti_windup == "clamp":
            dx_i = 0.0 if saturated else self.Ki * error
        elif self.anti_windup == "back_calc":
            dx_i = self.Ki * error + self.Kt * (u - u_unsat)
        else:
            raise ValueError(f"Unknown anti_windup mode: {self.anti_windup}")

        return u, np.array([dx_i], dtype=float)


class PIDControllerODE(PIControllerODE):
    state_size = 2

    def compute(
        self,
        *,
        error: float,
        state: np.ndarray,
        measurement: float,
        v_ref: float,
        t: float,
    ) -> tuple[float, np.ndarray]:
        del v_ref, t
        x_i = float(state[0])
        z = float(state[1])

        tau_d = self.cp.tau_D or 0.0
        n_filter = self.cp.N_filter
        derivative_term = -self.cp.Kp * n_filter * (measurement - z)
        u_unsat = self.u0 + self.cp.Kp * error + x_i + derivative_term
        u, saturated = self._apply_saturation(u_unsat)

        if self.anti_windup == "none":
            dx_i = self.Ki * error
        elif self.anti_windup == "clamp":
            dx_i = 0.0 if saturated else self.Ki * error
        elif self.anti_windup == "back_calc":
            dx_i = self.Ki * error + self.Kt * (u - u_unsat)
        else:
            raise ValueError(f"Unknown anti_windup mode: {self.anti_windup}")

        if tau_d <= 0.0:
            dz = 0.0
        else:
            dz = (n_filter / tau_d) * (measurement - z)

        return u, np.array([dx_i, dz], dtype=float)


def build_p_controller(cp: ControllerParams):
    control_module = _require_control()
    return control_module.tf([cp.Kp], [1.0])


def build_pi_controller(cp: ControllerParams):
    if cp.tau_I is None:
        raise ValueError("PI controller requires tau_I.")
    control_module = _require_control()
    return control_module.tf([cp.Kp * cp.tau_I, cp.Kp], [cp.tau_I, 0.0])


def build_pid_controller(cp: ControllerParams):
    if cp.tau_I is None or cp.tau_D is None:
        raise ValueError("PID controller requires both tau_I and tau_D.")
    control_module = _require_control()
    p = control_module.tf([cp.Kp], [1.0])
    i = control_module.tf([cp.Kp], [cp.tau_I, 0.0])
    d = control_module.tf(
        [cp.Kp * cp.tau_D, 0.0],
        [cp.tau_D / cp.N_filter, 1.0],
    )
    return p + i + d


def build_linear_controller(spec: ControllerSpec):
    if spec.kind == "P":
        return build_p_controller(spec.params)
    if spec.kind == "PI":
        return build_pi_controller(spec.params)
    if spec.kind == "PID":
        return build_pid_controller(spec.params)
    raise ValueError(f"Unsupported controller kind: {spec.kind}")


def build_nonlinear_controller(
    spec: ControllerSpec,
    vp: VehicleParams,
    *,
    anti_windup: str = "back_calc",
):
    if spec.kind == "P":
        return PControllerODE(spec.params, vp, anti_windup=anti_windup)
    if spec.kind == "PI":
        return PIControllerODE(spec.params, vp, anti_windup=anti_windup)
    if spec.kind == "PID":
        return PIDControllerODE(spec.params, vp, anti_windup=anti_windup)
    raise ValueError(f"Unsupported controller kind: {spec.kind}")


def _step_objective(log_params: np.ndarray, vp: VehicleParams, kind: str) -> float:
    control_module = _require_control()
    plant = control_module.tf([vp.K], [vp.tau, 1.0])

    if kind == "PI":
        Kp, tau_i = np.exp(log_params)
        controller = build_pi_controller(
            ControllerParams(Kp=Kp, tau_I=tau_i, label="PI ITAE")
        )
    elif kind == "PID":
        Kp, tau_i, tau_d = np.exp(log_params)
        controller = build_pid_controller(
            ControllerParams(
                Kp=Kp,
                tau_I=tau_i,
                tau_D=tau_d,
                N_filter=10.0,
                label="PID comparison",
            )
        )
    else:
        raise ValueError(f"Unsupported optimization kind: {kind}")

    closed_loop = control_module.feedback(controller * plant)
    horizon = max(120.0, 10.0 * vp.tau)
    t = np.linspace(0.0, horizon, 5000)
    response = control_module.step_response(closed_loop, T=t)
    y = np.asarray(response.outputs, dtype=float)
    e = 1.0 - y

    itae = float(np.trapezoid(t * np.abs(e), x=t))
    overshoot = max(0.0, float(np.max(y) - 1.0))
    early_effort = float(np.max(np.abs(np.gradient(y, t))))
    settling_penalty = float(abs(1.0 - np.mean(y[-200:])))

    return itae + 400.0 * overshoot**2 + 2.5 * early_effort + 800.0 * settling_penalty


def design_itae_pi(vp: VehicleParams) -> ControllerParams:
    x0 = np.log([vp.tau / (vp.K * 5.0), vp.tau * 0.75])
    bounds = [
        (np.log(50.0), np.log(2000.0)),
        (np.log(1.0), np.log(2.5 * vp.tau)),
    ]
    result = minimize(
        _step_objective,
        x0=x0,
        args=(vp, "PI"),
        method="L-BFGS-B",
        bounds=bounds,
    )
    Kp, tau_i = np.exp(result.x if result.success else x0)
    return ControllerParams(
        Kp=float(Kp),
        tau_I=float(tau_i),
        label="PI ITAE",
        key="PI_ITAE",
    )


def design_pid_comparison(vp: VehicleParams) -> ControllerParams:
    tau_engine = 1.0
    base_kp = (vp.tau + tau_engine) / (vp.K * 5.0)
    base_ti = vp.tau + tau_engine
    base_td = (vp.tau * tau_engine) / (vp.tau + tau_engine)
    x0 = np.log([base_kp, base_ti, base_td])
    bounds = [
        (np.log(80.0), np.log(1500.0)),
        (np.log(5.0), np.log(2.0 * vp.tau)),
        (np.log(0.05), np.log(12.0)),
    ]
    result = minimize(
        _step_objective,
        x0=x0,
        args=(vp, "PID"),
        method="L-BFGS-B",
        bounds=bounds,
    )
    Kp, tau_i, tau_d = np.exp(result.x if result.success else x0)
    return ControllerParams(
        Kp=float(Kp),
        tau_I=float(tau_i),
        tau_D=float(tau_d),
        N_filter=10.0,
        label="PID IMC",
        key="PID_IMC",
    )


def build_all_controllers(vp: VehicleParams) -> dict[str, ControllerSpec]:
    pi_itae = design_itae_pi(vp)
    pid_comp = design_pid_comparison(vp)

    specs = [
        ControllerSpec(
            key="P_295",
            kind="P",
            params=ControllerParams(Kp=295.0, label="P (Kp=295)", key="P_295"),
            notes="Baseline proportional controller.",
        ),
        ControllerSpec(
            key="PI_IMC_lambda5",
            kind="PI",
            params=ControllerParams(
                Kp=320.0,
                tau_I=vp.tau,
                label="PI IMC (λ=5)",
                key="PI_IMC_lambda5",
            ),
            notes="Primary design case.",
        ),
        ControllerSpec(
            key="PI_IMC_lambda1",
            kind="PI",
            params=ControllerParams(
                Kp=1603.0,
                tau_I=vp.tau,
                label="PI IMC (λ=1)",
                key="PI_IMC_lambda1",
            ),
            notes="Aggressive IMC tuning.",
        ),
        ControllerSpec(
            key="PI_IMC_lambda10",
            kind="PI",
            params=ControllerParams(
                Kp=160.0,
                tau_I=vp.tau,
                label="PI IMC (λ=10)",
                key="PI_IMC_lambda10",
            ),
            notes="Conservative IMC tuning.",
        ),
        ControllerSpec(
            key="PI_ZN",
            kind="PI",
            params=ControllerParams(
                Kp=1442.0,
                tau_I=3.3,
                label="PI ZN",
                key="PI_ZN",
            ),
            notes="Aggressive comparison case.",
        ),
        ControllerSpec(
            key="PI_ITAE",
            kind="PI",
            params=pi_itae,
            notes="Numerically tuned ITAE comparison controller.",
        ),
        ControllerSpec(
            key="PID_IMC",
            kind="PID",
            params=pid_comp,
            notes=(
                "Filtered PID comparison controller tuned around an IMC-style "
                "auxiliary-lag surrogate."
            ),
        ),
    ]
    return {spec.key: spec for spec in specs}
