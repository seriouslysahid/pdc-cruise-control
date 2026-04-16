from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np


@dataclass(frozen=True, slots=True)
class Scenario:
    name: str
    controller_keys: list[str]
    v_ref_func: Callable[[float], float]
    theta_func: Callable[[float], float]
    t_span: tuple[float, float]
    model: str
    t_step: float = 10.0
    t_dist: float | None = None
    description: str = ""
    tags: tuple[str, ...] = field(default_factory=tuple)


def deg_to_rad(value: float) -> float:
    return float(np.deg2rad(value))


def make_step_reference(v_initial: float, v_final: float, t_step: float) -> Callable[[float], float]:
    return lambda t: v_initial if t < t_step else v_final


def make_grade_step(theta_initial_deg: float, theta_final_deg: float, t_step: float):
    theta_initial = deg_to_rad(theta_initial_deg)
    theta_final = deg_to_rad(theta_final_deg)
    return lambda t: theta_initial if t < t_step else theta_final


def make_grade_pulse(theta_deg: float, t_on: float, t_off: float):
    theta = deg_to_rad(theta_deg)

    def _profile(t: float) -> float:
        return theta if t_on <= t <= t_off else 0.0

    return _profile


def make_constant_grade(theta_deg: float = 0.0):
    theta = deg_to_rad(theta_deg)
    return lambda t: theta


def build_all_scenarios() -> dict[str, Scenario]:
    all_controllers = [
        "P_295",
        "PI_IMC_lambda5",
        "PI_IMC_lambda1",
        "PI_IMC_lambda10",
        "PI_ZN",
        "PI_ITAE",
        "PID_IMC",
    ]
    scenarios = [
        Scenario(
            name="S1",
            controller_keys=all_controllers,
            v_ref_func=make_step_reference(25.0, 30.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 80.0),
            model="both",
            t_step=10.0,
            description="Moderate servo step.",
        ),
        Scenario(
            name="S2",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=make_step_reference(15.0, 30.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 120.0),
            model="nonlinear",
            t_step=10.0,
            description="Large step to expose saturation and windup.",
        ),
        Scenario(
            name="S3",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=make_step_reference(25.0, 26.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 60.0),
            model="both",
            t_step=10.0,
            description="Small-step cross-model validation.",
        ),
        Scenario(
            name="D1",
            controller_keys=all_controllers,
            v_ref_func=lambda t: 25.0,
            theta_func=make_grade_step(0.0, 4.0, 30.0),
            t_span=(0.0, 120.0),
            model="both",
            t_step=0.0,
            t_dist=30.0,
            description="Moderate grade disturbance rejection.",
        ),
        Scenario(
            name="D2",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=lambda t: 25.0,
            theta_func=make_grade_step(0.0, 8.0, 30.0),
            t_span=(0.0, 150.0),
            model="nonlinear",
            t_step=0.0,
            t_dist=30.0,
            description="Steep-grade nonlinear stress test.",
        ),
        Scenario(
            name="D3",
            controller_keys=["PI_IMC_lambda5", "P_295"],
            v_ref_func=lambda t: 25.0,
            theta_func=make_grade_pulse(4.0, 30.0, 80.0),
            t_span=(0.0, 160.0),
            model="nonlinear",
            t_step=0.0,
            t_dist=30.0,
            description="Grade pulse hill-climb test.",
        ),
        Scenario(
            name="E2",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=make_step_reference(15.0, 30.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 120.0),
            model="nonlinear",
            t_step=10.0,
            description="Anti-windup comparison.",
        ),
        Scenario(
            name="R1",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=make_step_reference(25.0, 30.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 80.0),
            model="nonlinear",
            t_step=10.0,
            description="Mass-variation robustness sweep.",
        ),
        Scenario(
            name="R2",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=make_step_reference(25.0, 28.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 80.0),
            model="nonlinear",
            t_step=10.0,
            description="Operating-point robustness sweep.",
        ),
        Scenario(
            name="R3",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=lambda t: 25.0,
            theta_func=make_grade_step(0.0, 4.0, 30.0),
            t_span=(0.0, 120.0),
            model="nonlinear",
            t_step=0.0,
            t_dist=30.0,
            description="Drag-variation robustness sweep.",
        ),
        Scenario(
            name="E1",
            controller_keys=["PI_IMC_lambda5"],
            v_ref_func=make_step_reference(25.0, 27.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 80.0),
            model="both",
            t_step=10.0,
            description="Linearization-divergence sweep.",
        ),
        Scenario(
            name="E3",
            controller_keys=[
                "PI_IMC_lambda1",
                "PI_IMC_lambda5",
                "PI_IMC_lambda10",
            ],
            v_ref_func=make_step_reference(25.0, 30.0, 10.0),
            theta_func=make_constant_grade(),
            t_span=(0.0, 80.0),
            model="both",
            t_step=10.0,
            description="IMC aggressiveness sweep baseline cases.",
        ),
    ]
    return {scenario.name: scenario for scenario in scenarios}
