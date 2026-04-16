from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from sim.controllers import (
    ControllerSpec,
    build_all_controllers,
    build_linear_controller,
    build_nonlinear_controller,
)
from sim.metrics import compute_all_metrics, export_metrics
from sim.params import ControllerParams, VehicleParams
from sim.plant import build_linear_plant
from sim.plotting import (
    plot_bode,
    plot_error_overlay,
    plot_linear_vs_nonlinear,
    plot_metrics_bar_chart,
    plot_open_loop_step,
    plot_pole_map,
    plot_servo_comparison,
    plot_sweep,
)
from sim.scenarios import Scenario, build_all_scenarios, make_constant_grade, make_step_reference
from sim.simulate import run_linear, run_nonlinear


CONTROLLER_LABELS = {
    "none": "No anti-windup",
    "clamp": "Clamp anti-windup",
    "back_calc": "Back-calculation anti-windup",
}


def _require_control():
    try:
        import control as control_module
    except ImportError as exc:
        raise RuntimeError(
            "run_all.py requires python-control. Install dependencies with "
            "`pip install -r requirements.txt`."
        ) from exc
    return control_module


def verify_plant(plant_tf, vp: VehicleParams) -> None:
    ct = _require_control()
    pole = float(np.real(ct.poles(plant_tf)[0]))
    dc_gain = float(ct.dcgain(plant_tf))
    assert np.isclose(pole, -1.0 / vp.tau, rtol=1e-2)
    assert np.isclose(dc_gain, vp.K, rtol=1e-3)
    assert np.isclose(vp.u0, 468.8, atol=1.0)
    assert np.isclose(vp.tau, 64.1, atol=1.0)


def verify_controllers(controllers: dict[str, ControllerSpec], plant_tf) -> dict[str, np.ndarray]:
    ct = _require_control()
    poles_by_controller: dict[str, np.ndarray] = {}
    for key, spec in controllers.items():
        controller_tf = build_linear_controller(spec)
        poles = np.asarray(ct.poles(ct.feedback(controller_tf * plant_tf)))
        poles_by_controller[key] = poles

    p_poles = poles_by_controller["P_295"]
    assert len(p_poles) == 1
    assert np.isclose(np.real(p_poles[0]), -0.2, atol=0.03)

    imc_poles = poles_by_controller["PI_IMC_lambda5"]
    assert np.min(np.abs(np.real(imc_poles) + 0.2)) < 1e-3

    zn_poles = poles_by_controller["PI_ZN"]
    assert np.any(np.abs(np.imag(zn_poles)) > 1e-6)
    return poles_by_controller


def run_scenario(
    *,
    vp: VehicleParams,
    scenario: Scenario,
    controllers: dict[str, ControllerSpec],
    linear_results: list,
    nonlinear_results: list,
    metrics_records: list[dict],
    anti_windup_mode: str | None = None,
) -> None:
    plant_tf = build_linear_plant(vp)

    for controller_key in scenario.controller_keys:
        spec = controllers[controller_key]
        controller_tf = build_linear_controller(spec)

        if scenario.model in {"linear", "both"}:
            res = run_linear(
                vp=vp,
                plant_tf=plant_tf,
                controller_tf=controller_tf,
                t_span=scenario.t_span,
                v_ref_func=scenario.v_ref_func,
                theta_func=scenario.theta_func if scenario.t_dist is not None else None,
                label=spec.params.label,
                scenario=scenario.name,
                controller_key=controller_key,
            )
            linear_results.append(res)
            metrics = compute_all_metrics(
                res,
                t_step=scenario.t_step,
                t_dist=scenario.t_dist,
                u_min=vp.u_min,
                u_max=vp.u_max,
            )
            export_metrics(metrics)
            metrics_records.append(metrics)

        if scenario.model in {"nonlinear", "both"}:
            controller = build_nonlinear_controller(
                spec,
                vp,
                anti_windup=anti_windup_mode or "back_calc",
            )
            res = run_nonlinear(
                vp=vp,
                controller=controller,
                t_span=scenario.t_span,
                v_ref_func=scenario.v_ref_func,
                theta_func=scenario.theta_func,
                label=spec.params.label,
                scenario=scenario.name,
                controller_key=controller_key if anti_windup_mode is None else anti_windup_mode,
            )
            if anti_windup_mode is not None:
                res.label = CONTROLLER_LABELS[anti_windup_mode]
            nonlinear_results.append(res)
            metrics = compute_all_metrics(
                res,
                t_step=scenario.t_step,
                t_dist=scenario.t_dist,
                u_min=vp.u_min,
                u_max=vp.u_max,
            )
            export_metrics(metrics)
            metrics_records.append(metrics)


def compile_summary(metrics_records: list[dict]) -> dict:
    summary = {
        "S1": [item for item in metrics_records if item["scenario"] == "S1"],
        "D1": [item for item in metrics_records if item["scenario"] == "D1"],
    }
    out_path = Path("results/metrics/summary.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def make_lambda_sweep_spec(vp: VehicleParams, lambda_value: float) -> ControllerSpec:
    return ControllerSpec(
        key=f"lambda_{int(lambda_value)}",
        kind="PI",
        params=ControllerParams(
            Kp=float(vp.tau / (vp.K * lambda_value)),
            tau_I=vp.tau,
            label=f"PI IMC (λ={int(lambda_value)})",
            key=f"lambda_{int(lambda_value)}",
        ),
        notes="IMC sweep controller.",
    )


def main() -> None:
    ct = _require_control()
    vp = VehicleParams()
    scenarios = build_all_scenarios()
    controllers = build_all_controllers(vp)
    plant_tf = build_linear_plant(vp)

    verify_plant(plant_tf, vp)
    poles_by_controller = verify_controllers(controllers, plant_tf)

    t_ol = np.linspace(0.0, 300.0, 2000)
    step_ol = ct.step_response(plant_tf, T=t_ol)
    plot_open_loop_step(step_ol.time, np.asarray(step_ol.outputs))
    plot_bode(plant_tf)
    plot_pole_map(poles_by_controller)

    metrics_records: list[dict] = []

    s1_linear: list = []
    s1_nonlinear: list = []
    run_scenario(
        vp=vp,
        scenario=scenarios["S1"],
        controllers=controllers,
        linear_results=s1_linear,
        nonlinear_results=s1_nonlinear,
        metrics_records=metrics_records,
    )
    plot_servo_comparison(s1_linear, title="S1 servo comparison (linear)", filename="B1_servo_linear")
    plot_servo_comparison(
        s1_nonlinear,
        title="S1 servo comparison (nonlinear)",
        filename="B1_servo_nonlinear",
    )
    plot_error_overlay(s1_linear, title="S1 error comparison (linear)", filename="B2_error_linear")

    d1_linear: list = []
    d1_nonlinear: list = []
    run_scenario(
        vp=vp,
        scenario=scenarios["D1"],
        controllers=controllers,
        linear_results=d1_linear,
        nonlinear_results=d1_nonlinear,
        metrics_records=metrics_records,
    )
    plot_servo_comparison(
        d1_linear,
        title="D1 disturbance rejection (linear)",
        filename="C1_disturbance_linear",
    )
    plot_servo_comparison(
        d1_nonlinear,
        title="D1 disturbance rejection (nonlinear)",
        filename="C1_disturbance_nonlinear",
    )

    s3_linear: list = []
    s3_nonlinear: list = []
    run_scenario(
        vp=vp,
        scenario=scenarios["S3"],
        controllers=controllers,
        linear_results=s3_linear,
        nonlinear_results=s3_nonlinear,
        metrics_records=metrics_records,
    )
    plot_linear_vs_nonlinear(
        s3_linear,
        s3_nonlinear,
        step_labels=["+1 m/s"],
        filename="D2_small_step_validation",
        title="Linear vs nonlinear validation",
    )

    d3_nonlinear: list = []
    run_scenario(
        vp=vp,
        scenario=scenarios["D3"],
        controllers=controllers,
        linear_results=[],
        nonlinear_results=d3_nonlinear,
        metrics_records=metrics_records,
    )
    plot_servo_comparison(d3_nonlinear, title="D3 grade pulse", filename="C2_grade_pulse")

    d2_nonlinear: list = []
    run_scenario(
        vp=vp,
        scenario=scenarios["D2"],
        controllers=controllers,
        linear_results=[],
        nonlinear_results=d2_nonlinear,
        metrics_records=metrics_records,
    )
    plot_servo_comparison(d2_nonlinear, title="D2 steep grade", filename="C3_steep_grade")

    sweep_linear: list = []
    sweep_nonlinear: list = []
    for lambda_value in [1.0, 2.0, 5.0, 10.0, 20.0]:
        sweep_spec = make_lambda_sweep_spec(vp, lambda_value)
        sweep_scenario = Scenario(
            name="E3",
            controller_keys=[sweep_spec.key],
            v_ref_func=scenarios["E3"].v_ref_func,
            theta_func=scenarios["E3"].theta_func,
            t_span=scenarios["E3"].t_span,
            model="both",
            t_step=scenarios["E3"].t_step,
            description=scenarios["E3"].description,
        )
        run_scenario(
            vp=vp,
            scenario=sweep_scenario,
            controllers={sweep_spec.key: sweep_spec},
            linear_results=sweep_linear,
            nonlinear_results=sweep_nonlinear,
            metrics_records=metrics_records,
        )
    plot_sweep(sweep_linear, title="IMC λ sweep (linear)", filename="B3_lambda_sweep_linear")
    plot_sweep(sweep_nonlinear, title="IMC λ sweep (nonlinear)", filename="B3_lambda_sweep_nonlinear")

    divergence_steps = [2.0, 5.0, 10.0, 15.0]
    e1_linear = []
    e1_nonlinear = []
    controller_spec = controllers["PI_IMC_lambda5"]
    controller_tf = build_linear_controller(controller_spec)
    for step in divergence_steps:
        ref = make_step_reference(vp.v0, vp.v0 + step, 10.0)
        e1_linear.append(
            run_linear(
                vp=vp,
                plant_tf=plant_tf,
                controller_tf=controller_tf,
                t_span=(0.0, 80.0),
                v_ref_func=ref,
                theta_func=make_constant_grade(),
                label=f"Linear +{step:g}",
                scenario="E1",
                controller_key=f"linear_{step:g}",
            )
        )
        e1_nonlinear.append(
            run_nonlinear(
                vp=vp,
                controller=build_nonlinear_controller(controller_spec, vp),
                t_span=(0.0, 80.0),
                v_ref_func=ref,
                theta_func=make_constant_grade(),
                label=f"Nonlinear +{step:g}",
                scenario="E1",
                controller_key=f"nonlinear_{step:g}",
            )
        )
    plot_linear_vs_nonlinear(
        e1_linear,
        e1_nonlinear,
        [f"+{step:g} m/s" for step in divergence_steps],
        filename="D1_linear_vs_nonlinear",
        title="Linearization divergence vs step size",
    )

    anti_windup_results = []
    for mode in ["none", "clamp", "back_calc"]:
        run_scenario(
            vp=vp,
            scenario=scenarios["E2"],
            controllers={"PI_IMC_lambda5": controllers["PI_IMC_lambda5"]},
            linear_results=[],
            nonlinear_results=anti_windup_results,
            metrics_records=metrics_records,
            anti_windup_mode=mode,
        )
    plot_servo_comparison(
        anti_windup_results,
        title="Anti-windup comparison",
        filename="F1_anti_windup",
    )

    robustness_mass = []
    for mass in [1200.0, 1400.0, 1600.0, 1800.0, 2000.0]:
        vp_mass = vp.with_updates(m=mass)
        result = run_nonlinear(
            vp=vp_mass,
            controller=build_nonlinear_controller(controllers["PI_IMC_lambda5"], vp_mass),
            t_span=(0.0, 80.0),
            v_ref_func=make_step_reference(25.0, 30.0, 10.0),
            theta_func=make_constant_grade(),
            label=f"m={int(mass)} kg",
            scenario="R1",
            controller_key=f"mass_{int(mass)}",
        )
        robustness_mass.append(result)
        metrics_records.append(
            compute_all_metrics(result, t_step=10.0, u_min=vp_mass.u_min, u_max=vp_mass.u_max)
        )
    plot_sweep(robustness_mass, title="Mass variation robustness", filename="E1_mass_variation")

    robustness_speed = []
    for v0 in [15.0, 20.0, 25.0, 30.0, 35.0]:
        vp_speed = vp.with_updates(v0=v0)
        result = run_nonlinear(
            vp=vp_speed,
            controller=build_nonlinear_controller(controllers["PI_IMC_lambda5"], vp_speed),
            t_span=(0.0, 80.0),
            v_ref_func=make_step_reference(v0, v0 + 3.0, 10.0),
            theta_func=make_constant_grade(),
            label=f"v0={int(v0)} m/s",
            scenario="R2",
            controller_key=f"speed_{int(v0)}",
        )
        robustness_speed.append(result)
        metrics_records.append(
            compute_all_metrics(result, t_step=10.0, u_min=vp_speed.u_min, u_max=vp_speed.u_max)
        )
    plot_sweep(robustness_speed, title="Operating-point variation", filename="E2_speed_variation")

    robustness_drag = []
    for drag in [0.24, 0.28, 0.32, 0.36, 0.40]:
        vp_drag = vp.with_updates(C_d=drag)
        result = run_nonlinear(
            vp=vp_drag,
            controller=build_nonlinear_controller(controllers["PI_IMC_lambda5"], vp_drag),
            t_span=(0.0, 120.0),
            v_ref_func=lambda t, v0=vp_drag.v0: v0,
            theta_func=lambda t: 0.0 if t < 30.0 else np.deg2rad(4.0),
            label=f"Cd={drag:.2f}",
            scenario="R3",
            controller_key=f"drag_{drag:.2f}",
        )
        robustness_drag.append(result)
        metrics_records.append(
            compute_all_metrics(
                result,
                t_step=0.0,
                t_dist=30.0,
                u_min=vp_drag.u_min,
                u_max=vp_drag.u_max,
            )
        )
    plot_sweep(robustness_drag, title="Drag variation under grade", filename="E3_drag_variation")

    s1_linear_metrics = [
        item
        for item in metrics_records
        if item["scenario"] == "S1" and item["model"] == "linear"
    ]
    plot_metrics_bar_chart(s1_linear_metrics)

    compile_summary(metrics_records)
    print("Completed cruise-control simulation pipeline.")
    print("Metrics summary written to results/metrics/summary.json")


if __name__ == "__main__":
    main()
