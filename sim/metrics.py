from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .simulate import SimResult


def _after_time_mask(res: SimResult, t_event: float) -> np.ndarray:
    return res.t >= t_event


def _reference_window_mean(values: np.ndarray, window: int) -> float:
    window = max(1, min(window, len(values)))
    return float(np.mean(values[-window:]))


def rise_time(res: SimResult, t_step: float) -> float:
    mask = _after_time_mask(res, t_step)
    if not np.any(mask):
        return float("nan")

    t = res.t[mask]
    v = res.v[mask]
    pre_mask = res.t < t_step
    v_init = float(np.mean(res.v[pre_mask][-10:])) if np.any(pre_mask) else float(res.v[0])
    v_final = _reference_window_mean(res.v_ref[mask], 20)
    delta = v_final - v_init

    if np.isclose(delta, 0.0):
        return float("nan")

    lower = v_init + 0.1 * delta
    upper = v_init + 0.9 * delta

    if delta > 0:
        idx_10 = np.flatnonzero(v >= lower)
        idx_90 = np.flatnonzero(v >= upper)
    else:
        idx_10 = np.flatnonzero(v <= lower)
        idx_90 = np.flatnonzero(v <= upper)

    if len(idx_10) == 0 or len(idx_90) == 0:
        return float("nan")

    return float(t[idx_90[0]] - t[idx_10[0]])


def settling_time(res: SimResult, t_step: float, band: float = 0.02) -> float:
    mask = _after_time_mask(res, t_step)
    if not np.any(mask):
        return float("nan")

    t = res.t[mask]
    v = res.v[mask]
    pre_mask = res.t < t_step
    v_init = float(np.mean(res.v[pre_mask][-10:])) if np.any(pre_mask) else float(res.v[0])
    v_final = _reference_window_mean(v, max(5, len(v) // 10))
    step_mag = abs(v_final - v_init)

    if np.isclose(step_mag, 0.0):
        return float("nan")

    threshold = band * step_mag
    outside = np.flatnonzero(np.abs(v - v_final) > threshold)
    if len(outside) == 0:
        return 0.0
    return float(t[outside[-1]] - t_step)


def overshoot(res: SimResult, t_step: float) -> float:
    mask = _after_time_mask(res, t_step)
    if not np.any(mask):
        return float("nan")

    v = res.v[mask]
    pre_mask = res.t < t_step
    v_init = float(np.mean(res.v[pre_mask][-10:])) if np.any(pre_mask) else float(res.v[0])
    v_final = _reference_window_mean(res.v_ref[mask], 20)
    delta = v_final - v_init

    if np.isclose(delta, 0.0):
        return 0.0

    peak = float(np.max(v) if delta > 0 else np.min(v))
    if delta > 0:
        return max(0.0, (peak - v_final) / abs(delta) * 100.0)
    return max(0.0, (v_final - peak) / abs(delta) * 100.0)


def steady_state_error(res: SimResult) -> float:
    v_final = _reference_window_mean(res.v, max(5, len(res.v) // 10))
    ref_final = _reference_window_mean(res.v_ref, max(5, len(res.v_ref) // 10))
    if np.isclose(ref_final, 0.0):
        return float("nan")
    return float((ref_final - v_final) / ref_final * 100.0)


def integral_errors(res: SimResult) -> dict[str, float]:
    e_abs = np.abs(res.e)
    return {
        "IAE": float(np.trapezoid(e_abs, x=res.t)),
        "ISE": float(np.trapezoid(res.e**2, x=res.t)),
        "ITAE": float(np.trapezoid(res.t * e_abs, x=res.t)),
    }


def peak_disturbance_deviation(res: SimResult, t_dist: float) -> float:
    mask = _after_time_mask(res, t_dist)
    if not np.any(mask):
        return float("nan")
    return float(np.max(np.abs(res.v[mask] - res.v_ref[mask])))


def control_effort_metrics(
    res: SimResult,
    *,
    u_min: float | None = None,
    u_max: float | None = None,
) -> dict[str, float]:
    saturation_fraction = 0.0
    if u_min is not None and u_max is not None:
        sat_mask = np.isclose(res.u, u_min, atol=1e-6) | np.isclose(res.u, u_max, atol=1e-6)
        saturation_fraction = float(np.mean(sat_mask))

    return {
        "peak_control_effort_N": float(np.max(np.abs(res.u))),
        "total_variation_N": float(np.sum(np.abs(np.diff(res.u)))),
        "saturation_fraction": saturation_fraction,
    }


def compute_all_metrics(
    res: SimResult,
    *,
    t_step: float,
    t_dist: float | None = None,
    u_min: float | None = None,
    u_max: float | None = None,
) -> dict:
    metrics = {
        "scenario": res.scenario,
        "controller": res.label,
        "controller_key": res.controller_key,
        "model": res.model,
        "metrics": {
            "rise_time_s": rise_time(res, t_step),
            "settling_time_s": settling_time(res, t_step),
            "overshoot_pct": overshoot(res, t_step),
            "ss_error_pct": steady_state_error(res),
            **integral_errors(res),
            **control_effort_metrics(res, u_min=u_min, u_max=u_max),
        },
    }
    if t_dist is not None:
        metrics["metrics"]["peak_deviation_mps"] = peak_disturbance_deviation(res, t_dist)
    if res.metadata:
        metrics["metadata"] = dict(res.metadata)
    return metrics


def export_metrics(metrics: dict, output_dir: str = "results/metrics") -> Path:
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    scenario = metrics["scenario"].replace(" ", "_")
    controller_key = metrics.get("controller_key", metrics["controller"]).replace(" ", "_")
    model = metrics["model"].replace(" ", "_")
    path = out_dir / f"{scenario}__{controller_key}__{model}.json"
    with path.open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2)
    return path
