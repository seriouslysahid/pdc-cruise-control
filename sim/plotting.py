from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from .simulate import SimResult

STYLE = {
    "figure.figsize": (10, 6),
    "font.size": 11,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.color": "#bbbbbb",
    "lines.linewidth": 1.8,
}
mpl.rcParams.update(STYLE)

CONTROLLER_STYLES = {
    "P_295": {"color": "gray", "ls": "-", "label": "P (Kp=295)"},
    "PI_IMC_lambda5": {"color": "#1f77b4", "ls": "-", "label": "PI IMC (λ=5)"},
    "PI_IMC_lambda1": {"color": "#d62728", "ls": "-", "label": "PI IMC (λ=1)"},
    "PI_IMC_lambda10": {"color": "#2ca02c", "ls": "-", "label": "PI IMC (λ=10)"},
    "PI_ZN": {"color": "#ff7f0e", "ls": "--", "label": "PI ZN"},
    "PI_ITAE": {"color": "#9467bd", "ls": "-.", "label": "PI ITAE"},
    "PID_IMC": {"color": "#8c564b", "ls": "-", "label": "PID IMC"},
    "none": {"color": "#666666", "ls": "-", "label": "No anti-windup"},
    "clamp": {"color": "#2ca02c", "ls": "--", "label": "Clamp"},
    "back_calc": {"color": "#1f77b4", "ls": "-", "label": "Back-calculation"},
}


def _style_for(key: str, fallback: str) -> dict[str, str]:
    style = CONTROLLER_STYLES.get(key, {}).copy()
    style.setdefault("color", None)
    style.setdefault("ls", "-")
    style.setdefault("label", fallback)
    return style


def _save(fig: plt.Figure, filename: str) -> None:
    out_dir = Path("results/plots")
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / f"{filename}.png", dpi=300, bbox_inches="tight")
    fig.savefig(out_dir / f"{filename}.pdf", bbox_inches="tight")
    plt.close(fig)


def plot_open_loop_step(t: np.ndarray, y: np.ndarray, filename: str = "A1_open_loop_step") -> None:
    fig, ax = plt.subplots(layout="constrained")
    ax.plot(t, y, color="#1f77b4")
    ax.set_title("Open-loop step response")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity deviation (m/s)")
    _save(fig, filename)


def plot_bode(plant_tf, filename: str = "A2_bode") -> None:
    try:
        import control as ct
    except ImportError as exc:
        raise RuntimeError("python-control is required for Bode plots.") from exc

    omega = np.logspace(-4, 1, 500)
    mag, phase, omega = ct.frequency_response(plant_tf, omega=omega)
    mag_db = 20.0 * np.log10(np.maximum(np.asarray(mag).squeeze(), 1e-12))
    phase_deg = np.degrees(np.asarray(phase).squeeze())

    fig, (ax_mag, ax_phase) = plt.subplots(2, 1, layout="constrained", figsize=(10, 7))
    ax_mag.semilogx(omega, mag_db, color="#1f77b4")
    ax_mag.set_ylabel("Magnitude (dB)")
    ax_mag.set_title("Open-loop Bode plot")
    ax_phase.semilogx(omega, phase_deg, color="#d62728")
    ax_phase.set_xlabel("Frequency (rad/s)")
    ax_phase.set_ylabel("Phase (deg)")
    _save(fig, filename)


def plot_servo_comparison(
    results: list[SimResult],
    *,
    title: str,
    filename: str,
    reference_label: str = "Reference",
) -> None:
    fig, (ax_v, ax_u) = plt.subplots(
        2,
        1,
        height_ratios=[7, 3],
        layout="constrained",
        figsize=(10, 8),
    )

    for res in results:
        style = _style_for(res.controller_key, res.label)
        ax_v.plot(res.t, res.v, color=style["color"], ls=style["ls"], label=style["label"])
        ax_u.plot(res.t, res.u, color=style["color"], ls=style["ls"])

    if results:
        ax_v.plot(
            results[0].t,
            results[0].v_ref,
            color="black",
            ls=":",
            lw=1.1,
            label=reference_label,
        )

    ax_v.set_title(title)
    ax_v.set_ylabel("Velocity (m/s)")
    ax_v.legend(loc="best")
    ax_u.set_xlabel("Time (s)")
    ax_u.set_ylabel("Control force (N)")
    _save(fig, filename)


def plot_error_overlay(results: list[SimResult], *, title: str, filename: str) -> None:
    fig, ax = plt.subplots(layout="constrained")
    for res in results:
        style = _style_for(res.controller_key, res.label)
        ax.plot(res.t, res.e, color=style["color"], ls=style["ls"], label=style["label"])
    ax.axhline(0.0, color="black", ls=":", lw=0.9)
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m/s)")
    ax.legend(loc="best")
    _save(fig, filename)


def plot_linear_vs_nonlinear(
    lin_results: list[SimResult],
    nl_results: list[SimResult],
    step_labels: list[str],
    *,
    filename: str,
    title: str,
) -> None:
    count = max(1, min(len(lin_results), len(nl_results), len(step_labels)))
    cols = 2
    rows = int(np.ceil(count / cols))
    fig, axs = plt.subplots(rows, cols, layout="constrained", figsize=(12, 4 * rows))
    axs = np.atleast_1d(axs).ravel()
    for ax, lin_res, nl_res, label in zip(axs, lin_results, nl_results, step_labels):
        ax.plot(lin_res.t, lin_res.v, color="#1f77b4", label="Linear")
        ax.plot(nl_res.t, nl_res.v, color="#d62728", ls="--", label="Nonlinear")
        ax.plot(lin_res.t, lin_res.v_ref, color="black", ls=":", lw=0.9, label="Reference")
        ax.set_title(f"Step {label}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend(loc="best")
    for ax in axs[count:]:
        ax.set_visible(False)
    fig.suptitle(title)
    _save(fig, filename)


def plot_pole_map(poles_by_controller: dict[str, np.ndarray], filename: str = "B4_pole_map") -> None:
    fig, ax = plt.subplots(layout="constrained")
    for controller_key, poles in poles_by_controller.items():
        style = _style_for(controller_key, controller_key)
        poles = np.asarray(poles)
        ax.scatter(
            np.real(poles),
            np.imag(poles),
            label=style["label"],
            color=style["color"],
            s=45,
        )
    ax.axvline(0.0, color="black", lw=0.8, ls=":")
    ax.set_title("Closed-loop pole map")
    ax.set_xlabel("Real axis")
    ax.set_ylabel("Imaginary axis")
    ax.legend(loc="best")
    _save(fig, filename)


def plot_sweep(
    results: list[SimResult],
    *,
    title: str,
    filename: str,
    reference: bool = True,
) -> None:
    fig, (ax_v, ax_u) = plt.subplots(
        2,
        1,
        height_ratios=[7, 3],
        layout="constrained",
        figsize=(10, 8),
    )
    for res in results:
        style = _style_for(res.controller_key, res.label)
        ax_v.plot(res.t, res.v, color=style["color"], ls=style["ls"], label=res.label)
        ax_u.plot(res.t, res.u, color=style["color"], ls=style["ls"], label=res.label)

    if reference and results:
        ax_v.plot(results[0].t, results[0].v_ref, color="black", ls=":", lw=0.9, label="Reference")

    ax_v.set_title(title)
    ax_v.set_ylabel("Velocity (m/s)")
    ax_v.legend(loc="best")
    ax_u.set_xlabel("Time (s)")
    ax_u.set_ylabel("Control force (N)")
    _save(fig, filename)


def plot_metrics_bar_chart(metrics_list: list[dict], *, filename: str = "F2_metrics_summary") -> None:
    if not metrics_list:
        return

    metric_keys = ["rise_time_s", "settling_time_s", "overshoot_pct", "IAE"]
    labels = [item["controller"] for item in metrics_list]
    x = np.arange(len(labels))

    fig, axs = plt.subplots(2, 2, layout="constrained", figsize=(12, 8))
    for ax, metric_key in zip(axs.flat, metric_keys):
        values = [item["metrics"].get(metric_key, np.nan) for item in metrics_list]
        ax.bar(x, values, width=0.7, color="#1f77b4")
        ax.set_title(metric_key.replace("_", " "))
        ax.set_xticks(x, labels, rotation=30, ha="right")
    _save(fig, filename)
