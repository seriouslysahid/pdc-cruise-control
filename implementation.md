# Implementation Plan — Python Simulation & Testing Framework

**Project:** PDC Cruise Control Simulation  
**Source:** Section 4 — Python Simulation and Testing Framework  
**Date:** 2026-04-16  
**Status:** Ready for implementation

---

## Overview

This plan implements the dual-model (linear transfer-function + nonlinear ODE) simulation framework for the cruise control system defined in Sections 2–4. The plant is G(s) = 0.0401/(64.1s + 1) at v₀ = 25 m/s. Seven controller configurations (P, PI-IMC ×3, PI-ZN, PI-ITAE, PID) are simulated across 14 scenarios covering servo, regulatory, robustness, and edge-case tests.

The implementation follows the incremental build order prescribed in Section 4 §1: parameters → plant → controllers → linear simulation → nonlinear ODE → anti-windup → metrics → plotting → robustness scenarios. Each phase produces testable intermediate results before adding complexity.

---

## Toolchain

| Library | Version | PyPI Name | Role |
|---|---|---|---|
| **python-control** | ≥ 0.10.2 | `control` | Transfer functions, closed-loop algebra, `step_response`, `forced_response`, Bode, root locus |
| **SciPy** | ≥ 1.14 | `scipy` | `solve_ivp` (RK45) for nonlinear ODE integration |
| **NumPy** | ≥ 2.0 | `numpy` | Array ops; use `np.trapezoid` (not deprecated `np.trapz`, removed in NumPy 2.4) |
| **Matplotlib** | ≥ 3.8 | `matplotlib` | All plotting; use `layout='constrained'` for subplot management |

**Critical API note:** `numpy.trapz` was deprecated in NumPy 2.0 and **removed in NumPy 2.4.0**. All trapezoidal integration must use `numpy.trapezoid(y, x=x)` or equivalently `scipy.integrate.trapezoid`.

### Installation

```bash
pip install numpy scipy matplotlib control
```

### requirements.txt

```
numpy>=2.0
scipy>=1.14
matplotlib>=3.8
control>=0.10.0
```

---

## Project Structure

```
pdc-cruise-control/
├── sim/
│   ├── __init__.py
│   ├── params.py          # Vehicle + controller parameter dataclasses
│   ├── plant.py           # Linear TF + nonlinear ODE RHS
│   ├── controllers.py     # TF-based + stateful ODE-based controllers
│   ├── simulate.py        # run_linear() + run_nonlinear() orchestration
│   ├── metrics.py         # Rise time, settling time, overshoot, ISE, IAE, ITAE
│   ├── plotting.py        # Standardized figure generation
│   └── scenarios.py       # Test matrix definitions
├── run_all.py             # Master execution script
├── results/
│   ├── plots/             # PNG (300 dpi) + PDF output
│   └── metrics/           # JSON metric files
├── requirements.txt
└── implementation.md
```

---

## Phase 1 — Plant Parameters & Linear Plant (Foundation)

**Goal:** Centralize all physical constants, build the linear plant TF, and verify against known analytic values from Section 2.

**Deliverables:** `sim/params.py`, `sim/plant.py` (linear portion only)

### 1.1 — `sim/params.py`

Define two dataclasses as the single source of truth for all numerical values.

```python
from dataclasses import dataclass

@dataclass
class VehicleParams:
    m: float = 1600.0       # kg — vehicle mass
    C_d: float = 0.32       # dimensionless — drag coefficient
    A: float = 2.4          # m² — frontal area
    rho: float = 1.3        # kg/m³ — air density
    C_rr: float = 0.01      # dimensionless — rolling resistance
    g: float = 9.8          # m/s² — gravitational acceleration
    v0: float = 25.0        # m/s — operating point velocity

    @property
    def b_eff(self) -> float:
        """Effective damping at operating point: ρ·C_d·A·v₀"""
        return self.rho * self.C_d * self.A * self.v0  # 24.96 N·s/m

    @property
    def tau(self) -> float:
        """Process time constant: m / b_eff"""
        return self.m / self.b_eff  # 64.1 s

    @property
    def K(self) -> float:
        """Process DC gain: 1 / b_eff"""
        return 1.0 / self.b_eff  # 0.0401 (m/s)/N

    @property
    def u0(self) -> float:
        """Steady-state force at v₀: ½ρC_dAv₀² + C_rr·m·g"""
        return (0.5 * self.rho * self.C_d * self.A * self.v0**2
                + self.C_rr * self.m * self.g)  # 468.8 N

    @property
    def K_d(self) -> float:
        """Disturbance gain: g·m / b_eff"""
        return self.g * self.m / self.b_eff  # 628 (m/s)/rad


@dataclass
class ControllerParams:
    Kp: float
    tau_I: float | None = None   # None → P-only
    tau_D: float | None = None   # None → PI-only
    N_filter: float = 10.0       # Derivative filter coefficient
    label: str = ""              # Human-readable name for plots/metrics
```

Key design decisions:
- Linearization coefficients (`b_eff`, `tau`, `K`) are **computed properties** from physical params — never hardcoded separately. This guarantees consistency when VehicleParams change (e.g., mass variation scenarios).
- `u0` is computed analytically, serving as the operating-point bias for the nonlinear controller.

### 1.2 — `sim/plant.py` (linear model)

```python
import control as ct
from .params import VehicleParams

def build_linear_plant(vp: VehicleParams) -> ct.TransferFunction:
    """G(s) = K / (τs + 1) from VehicleParams."""
    return ct.tf([vp.K], [vp.tau, 1.0])
```

### 1.3 — Verification Checklist

Run these checks before proceeding. Any failure means stop and debug.

| Check | Code | Expected |
|---|---|---|
| Open-loop pole | `plant_tf.poles()` | `[-0.01561]` |
| DC gain | `ct.dcgain(plant_tf)` | `0.0401` |
| Step @ t = τ | `ct.step_response(plant_tf)` at t ≈ 64.1 s | `0.632 × 0.0401 ≈ 0.02536` |
| Steady-state force | `vp.u0` | `468.8 N` |
| Time constant | `vp.tau` | `64.1 s` |

---

## Phase 2 — Controllers & Linear Closed-Loop Simulation

**Goal:** Build all seven controller TFs, verify closed-loop poles against Section 3, and run the linear simulation pipeline.

**Deliverables:** `sim/controllers.py`, `sim/simulate.py` (linear path)

### 2.1 — `sim/controllers.py` (transfer function constructors)

Build controller TFs for the linear simulation path. Each controller is a function returning `ct.TransferFunction`.

**Seven controller configurations from Section 3 §10:**

| # | Controller | Kp [N/(m/s)] | τ_I [s] | τ_D [s] | λ [s] |
|---|---|---|---|---|---|
| 1 | P (baseline) | 295 | — | — | — |
| 2 | PI IMC moderate | 320 | 64.1 | — | 5 |
| 3 | PI IMC aggressive | 1603 | 64.1 | — | 1 |
| 4 | PI IMC conservative | 160 | 64.1 | — | 10 |
| 5 | PI ZN | 1442 | 3.3 | — | — |
| 6 | PI ITAE | TBD in sim | TBD | — | — |
| 7 | PID IMC | TBD | TBD | TBD | 5 |

Implementation:

```python
import control as ct
from .params import ControllerParams

def build_p_controller(cp: ControllerParams) -> ct.TransferFunction:
    """G_c(s) = Kp"""
    return ct.tf([cp.Kp], [1.0])

def build_pi_controller(cp: ControllerParams) -> ct.TransferFunction:
    """G_c(s) = Kp(τ_I·s + 1) / (τ_I·s)"""
    return ct.tf([cp.Kp * cp.tau_I, cp.Kp], [cp.tau_I, 0.0])

def build_pid_controller(cp: ControllerParams) -> ct.TransferFunction:
    """PID with derivative filter: Kp(1 + 1/(τ_I·s) + τ_D·s/(τ_D/N·s + 1))"""
    pi = build_pi_controller(cp)
    # Filtered derivative: Kp·τ_D·N·s / (τ_D·s + N)
    d_num = [cp.Kp * cp.tau_D * cp.N_filter, 0.0]
    d_den = [cp.tau_D, cp.N_filter]
    derivative = ct.tf(d_num, d_den)
    return pi + derivative
```

### 2.2 — Controller Verification Checklist

| Check | Expected |
|---|---|
| P closed-loop τ_CL | `64.1 / (1 + 295 × 0.0401) ≈ 5.0 s` |
| P closed-loop DC gain | `295 × 0.0401 / (1 + 295 × 0.0401) = 0.922` |
| PI IMC (λ=5) closed-loop poles | Single pole at `s = -0.2` (pole-zero cancellation) |
| PI IMC (λ=5) DC gain | `1.0` (zero steady-state error) |
| PI ZN poles | Two complex conjugate (underdamped) |

### 2.3 — `sim/simulate.py` (linear path)

```python
import numpy as np
import control as ct
from dataclasses import dataclass

@dataclass
class SimResult:
    """Standardized simulation output container."""
    t: np.ndarray          # Time vector [s]
    v: np.ndarray          # Velocity [m/s] (absolute, not deviation)
    u: np.ndarray          # Control force [N] (absolute)
    v_ref: np.ndarray      # Reference velocity [m/s]
    e: np.ndarray          # Error: v_ref - v
    label: str = ""        # Controller/scenario identifier
    model: str = ""        # "linear" or "nonlinear"

def run_linear(plant_tf, controller_tf, t_span, r_func, v0=25.0,
               dt=0.1, label="", d_func=None):
    """
    Linear TF simulation using control.forced_response.

    Args:
        plant_tf: Open-loop plant G(s)
        controller_tf: Controller G_c(s)
        t_span: (t_start, t_end) tuple
        r_func: callable(t) → reference deviation δv_ref
        v0: operating point velocity
        dt: time step for output
        label: identifier string
        d_func: optional disturbance function (for regulatory tests)

    Returns:
        SimResult with absolute (not deviation) quantities.
    """
    t = np.arange(t_span[0], t_span[1], dt)

    # Build closed-loop: T(s) = G_c·G / (1 + G_c·G)
    cl_tf = ct.feedback(controller_tf * plant_tf)

    # Reference input (deviation variable)
    r_dev = np.array([r_func(ti) for ti in t])

    # Compute response
    resp = ct.forced_response(cl_tf, T=t, U=r_dev)

    # Convert deviation → absolute
    v = resp.outputs + v0
    v_ref = r_dev + v0

    # Control effort reconstruction (deviation):
    # u_dev = G_c(s) · (r_dev - v_dev) — approximate via error
    e = v_ref - v

    return SimResult(
        t=t, v=v, u=np.zeros_like(t),  # u computed separately if needed
        v_ref=v_ref, e=e,
        label=label, model="linear"
    )
```

**Note on control effort for linear model:** The control effort can be obtained by simulating the controller TF with the error signal as input via `ct.forced_response(controller_tf, T=t, U=e)`. This is done as a second pass after the closed-loop velocity is known.

**Note on disturbance rejection:** For grade disturbance scenarios, use the disturbance transfer function path: `V(s) = G_d(s)/(1 + G_c·G) · Θ(s)`. Build the disturbance closed-loop as `ct.feedback(plant_tf, controller_tf) * disturbance_gain` or compute via `forced_response` on the sensitivity transfer function.

---

## Phase 3 — Nonlinear ODE Model

**Goal:** Implement the full force-balance ODE with stateful PI controller in the integration loop using `scipy.integrate.solve_ivp`.

**Deliverables:** `sim/plant.py` (nonlinear ODE RHS), `sim/controllers.py` (stateful controller class), `sim/simulate.py` (nonlinear path)

### 3.1 — Nonlinear ODE Right-Hand Side

```python
import numpy as np
from .params import VehicleParams

def nonlinear_rhs(t, y, vp: VehicleParams, controller, v_ref_func, theta_func):
    """
    Full force-balance ODE: m·dv/dt = u - F_drag - F_roll - F_grade

    State vector y = [v, x_i] where:
        v   = vehicle velocity (m/s)
        x_i = controller integrator state (N·s)

    The integrator state is co-integrated with the plant to support
    anti-windup correction inside the ODE loop (Section 4 §7, item 7).
    """
    v, x_i = y
    v = max(v, 0.0)  # physical bound: velocity ≥ 0

    v_ref = v_ref_func(t)
    theta = theta_func(t)
    error = v_ref - v

    # Controller computes force and updated integrator derivative
    u, dx_i = controller.compute(error, x_i, v)

    # Forces
    F_drag = 0.5 * vp.rho * vp.C_d * vp.A * v**2
    F_roll = vp.C_rr * vp.m * vp.g * np.cos(theta)
    F_grade = vp.m * vp.g * np.sin(theta)

    # Newton's second law
    dv_dt = (u - F_drag - F_roll - F_grade) / vp.m

    return [dv_dt, dx_i]
```

### 3.2 — Stateful Controller for ODE Integration

```python
from dataclasses import dataclass
from .params import ControllerParams, VehicleParams

class PIControllerODE:
    """
    Stateful PI controller for nonlinear ODE simulation.

    The integrator state x_i is a co-integrated ODE variable (not internal
    state). This class computes [u, dx_i/dt] given [error, x_i].

    Anti-windup modes:
        "none"   — no anti-windup
        "clamp"  — integrator clamped when output saturates
        "back_calc" — back-calculation with tracking gain K_t = 1/τ_I
    """
    def __init__(self, cp: ControllerParams, vp: VehicleParams,
                 u_min: float = 0.0, u_max: float = 7000.0,
                 anti_windup: str = "back_calc"):
        self.Kp = cp.Kp
        self.tau_I = cp.tau_I
        self.Ki = cp.Kp / cp.tau_I if cp.tau_I else 0.0
        self.u0 = vp.u0           # operating-point bias
        self.u_min = u_min
        self.u_max = u_max
        self.anti_windup = anti_windup
        self.K_t = 1.0 / cp.tau_I if cp.tau_I else 0.0  # tracking gain

    def compute(self, error: float, x_i: float, v: float):
        """
        Returns (u_applied, dx_i_dt).

        u_applied: actual force applied (after saturation)
        dx_i_dt: integrator rate for ODE co-integration
        """
        # Unsaturated output: u₀ + Kp·e + x_i
        u_unsat = self.u0 + self.Kp * error + x_i

        # Saturate
        u_applied = np.clip(u_unsat, self.u_min, self.u_max)

        # Integrator dynamics with anti-windup
        if self.anti_windup == "none":
            dx_i = self.Ki * error
        elif self.anti_windup == "clamp":
            if (u_applied != u_unsat):
                dx_i = 0.0  # freeze integrator during saturation
            else:
                dx_i = self.Ki * error
        elif self.anti_windup == "back_calc":
            dx_i = self.Ki * error + self.K_t * (u_applied - u_unsat)
        else:
            raise ValueError(f"Unknown anti_windup mode: {self.anti_windup}")

        return u_applied, dx_i
```

**Key design decisions (from Section 4 §7):**
- The integrator state `x_i` is a **second ODE variable** co-integrated with velocity. This is the correct approach for anti-windup in an ODE loop — the anti-windup correction modifies `dx_i/dt` at each step.
- The controller output adds the operating-point force `u₀` to the deviation output `Kp·e + x_i`. Omitting `u₀` is the #2 most common implementation bug per Section 4 §7.
- Initial integrator value: `x_i(0) = 0` when starting at steady state, because `u₀` already provides the bias. The total initial output is `u₀ + 0 + 0 = u₀`, which is correct.

### 3.3 — `sim/simulate.py` (nonlinear path)

```python
from scipy.integrate import solve_ivp

def run_nonlinear(vp, controller, t_span, v_ref_func, theta_func,
                  dt=0.1, label=""):
    """
    Nonlinear ODE simulation using scipy.integrate.solve_ivp.

    Uses RK45 with max_step=0.1s to ensure the controller is evaluated
    frequently enough (Section 4 §7, item 5). t_eval provides uniform
    output spacing for metrics/plotting.
    """
    t_eval = np.arange(t_span[0], t_span[1], dt)
    y0 = [vp.v0, 0.0]  # [velocity, integrator_state]

    sol = solve_ivp(
        fun=nonlinear_rhs,
        t_span=t_span,
        y0=y0,
        method='RK45',
        t_eval=t_eval,
        max_step=0.1,       # ensures controller evaluated every ≤0.1 s
        args=(vp, controller, v_ref_func, theta_func),
        dense_output=False,  # not needed; t_eval gives uniform output
    )

    if not sol.success:
        raise RuntimeError(f"ODE solver failed: {sol.message}")

    v = sol.y[0]
    v_ref = np.array([v_ref_func(ti) for ti in sol.t])

    # Reconstruct control effort at each output point
    u = np.zeros_like(sol.t)
    for i, ti in enumerate(sol.t):
        error = v_ref[i] - v[i]
        u[i], _ = controller.compute(error, sol.y[1, i], v[i])

    return SimResult(
        t=sol.t, v=v, u=u,
        v_ref=v_ref, e=v_ref - v,
        label=label, model="nonlinear"
    )
```

### 3.4 — Cross-Model Validation (Scenario S3)

Before proceeding, validate the nonlinear model against the linear model:

| Check | Method | Expected |
|---|---|---|
| 1 m/s step, PI IMC λ=5 | Overlay linear vs. nonlinear | < 1% difference throughout transient |
| Steady-state at v₀ = 25 | Run nonlinear for 200 s with no step | v holds at 25.0 ± 0.01 m/s |
| Force balance residual | Compute `m·dv/dt - (u - F_drag - F_roll)` | < 0.1 N at every output point |

---

## Phase 4 — Metrics Computation

**Goal:** Automated extraction of all performance metrics defined in Section 4 §5.

**Deliverable:** `sim/metrics.py`

### 4.1 — Metric Functions

```python
import numpy as np
from .simulate import SimResult

def rise_time(res: SimResult, t_step: float) -> float:
    """10% → 90% of step magnitude, from step onset."""
    mask = res.t >= t_step
    v = res.v[mask]
    v_init = res.v_ref[mask][0] - (res.v_ref[mask][-1] - res.v[0])  # pre-step value
    v_final = res.v_ref[mask][-1]
    delta = v_final - v_init
    if delta == 0:
        return float('nan')

    t_sub = res.t[mask]
    t_10 = t_sub[np.argmax(v >= v_init + 0.1 * delta)]
    t_90 = t_sub[np.argmax(v >= v_init + 0.9 * delta)]
    return t_90 - t_10

def settling_time(res: SimResult, t_step: float, band: float = 0.02) -> float:
    """Time after which response stays within ±band of final value."""
    mask = res.t >= t_step
    t_sub = res.t[mask]
    v = res.v[mask]
    v_final = np.mean(v[-max(1, len(v)//10):])  # mean of last 10%
    delta = abs(res.v_ref[mask][-1] - res.v[0])
    if delta == 0:
        return float('nan')
    threshold = band * delta
    outside = np.where(np.abs(v - v_final) > threshold)[0]
    if len(outside) == 0:
        return 0.0
    return t_sub[outside[-1]] - t_step

def overshoot(res: SimResult, t_step: float) -> float:
    """Maximum excursion above final value, as % of step magnitude."""
    mask = res.t >= t_step
    v = res.v[mask]
    v_final = res.v_ref[mask][-1]
    v_init = res.v[0]
    delta = v_final - v_init
    if delta == 0:
        return 0.0
    os = (np.max(v) - v_final) / delta * 100.0
    return max(0.0, os)

def steady_state_error(res: SimResult) -> float:
    """(v_ref - v_final) / v_ref × 100%, using mean of last 10%."""
    n = max(1, len(res.v) // 10)
    v_final = np.mean(res.v[-n:])
    v_ref = res.v_ref[-1]
    if v_ref == 0:
        return float('nan')
    return (v_ref - v_final) / v_ref * 100.0

def integral_errors(res: SimResult) -> dict:
    """IAE, ISE, ITAE via numpy.trapezoid (not deprecated np.trapz)."""
    e_abs = np.abs(res.e)
    return {
        "IAE": float(np.trapezoid(e_abs, x=res.t)),
        "ISE": float(np.trapezoid(res.e**2, x=res.t)),
        "ITAE": float(np.trapezoid(res.t * e_abs, x=res.t)),
    }

def peak_disturbance_deviation(res: SimResult, t_dist: float) -> float:
    """Max |v - v_ref| after disturbance onset."""
    mask = res.t >= t_dist
    return float(np.max(np.abs(res.v[mask] - res.v_ref[mask])))

def control_effort_metrics(res: SimResult) -> dict:
    """Peak effort, total variation, saturation fraction."""
    return {
        "peak_control_effort_N": float(np.max(np.abs(res.u))),
        "total_variation_N": float(np.sum(np.abs(np.diff(res.u)))),
        "saturation_fraction": 0.0,  # computed if u_max known
    }
```

### 4.2 — Metrics Aggregation & Export

```python
import json
from pathlib import Path

def compute_all_metrics(res: SimResult, t_step: float,
                        t_dist: float | None = None) -> dict:
    """Compute full metrics dictionary for a single simulation result."""
    m = {
        "scenario": res.label,
        "controller": res.label,
        "model": res.model,
        "metrics": {
            "rise_time_s": rise_time(res, t_step),
            "settling_time_s": settling_time(res, t_step),
            "overshoot_pct": overshoot(res, t_step),
            "ss_error_pct": steady_state_error(res),
            **integral_errors(res),
            **control_effort_metrics(res),
        }
    }
    if t_dist is not None:
        m["metrics"]["peak_deviation_mps"] = peak_disturbance_deviation(res, t_dist)
    return m

def export_metrics(metrics: dict, output_dir: str = "results/metrics"):
    """Export to JSON."""
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    name = f"{metrics['scenario']}_{metrics['model']}.json"
    with open(Path(output_dir) / name, 'w') as f:
        json.dump(metrics, f, indent=2)
```

---

## Phase 5 — Plotting Pipeline

**Goal:** Standardized, publication-quality figures for all 15+ plots defined in Section 4 §6.

**Deliverable:** `sim/plotting.py`

### 5.1 — Global Style Configuration

```python
import matplotlib.pyplot as plt
import matplotlib as mpl

# Global style — applied once at import
STYLE = {
    'figure.figsize': (10, 6),
    'font.size': 12,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.color': '#cccccc',
    'lines.linewidth': 1.8,
}
mpl.rcParams.update(STYLE)

# Controller color/style mapping (Section 4 §6)
CONTROLLER_STYLES = {
    "P_295":            {"color": "gray",   "ls": "-",  "label": "P (Kp=295)"},
    "PI_IMC_lambda5":   {"color": "#1f77b4","ls": "-",  "label": "PI IMC (λ=5)"},
    "PI_IMC_lambda1":   {"color": "#d62728","ls": "-",  "label": "PI IMC (λ=1)"},
    "PI_IMC_lambda10":  {"color": "#2ca02c","ls": "-",  "label": "PI IMC (λ=10)"},
    "PI_ZN":            {"color": "#ff7f0e","ls": "--", "label": "PI ZN"},
    "PI_ITAE":          {"color": "#9467bd","ls": "-.", "label": "PI ITAE"},
    "PID_IMC":          {"color": "#8c564b","ls": "-",  "label": "PID IMC"},
}
```

### 5.2 — Core Plot Functions

**Dual-subplot pattern** (velocity + control effort) — the standard format per Section 4 §6:

```python
from pathlib import Path

def plot_servo_comparison(results: list[SimResult], title: str,
                          filename: str, v_ref_line: float = 30.0):
    """
    Plot B1-style: velocity overlay (top 70%) + control effort (bottom 30%).
    Saves PNG (300 dpi) + PDF to results/plots/.
    """
    fig, (ax_v, ax_u) = plt.subplots(
        2, 1, height_ratios=[7, 3], layout='constrained', figsize=(10, 8)
    )

    for res in results:
        style = CONTROLLER_STYLES.get(res.label, {})
        ax_v.plot(res.t, res.v, color=style.get("color"),
                  ls=style.get("ls", "-"), label=style.get("label", res.label))
        ax_u.plot(res.t, res.u, color=style.get("color"),
                  ls=style.get("ls", "-"))

    # Reference line
    ax_v.axhline(v_ref_line, color='k', ls='--', lw=1.0, label='$v_{ref}$')

    ax_v.set_ylabel("Velocity (m/s)")
    ax_v.set_title(title)
    ax_v.legend(loc='best')

    ax_u.set_xlabel("Time (s)")
    ax_u.set_ylabel("Control Force (N)")

    _save(fig, filename)

def plot_linear_vs_nonlinear(lin_results: list[SimResult],
                              nl_results: list[SimResult],
                              step_sizes: list[str], filename: str):
    """Plot D1-style: 2×2 grid of linear vs. nonlinear at varying step sizes."""
    n = len(lin_results)
    fig, axs = plt.subplots(2, 2, layout='constrained', figsize=(12, 8))

    for i, (lr, nr, label) in enumerate(zip(lin_results, nl_results, step_sizes)):
        ax = axs.flat[i]
        ax.plot(lr.t, lr.v, 'b-', label='Linear')
        ax.plot(nr.t, nr.v, 'r--', label='Nonlinear')
        ax.axhline(lr.v_ref[-1], color='k', ls=':', lw=0.8)
        ax.set_title(f"Step: {label}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend(fontsize=9)

    fig.suptitle("Linear vs. Nonlinear Model Comparison", fontsize=14)
    _save(fig, filename)

def plot_metrics_bar_chart(metrics_list: list[dict], filename: str):
    """Plot F2-style: grouped bar chart for rise time, settling time, OS, IAE."""
    # Implementation: extract metrics from list, group by controller, bar chart
    ...

def _save(fig, filename: str):
    """Save figure as PNG (300 dpi) and PDF."""
    out = Path("results/plots")
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{filename}.png", dpi=300, bbox_inches='tight')
    fig.savefig(out / f"{filename}.pdf", bbox_inches='tight')
    plt.close(fig)
```

### 5.3 — Full Plot Inventory

| Plot ID | Description | Phase |
|---|---|---|
| A1 | Open-loop step response | Phase 2 |
| A2 | Bode plot of open-loop plant | Phase 2 |
| B1 | All-controller servo comparison (S1) | Phase 2 |
| B2 | Error vs. time, all controllers (S1) | Phase 2 |
| B3 | IMC λ-sweep (E3) | Phase 2 |
| B4 | Closed-loop pole map | Phase 2 |
| C1 | All-controller disturbance rejection (D1) | Phase 2 |
| C2 | Grade pulse P vs. PI (D3) | Phase 6 |
| C3 | Steep grade stress test (D2) | Phase 6 |
| D1 | Linear vs. nonlinear, varying steps (E1) | Phase 6 |
| D2 | Small-step validation (S3) | Phase 3 |
| E1 | Mass variation (R1) | Phase 6 |
| E2 | Operating-point variation (R2) | Phase 6 |
| E3 | Drag coefficient variation (R3) | Phase 6 |
| F1 | Anti-windup comparison (E2) | Phase 6 |
| F2 | Metrics summary bar chart | Phase 6 |

---

## Phase 6 — Scenario Execution (Full Test Matrix)

**Goal:** Define and execute all 14 scenarios from Section 4 §4, collect metrics, generate all plots.

**Deliverables:** `sim/scenarios.py`, `run_all.py`, populated `results/` directory

### 6.1 — `sim/scenarios.py`

```python
from dataclasses import dataclass
from typing import Callable

@dataclass
class Scenario:
    name: str
    controller_keys: list[str]       # keys into controller registry
    v_ref_func: Callable             # t → v_ref (absolute, m/s)
    theta_func: Callable             # t → road grade (rad)
    t_span: tuple[float, float]
    model: str                       # "linear", "nonlinear", "both"
    t_step: float = 10.0             # time of set-point change
    t_dist: float | None = None      # time of disturbance onset
    description: str = ""
```

**Scenario definitions (concrete):**

| ID | Name | Step/Dist | Duration | Controllers | Model |
|---|---|---|---|---|---|
| S1 | Moderate step | 25→30 m/s @ t=10 | 80 s | All 7 | both |
| S2 | Large step (saturation) | 15→30 m/s @ t=10 | 120 s | PI IMC λ=5 ±windup | nonlinear |
| S3 | Small step (validation) | 25→26 m/s @ t=10 | 60 s | PI IMC λ=5 | both |
| D1 | Moderate grade | θ: 0→4° @ t=30 | 120 s | All 7 | both |
| D2 | Steep grade | θ: 0→8° @ t=30 | 150 s | PI IMC λ=5 | nonlinear |
| D3 | Grade pulse | θ: 4° [30–80 s] | 160 s | PI IMC λ=5, P | nonlinear |
| R1 | Mass variation | 25→30 step | 80 s | PI IMC λ=5 (fixed) | nonlinear |
| R2 | Speed variation | +3 m/s steps | 80 s | PI IMC λ=5 (fixed) | nonlinear |
| R3 | Drag variation | θ=4° @ t=30 | 120 s | PI IMC λ=5 (fixed) | nonlinear |
| E1 | Linearization divergence | +2,5,10,15 m/s | 80 s | PI IMC λ=5 | both |
| E2 | Anti-windup modes | 15→30 step | 120 s | PI IMC λ=5 × 3 modes | nonlinear |
| E3 | λ-sweep | 25→30 step | 80 s | PI IMC λ∈{1,2,5,10,20} | both |

### 6.2 — `run_all.py`

```python
"""Master script: executes all scenarios in the prescribed phase order."""
from sim.params import VehicleParams, ControllerParams
from sim.plant import build_linear_plant
from sim.controllers import build_pi_controller, build_p_controller
from sim.simulate import run_linear, run_nonlinear
from sim.metrics import compute_all_metrics, export_metrics
from sim.plotting import plot_servo_comparison, plot_linear_vs_nonlinear
from sim.scenarios import build_all_scenarios

def main():
    vp = VehicleParams()

    # ── Phase 1 verification ──
    plant_tf = build_linear_plant(vp)
    verify_plant(plant_tf, vp)

    # ── Phase 2: Linear model battery ──
    controllers = build_all_controllers(vp)
    verify_controllers(controllers, plant_tf)
    run_linear_scenarios(plant_tf, controllers, vp)

    # ── Phase 3: Nonlinear validation ──
    run_cross_validation(vp, controllers)

    # ── Phase 4-6: Full test matrix ──
    run_all_scenarios(vp, controllers)

    # ── Final: metrics summary ──
    compile_summary_table()

if __name__ == "__main__":
    main()
```

### 6.3 — Robustness Scenario Implementation Notes

**R1 (Mass variation):** Construct 5 `VehicleParams` instances with `m ∈ {1200, 1400, 1600, 1800, 2000}`. Controller gains stay fixed at the m=1600 design. Rebuild the nonlinear ODE RHS for each mass — the linearization coefficients change but the controller does not.

**R2 (Operating-point variation):** For each `v₀ ∈ {15, 20, 25, 30, 35}`, set `VehicleParams(v0=v₀)` and recompute `u0` for correct steady-state bias. The controller gains remain at the v₀=25 design values. Initial integrator state is 0 at each respective equilibrium.

**R3 (Drag variation):** Same approach — vary `C_d ∈ {0.24, 0.28, 0.32, 0.36, 0.40}`, hold controller gains fixed.

---

## Phase 7 — Final Compilation & Validation

**Goal:** Regenerate all figures with final styling, export complete metrics, verify against Section 3 predictions.

### 7.1 — Expected Outcomes Verification

| Prediction (Section 3) | How to verify |
|---|---|
| PI IMC (λ=5) meets overshoot < 10%, e_ss = 0% | S1 metrics |
| P controller ≈ 7.8% steady-state error | S1 metrics, P controller |
| ZN PI ≈ 25% overshoot | S1 metrics, PI ZN |
| Anti-windup reduces overshoot for large steps | S2 comparison |
| Linear ≈ nonlinear for ±1 m/s | S3 overlay |
| Linear ≠ nonlinear for ±10 m/s | E1 overlay |
| PI IMC works across 1200–2000 kg mass range | R1 metrics |
| λ=1 may cause saturation on large steps | E3 nonlinear |

### 7.2 — Final Deliverables

1. **`results/plots/`** — 16 figures (PNG 300 dpi + PDF vector), named by plot ID (A1, B1, etc.)
2. **`results/metrics/`** — JSON files for every scenario × controller × model combination
3. **Summary comparison table** — all controllers × {rise time, settling time, overshoot, e_ss, IAE, ITAE} for S1 and D1, printed to console and saved as `results/metrics/summary.json`

### 7.3 — Priority Figures for Report

Per Section 4 §9, in priority order:

1. **B1** — All-controller velocity comparison (main result)
2. **D1** — Linear vs. nonlinear at varying steps (academic highlight)
3. **C1** — All-controller disturbance rejection
4. **B3** — IMC λ-sweep (tuning methodology)
5. **E1** — Mass variation robustness
6. **F1** — Anti-windup demonstration

---

## Execution Summary

| Phase | Scope | Key Risk |
|---|---|---|
| **1** | `params.py`, `plant.py` (linear) | Wrong linearization coefficients propagate everywhere |
| **2** | `controllers.py`, `simulate.py` (linear), plots A1–B4, C1 | Controller TF algebra errors; verify poles |
| **3** | Nonlinear ODE + stateful controller | Sign errors in forces; forgetting u₀ bias; integrator init |
| **4** | `metrics.py` | Edge cases (no overshoot, no crossing); use `np.trapezoid` not `np.trapz` |
| **5** | `plotting.py` | Consistent styling; `layout='constrained'` for all subplots |
| **6** | Full test matrix + robustness | Parameter management for sweeps; correct steady-state init per operating point |
| **7** | Final validation + compilation | Verifying all Section 3 predictions confirmed or explained |

Each phase produces testable intermediate results. **Do not proceed to the next phase until all verification checks for the current phase pass.** This is the single most important discipline — it prevents the cascading-bug failure mode where everything is built at once and errors are impossible to isolate.
