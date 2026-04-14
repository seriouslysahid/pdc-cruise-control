# Python simulation and testing framework for car cruise control

**The controller design in Section 3 produced seven specific controller configurations, seven simulation scenarios, and five quantitative performance specifications — all grounded in the first-order plant G(s) = 0.0401/(64.1s + 1) derived in Section 2.** This section translates that design into a concrete Python simulation framework: the architecture for running both linear transfer-function and nonlinear ODE models, the library stack, the project layout, the complete test matrix, the metrics pipeline, the visualization strategy, and the execution sequence. The framework is designed so that every simulation scenario defined in Section 3 §10 can be executed, measured, and compared in a reproducible, professional manner. All numerical parameters referenced below are inherited directly from Sections 2 and 3 unless otherwise noted.

---

## 1. Simulation architecture recommendation

### Dual-model simulation strategy

The simulation framework must support **two distinct model representations** running in parallel, because they answer different questions and cross-validate each other:

| Model layer | Representation | Purpose | When to use |
|---|---|---|---|
| **Linear transfer-function model** | G(s) = K/(τs + 1) via `python-control` | Controller tuning verification, root locus, Bode, analytic benchmarks | First — validates controller math against closed-form predictions |
| **Nonlinear ODE model** | m(dv/dt) = u − ½ρC_dAv² − C_rrmg − mg sin θ via `scipy.integrate` | Physical fidelity, actuator saturation, anti-windup, large-perturbation tests | Second — validates that the linearized design survives real physics |

The linear model provides **analytic ground truth** for small-perturbation behavior: if the linear simulation disagrees with the closed-form predictions from Section 3 (e.g., τ_CL = 5 s, zero overshoot for IMC with pole-zero cancellation), there is a code bug. The nonlinear model provides **physical ground truth** for large perturbations, saturation, and operating-point shifts where linearization breaks down.

### Recommended simulation workflow

```
1. Linear TF model  ─→  Verify controller math against closed-form
2. Linear TF model  ─→  Run all controller comparison scenarios
3. Nonlinear ODE     ─→  Repeat key scenarios; compare with linear results
4. Nonlinear ODE     ─→  Run saturation / anti-windup / robustness scenarios
5. Combined plots    ─→  Overlay linear vs. nonlinear for same controller
```

### Implementation order

The framework should be built incrementally in this order:

1. **Plant parameters module** — centralize all physical constants from Section 2
2. **Linear plant construction** — build G(s) using `python-control`; verify open-loop step response against τ = 64.1 s, K = 0.0401
3. **Controller construction** — build P, PI, PID transfer functions; verify closed-loop poles against Section 3 predictions
4. **Linear closed-loop simulation** — step responses, disturbance responses; compute metrics
5. **Nonlinear ODE model** — implement the full force-balance ODE with the PI controller in the integration loop
6. **Saturation and anti-windup logic** — add throttle clamping and back-calculation to the nonlinear simulation
7. **Metrics computation** — automated extraction of rise time, overshoot, settling time, steady-state error, integral error criteria
8. **Plotting pipeline** — standardized comparative figures
9. **Robustness and edge-case scenarios** — parameter sweeps, large perturbations

Each stage produces testable intermediate results before the next layer of complexity is added. This prevents the common failure mode of building everything at once and being unable to isolate bugs.

---

## 2. Python toolchain and libraries

### Core libraries

| Library | Version | Role | Why this library |
|---|---|---|---|
| **`python-control`** | ≥ 0.10.0 | Transfer function creation, closed-loop algebra, step/impulse response, root locus, Bode plots | Purpose-built for control systems; mirrors MATLAB's Control System Toolbox; directly handles `tf()`, `feedback()`, `step_response()`, `bode_plot()` |
| **`scipy`** | ≥ 1.11 | ODE integration (`solve_ivp`), signal processing, optimization | Industry-standard scientific computing; `solve_ivp` with RK45/BDF provides robust adaptive-step ODE solving for both stiff and non-stiff systems |
| **`numpy`** | ≥ 1.24 | Array operations, linear algebra, numerical utilities | Foundational; required by all other scientific libraries |
| **`matplotlib`** | ≥ 3.8 | All plotting — time responses, Bode, root locus, comparison figures | Most flexible Python plotting library; publication-quality output; full control over layout, annotations, and styling |

### Supporting libraries

| Library | Role | Notes |
|---|---|---|
| **`dataclasses`** (stdlib) | Structured parameter containers | Clean, type-hinted parameter passing; avoids magic dictionaries |
| **`pathlib`** (stdlib) | File path management | Cross-platform output directory handling |
| **`json`** (stdlib) | Metrics export | Machine-readable results for post-processing |

### Libraries explicitly NOT recommended

| Library | Why not |
|---|---|
| **`slycot`** | Optional backend for `python-control`; not needed for SISO systems; installation is problematic on Windows |
| **`control.matlab`** | MATLAB-compatibility shim in `python-control`; use the native API instead for cleaner code |
| **`sympy`** | Symbolic math is unnecessary — all transfer functions have known numerical coefficients from Section 2 |
| **`simulink` / `simscape`** | MATLAB ecosystem; this is a Python-first project |

### Installation

A single command installs the entire toolchain:

```
pip install numpy scipy matplotlib control
```

The `control` package is the PyPI name for `python-control`. No conda environment or special build tools are needed.

---

## 3. Code structure and project architecture

### Recommended file layout

```
pdc-cruise-control/
│
├── 01_literature_and_problem_framing.md
├── 02_mathematical_modeling_and_derivation.md
├── 03_controller_design_and_tuning.md
├── 04_python_simulation_and_testing_framework.md
│
├── sim/                              # ← All simulation code
│   ├── __init__.py
│   ├── params.py                     # Plant and controller parameters
│   ├── plant.py                      # Linear TF and nonlinear ODE models
│   ├── controllers.py                # P / PI / PID construction
│   ├── simulate.py                   # Simulation runner (linear + nonlinear)
│   ├── metrics.py                    # Performance metric extraction
│   ├── plotting.py                   # All visualization functions
│   └── scenarios.py                  # Scenario definitions and test matrix
│
├── run_all.py                        # Master script: executes all scenarios
├── results/                          # Generated plots and metrics (gitignored)
│   ├── plots/
│   └── metrics/
│
└── requirements.txt                  # pip dependencies
```

### Module responsibilities

**`params.py`** — Single source of truth for all numerical values. Every physical constant from Section 2 and every controller gain from Section 3 lives here. No magic numbers anywhere else in the codebase.

```python
# Conceptual structure (not exhaustive)
@dataclass
class VehicleParams:
    m: float = 1600.0       # kg
    C_d: float = 0.32       # dimensionless
    A: float = 2.4           # m²
    rho: float = 1.3         # kg/m³
    C_rr: float = 0.01       # dimensionless
    g: float = 9.8           # m/s²
    v0: float = 25.0         # m/s, operating point

@dataclass
class ControllerParams:
    Kp: float
    tau_I: float | None = None   # None → P-only
    tau_D: float | None = None   # None → PI-only
    N_filter: float = 10.0       # Derivative filter coefficient
    label: str = ""              # Human-readable identifier
```

**`plant.py`** — Constructs both model representations from the same parameter set. The linear model returns a `control.TransferFunction` object. The nonlinear model returns a callable ODE right-hand-side function `f(t, v, u, theta)` compatible with `scipy.integrate.solve_ivp`.

Key design principle: **both models must be constructable from the same `VehicleParams` instance**, ensuring parameter consistency. The linearization coefficients (a, b, K, τ) are computed from the physical parameters, not hardcoded separately.

**`controllers.py`** — Builds controller transfer functions for the linear simulation path and implements the controller as a stateful class for the nonlinear simulation path (where the integrator state must be tracked explicitly for anti-windup).

The stateful controller class for nonlinear simulation should track:
- Integral accumulator (for integral action)
- Previous error or PV (for derivative action)
- Saturation status (for anti-windup logic)

**`simulate.py`** — Orchestrates simulation runs. Two core functions:

1. `run_linear(plant_tf, controller_tf, t_span, r_func, d_func)` — uses `control.forced_response()` or `control.step_response()` for the linear TF model
2. `run_nonlinear(vehicle_params, controller, t_span, v_ref_func, theta_func, u_limits)` — uses `scipy.integrate.solve_ivp()` with the controller evaluated at each integration step

Both return a standardized result object containing time vector, velocity, control effort, error, and reference signal.

**`metrics.py`** — Post-processes simulation results to extract quantitative metrics. All metric functions take the standardized result object and return numerical values. Documented definitions ensure consistency with Section 3 specifications.

**`plotting.py`** — Every plot function takes one or more simulation result objects and produces a figure. Standardized styling (font sizes, colors, grid, legend placement) ensures visual consistency across the project.

**`scenarios.py`** — Defines the complete test matrix as a list of scenario objects, each specifying: controller configuration, reference signal profile, disturbance profile, simulation duration, and which model (linear/nonlinear/both) to use.

### Engineering coding practices

1. **No magic numbers.** Every physical constant and controller gain traces back to `params.py`, which traces back to Section 2 and Section 3.

2. **Reproducible results.** Each `run_all.py` execution produces identical outputs. No random seeds are involved (deterministic ODEs), but if noise is added later, seed it explicitly.

3. **Separation of computation and presentation.** Simulation functions return data; plotting functions consume data. Never mix computation with plotting — this enables batch execution followed by selective visualization.

4. **Self-documenting parameter sets.** Each controller configuration carries a `label` string that appears automatically in plots and metrics tables. No manual label management.

5. **Incremental validation.** Each module has a natural "smoke test": `plant.py` can print the open-loop time constant and verify it against 64.1 s; `controllers.py` can print closed-loop poles and verify against Section 3 formulas.

---

## 4. Simulation scenarios

All scenarios below are inherited from Section 3 §10 with implementation-level detail added. Times, magnitudes, and parameter ranges are specified concretely.

### 4.1 Set-point tracking (servo test)

#### Scenario S1: Moderate step change

| Parameter | Value |
|---|---|
| **Initial velocity** | v₀ = 25 m/s (steady state) |
| **Set-point step** | v_ref steps from 25 → 30 m/s at t = 10 s |
| **Road grade** | θ = 0 (flat road) |
| **Duration** | 80 s (sufficient for ~10× the closed-loop time constant) |
| **Controllers** | All seven from Section 3 §10 table |
| **Model** | Linear TF first, then nonlinear ODE |

**Purpose:** Primary servo performance benchmark. The 5 m/s (20%) step is large enough to test controller authority but small enough that linearization should remain accurate. Measures rise time, overshoot, settling time, and steady-state error — the four CTMS specification metrics.

**Expected behavior:**
- P (K_p = 295): First-order response, τ_CL ≈ 5 s, ~8% steady-state error, zero overshoot
- PI IMC (λ = 5): First-order response (pole-zero cancellation), τ_CL ≈ 5 s, zero steady-state error, zero overshoot
- PI ZN: Oscillatory response, ~25% overshoot, fast rise time, zero steady-state error eventually
- PI IMC aggressive (λ = 1): Very fast response, τ_CL ≈ 1 s, zero steady-state error, possible actuator saturation in nonlinear model

#### Scenario S2: Large step change (saturation-probing)

| Parameter | Value |
|---|---|
| **Set-point step** | v_ref steps from 15 → 30 m/s at t = 10 s |
| **Duration** | 120 s |
| **Controllers** | PI IMC (λ = 5) with and without anti-windup |
| **Model** | Nonlinear ODE only (linearization invalid for 15 m/s perturbation) |

**Purpose:** Tests actuator saturation and integrator windup. The 15 m/s step far exceeds the linearization range and will saturate the throttle. Compares behavior with and without back-calculation anti-windup.

**Expected behavior:**
- Without anti-windup: Significant overshoot (integrator winds up during saturation period)
- With anti-windup: Moderate or no overshoot (integrator is clamped/reset during saturation)

#### Scenario S3: Small step change (linearization validation)

| Parameter | Value |
|---|---|
| **Set-point step** | v_ref steps from 25 → 26 m/s at t = 10 s |
| **Duration** | 60 s |
| **Controllers** | PI IMC (λ = 5) |
| **Model** | Both linear and nonlinear, overlaid |

**Purpose:** Validates that the linear and nonlinear models produce essentially identical results for small perturbations. The 1 m/s step (4% of operating speed) should be well within the linearization accuracy.

**Expected behavior:** Linear and nonlinear responses nearly indistinguishable (< 1% difference), confirming linearization validity.

### 4.2 Disturbance rejection (regulatory test)

#### Scenario D1: Moderate grade disturbance

| Parameter | Value |
|---|---|
| **Set-point** | v_ref = 25 m/s (constant) |
| **Grade step** | θ steps from 0 → 4° (0.0698 rad) at t = 30 s |
| **Duration** | 120 s |
| **Controllers** | All seven |
| **Model** | Both |

**Purpose:** Tests the core operational scenario — maintaining speed on a highway incline. The 4° grade is a moderately steep highway hill (7.0% grade). The Section 3 specification requires < 5% peak velocity deviation (i.e., < 1.25 m/s drop).

**Expected behavior:**
- P: Permanent velocity drop of ~3.4 m/s (13.6% offset) — fails specification
- PI IMC (λ = 5): Transient drop, full recovery to 25 m/s; peak deviation ~3.4 m/s (exceeds 5% spec transiently), recovery within ~25 s
- PI IMC aggressive (λ = 1): Smaller peak deviation, faster recovery
- In nonlinear model: Grade applies mg sin θ = 1600 × 9.8 × sin(4°) ≈ 1094 N of opposing force — approximately 2.3× the steady-state cruising force

#### Scenario D2: Steep grade disturbance

| Parameter | Value |
|---|---|
| **Grade step** | θ steps from 0 → 8° (0.1396 rad) at t = 30 s |
| **Duration** | 150 s |
| **Controllers** | PI IMC (λ = 5) |
| **Model** | Nonlinear ODE only |

**Purpose:** Stress-tests disturbance rejection at a grade that is extreme for highway driving. The disturbance force is ~2190 N. This tests whether the controller can recover without throttle saturation and how far the linearized controller degrades on a nonlinear model under large disturbance.

#### Scenario D3: Grade pulse (hill and valley)

| Parameter | Value |
|---|---|
| **Grade profile** | θ = 0 → 4° at t = 30 s, returns to θ = 0° at t = 80 s |
| **Duration** | 160 s |
| **Controllers** | PI IMC (λ = 5), P (K_p = 295) |
| **Model** | Nonlinear ODE |

**Purpose:** Tests the realistic scenario of climbing then cresting a hill. The controller must reject the grade onset, then handle the grade removal without excessive overshoot. The P controller's offset in both directions (below on uphill, above on downhill) versus PI's full rejection provides a strong visual comparison.

### 4.3 Robustness and sensitivity

#### Scenario R1: Mass variation

| Parameter | Value |
|---|---|
| **Plant masses** | m ∈ {1200, 1400, 1600, 1800, 2000} kg |
| **Controller** | PI IMC (λ = 5) tuned for m = 1600 kg (K_p = 320, τ_I = 64.1) — **not retuned** |
| **Test** | Set-point step 25 → 30 m/s |
| **Model** | Nonlinear ODE |

**Purpose:** Tests robustness to the most common parameter uncertainty — vehicle mass varies with passengers and cargo. The controller gains are held fixed at the design values; only the plant mass changes.

**Expected behavior:**
- At m = 1200 kg (lighter): Faster response than designed, possible overshoot (effective K_pK is higher because K ∝ 1/b_eff and b_eff is independent of m, but τ = m/b_eff is shorter);
- At m = 2000 kg (heavier): Slower response, no overshoot, longer settling time
- The integral action should ensure zero steady-state error at all masses — this is a key strength of PI control to demonstrate

#### Scenario R2: Operating-point variation

| Parameter | Value |
|---|---|
| **Operating speeds** | v₀ ∈ {15, 20, 25, 30, 35} m/s |
| **Controller** | PI IMC (λ = 5) tuned at v₀ = 25 m/s — **not retuned** |
| **Test** | +3 m/s step from each operating point |
| **Model** | Nonlinear ODE |

**Purpose:** The linearized gain K and time constant τ both depend on v₀ (K ∝ 1/v₀, τ ∝ 1/v₀). A controller tuned at 25 m/s will have mismatched gains at other speeds. This scenario quantifies how severely the performance degrades.

**Expected behavior:**
- At v₀ = 15 m/s: Higher effective gain, faster/more oscillatory response (plant is "easier" to move)
- At v₀ = 35 m/s: Lower effective gain, sluggish response (stronger aerodynamic damping)
- Integral action maintains zero steady-state error at all speeds, compensating for gain mismatch

#### Scenario R3: Drag coefficient uncertainty

| Parameter | Value |
|---|---|
| **Drag coefficients** | C_d ∈ {0.24, 0.28, 0.32, 0.36, 0.40} |
| **Controller** | PI IMC (λ = 5) tuned at C_d = 0.32 |
| **Test** | Grade disturbance θ = 4° |
| **Model** | Nonlinear ODE |

**Purpose:** Tests sensitivity to aerodynamic parameter uncertainty. C_d varies with vehicle modifications (roof rack, open windows, dirty surface) and is the parameter most likely to differ from the nominal design value.

### 4.4 Edge cases and nonlinear stress tests

#### Scenario E1: Linear vs. nonlinear divergence test

| Parameter | Value |
|---|---|
| **Set-point steps** | +2 m/s, +5 m/s, +10 m/s, +15 m/s from v₀ = 25 m/s |
| **Controller** | PI IMC (λ = 5) |
| **Model** | Both linear and nonlinear for each step size |

**Purpose:** Systematically demonstrates where the linearization breaks down. By overlaying linear and nonlinear responses at increasing perturbation sizes, the plot will show:
- Near-perfect agreement at +2 m/s
- Minor divergence at +5 m/s
- Visible divergence at +10 m/s
- Qualitative disagreement at +15 m/s (linearization invalid)

This is the **most academically valuable plot** in the project — it directly demonstrates the linearization accuracy and its limits.

#### Scenario E2: Actuator limits and anti-windup

| Parameter | Value |
|---|---|
| **Throttle limits** | u ∈ [0, u_max] where u_max corresponds to maximum engine force |
| **Test** | Set-point step 15 → 30 m/s with PI (λ = 5) |
| **Anti-windup modes** | None, clamping, back-calculation |
| **Model** | Nonlinear ODE |

**Purpose:** Demonstrates integrator windup and the effectiveness of anti-windup strategies. Three runs overlaid on the same plot show the dramatically different transient behaviors.

#### Scenario E3: Tuning aggressiveness sweep

| Parameter | Value |
|---|---|
| **λ values** | {1, 2, 5, 10, 20} s |
| **Controller** | PI IMC (K_p = τ/(Kλ), τ_I = τ) for each λ |
| **Test** | Set-point step 25 → 30 m/s |
| **Model** | Both |

**Purpose:** Produces the signature IMC tuning tradeoff plot — a family of curves showing faster response (smaller λ) at the cost of larger control effort and reduced robustness. This single figure demonstrates mastery of the IMC design philosophy.

---

## 5. Performance metrics framework

### Metric definitions

All metrics are computed from the simulation output arrays `t` (time), `v` (velocity), `v_ref` (reference), `u` (control effort), and `e = v_ref − v` (error).

#### Rise time (t_r)

**Definition:** Time for the response to go from 10% to 90% of the step magnitude, measured from the step onset.

**Computation:**
```
delta = v_ref_final - v_initial
t_10 = first time where v(t) ≥ v_initial + 0.1 * delta
t_90 = first time where v(t) ≥ v_initial + 0.9 * delta
t_r = t_90 - t_10
```

**Target:** < 5 s (Section 3 specification)

#### Settling time (t_s)

**Definition:** Time after which the response remains within ±2% of the final steady-state value, measured from the step onset.

**Computation:**
```
band = 0.02 * |v_ref_final - v_initial|
t_s = last time where |v(t) - v_final| > band (using final value, 
      not reference, to handle P controller's persistent offset)
```

For zero-offset controllers (PI, PID), v_final = v_ref_final. For P controller, v_final ≈ v_ref × K_CL.

**Target:** < 15 s

#### Overshoot (OS)

**Definition:** Maximum excursion above the final value, expressed as a percentage of the step magnitude.

**Computation:**
```
OS = max(0, (max(v) - v_ref_final) / (v_ref_final - v_initial)) × 100%
```

**Target:** < 10%

#### Steady-state error (e_ss)

**Definition:** Difference between the reference and the final (settled) velocity, expressed as percentage of the reference.

**Computation:**
```
v_final = mean(v[-N:]) where N covers the last 10% of the simulation
e_ss = (v_ref_final - v_final) / v_ref_final × 100%
```

Using the mean of the last portion avoids sensitivity to residual oscillations.

**Target:** < 2% (ideally 0%)

#### Integral error criteria

Three integral performance indices provide single-number measures of overall tracking quality:

| Metric | Formula | Emphasis |
|---|---|---|
| **IAE** (Integral Absolute Error) | ∫₀^T \|e(t)\| dt | Overall error magnitude |
| **ISE** (Integral Squared Error) | ∫₀^T e²(t) dt | Penalizes large errors more heavily |
| **ITAE** (Integral Time-weighted Absolute Error) | ∫₀^T t·\|e(t)\| dt | Penalizes persistent (late) errors; rewards fast settling |

**Computation:** Numerical integration via `numpy.trapz()` (trapezoidal rule) over the simulation time vector. These are computed for both servo and regulatory scenarios.

**Purpose:** ITAE is the most physically meaningful for cruise control — a long-duration speed deviation on a hill is worse than a brief transient during a set-point change — and provides the objective function that ITAE tuning minimizes.

#### Peak disturbance deviation

**Definition:** Maximum velocity drop (or rise) from set-point during a disturbance event.

**Computation:**
```
peak_dev = max(|v(t) - v_ref|) for t ≥ t_disturbance
```

**Target:** < 5% of v_ref for a 4° grade

#### Control effort metrics

| Metric | Formula | Purpose |
|---|---|---|
| **Peak control effort** | max(\|u(t)\|) | Checks feasibility against actuator limits |
| **Total variation of control** | Σ\|u(t_i+1) − u(t_i)\| | Measures actuator wear / smoothness; penalizes chattering |
| **Saturation fraction** | Fraction of time u is at its limit | Quantifies how constrained the controller is |

### Metrics output format

All metrics for each scenario should be collected into a structured dictionary and exported as both:
1. A formatted console table (for quick inspection during development)
2. A JSON file in `results/metrics/` (for programmatic comparison and inclusion in reports)

Example output structure:
```json
{
  "scenario": "S1_step_25_to_30",
  "controller": "PI_IMC_lambda5",
  "model": "nonlinear",
  "metrics": {
    "rise_time_s": 4.8,
    "settling_time_s": 12.3,
    "overshoot_pct": 0.0,
    "ss_error_pct": 0.002,
    "IAE": 15.2,
    "ISE": 8.7,
    "ITAE": 42.1,
    "peak_control_effort_N": 1850,
    "saturation_fraction": 0.0
  }
}
```

---

## 6. Plotting and visualization strategy

### Required plots

The simulation should generate the following figures, organized by purpose:

#### Category A: Open-loop characterization (2 plots)

**A1. Open-loop step response.** Step force input to the plant (no controller). Verifies τ = 64.1 s and K = 0.0401.

**A2. Bode plot of open-loop plant.** Magnitude and phase vs. frequency. Shows the −3 dB bandwidth at ω = 1/τ = 0.0156 rad/s and the −90° phase asymptote.

#### Category B: Controller comparison — servo (3–4 plots)

**B1. Velocity vs. time overlay — all controllers, step S1.** Single figure with 4–7 curves (one per controller), each labeled. The most important comparative figure. Subplot below showing control effort u(t) for each controller.

**B2. Error vs. time overlay — all controllers, step S1.** Highlights steady-state error differences (P offset visible; PI/PID converge to zero).

**B3. IMC tuning sweep — λ variation (Scenario E3).** Family of velocity curves for λ ∈ {1, 2, 5, 10, 20}. Companion subplot showing control effort. Demonstrates the speed–effort tradeoff.

**B4. Root locus or closed-loop pole map.** Pole locations for each controller configuration, plotted in the s-plane. Verifies stability and shows how poles move with K_p.

#### Category C: Controller comparison — disturbance rejection (2–3 plots)

**C1. Velocity vs. time overlay — all controllers, grade D1.** Shows P controller's permanent offset vs. PI's recovery. Subplot shows control effort (throttle increase to climb hill).

**C2. Grade pulse response — P vs. PI (Scenario D3).** Hill-and-valley scenario; illustrates integral action's superiority over the full disturbance cycle.

**C3. Steep grade stress test (Scenario D2).** Single controller under heavy disturbance; shows controller limits.

#### Category D: Linearization and model validation (2 plots)

**D1. Linear vs. nonlinear overlay — varying step sizes (Scenario E1).** 2×2 or 4×1 subplot grid, each showing linear and nonlinear for one step size. The key academic-value figure.

**D2. Small-step validation (Scenario S3).** Linear and nonlinear overlaid for a 1 m/s step; should be nearly identical.

#### Category E: Robustness (2–3 plots)

**E1. Mass variation — velocity responses overlaid (Scenario R1).** Five curves for m ∈ {1200, ..., 2000} kg, same controller, same step input.

**E2. Operating-point variation (Scenario R2).** Five curves for v₀ ∈ {15, ..., 35} m/s.

**E3. Drag coefficient variation (Scenario R3).** Five curves for C_d sweep under grade disturbance.

#### Category F: Practical considerations (1–2 plots)

**F1. Anti-windup comparison (Scenario E2).** Three curves: no anti-windup, clamping, back-calculation. Subplot shows control effort with saturation visible.

**F2. Metrics summary bar chart.** Grouped bar chart comparing rise time, settling time, overshoot, and IAE across all controllers for Scenario S1. Provides a single at-a-glance performance comparison.

### Plotting style guidelines

**Consistent styling across all figures:**

- **Figure size:** 10 × 6 inches (single plot) or 10 × 8 inches (dual subplot with velocity + control effort)
- **Font:** 12 pt for axis labels, 10 pt for tick labels, 11 pt for legend
- **Colors:** Use a qualitative colormap with sufficient contrast. Suggested assignment:
  - P: gray (baseline)
  - PI IMC (λ=5): blue (primary design)
  - PI IMC (λ=1): red (aggressive)
  - PI IMC (λ=10): green (conservative)
  - PI ZN: orange dashed (cautionary example)
  - PID: purple (comparison)
- **Line styles:** Solid for primary controllers; dashed for secondary/comparison; dotted for reference signals
- **Grid:** Light gray grid on all axes for readability
- **Reference signal:** Horizontal dashed black line showing v_ref or the set-point; always present on velocity plots
- **Annotations:** Mark rise time, overshoot, and settling time on at least one representative plot (B1) with arrows and text callouts
- **Axis labels:** Always include units. "Velocity (m/s)", "Time (s)", "Control Force (N)", "Road Grade (degrees)"
- **Legend placement:** Outside the plot area (upper right or below) when more than 4 curves; inside when ≤ 4 curves

**Subplot convention for servo/regulatory plots:**

```
┌─────────────────────────────┐
│  Velocity v(t)  vs. time    │   (top, 70% height)
│  with v_ref dashed          │
├─────────────────────────────┤
│  Control effort u(t) vs t   │   (bottom, 30% height)
│  with saturation limits     │
└─────────────────────────────┘
```

This dual-subplot format is the standard in control systems literature and shows both the output quality and the cost (actuator effort) simultaneously.

**File format:** Save all figures as both PNG (300 dpi, for reports) and PDF (vector, for presentations) to `results/plots/`.

---

## 7. Validation and debugging strategy

### Layer 1: Analytic verification (before running scenarios)

Before running any scenario simulation, verify each module independently against known analytic results from Section 2 and Section 3.

**Plant verification checks:**

| Check | Expected result | How to verify |
|---|---|---|
| Open-loop poles | s = −0.01561 | `plant_tf.poles()` should return `[-0.01561]` |
| DC gain | 0.0401 | `control.dcgain(plant_tf)` should return `0.0401` |
| Step response at t = τ | 63.2% of final value | `control.step_response(plant_tf)` at t = 64.1 s ≈ 0.0254 (= 0.632 × 0.0401) |
| Steady-state force | 468.8 N | u₀ = ½ρC_dAv₀² + C_rr·m·g = 312.0 + 156.8 |

**Controller verification checks:**

| Check | Expected result |
|---|---|
| P closed-loop time constant | τ/(1 + K_pK) = 64.1/(1 + 295 × 0.0401) ≈ 5.0 s |
| P closed-loop DC gain | K_pK/(1 + K_pK) = 0.922 |
| PI IMC (λ=5) closed-loop poles | Single pole at s = −1/λ = −0.2 (after pole-zero cancellation) |
| PI IMC closed-loop DC gain | 1.0 (zero steady-state error) |
| PI ZN closed-loop poles | Two complex conjugate poles (underdamped) |

### Layer 2: Simulation sanity checks

**Energy/momentum plausibility:** In the nonlinear simulation, verify at every timestep that m·(dv/dt) ≈ u − F_drag − F_roll − F_grade by computing the residual. A large residual indicates an ODE solver error or incorrect force computation.

**Steady-state consistency:** After sufficient time, the nonlinear simulation's velocity should match the equilibrium velocity obtained by solving u = ½ρC_dAv_ss² + C_rr·m·g analytically. For the closed-loop with PI control, v_ss should equal v_ref exactly (since integral action eliminates offset).

**Conservation check:** For a step set-point change with no disturbance, the total impulse delivered by the control force (∫u·dt minus steady-state force × time) should equal the change in momentum m·Δv. Computing this integral numerically and comparing with m × 5 (for a 5 m/s step with m = 1600 kg → 8000 N·s) provides a strong end-to-end validation.

### Layer 3: Cross-model validation

**Small-perturbation agreement:** For a 1 m/s step (Scenario S3), the linear and nonlinear responses should agree to within 1% throughout the transient. If they disagree significantly, the nonlinear ODE implementation or the linearization coefficients have an error.

**Large-perturbation divergence direction:** For a 10 m/s step upward (from 25 to 35 m/s), the nonlinear model should settle *slightly below* the reference compared to the linear model, because the increased drag at higher speed reduces the effective gain (K decreases with v₀). If the nonlinear response is *higher* than linear, the drag term sign or magnitude is wrong.

### Common implementation mistakes to watch for

1. **Sign error in disturbance force.** The grade force m·g·sin(θ) *opposes* motion for positive θ (uphill). A sign error makes the car speed up on uphills — immediately obvious but sometimes missed.

2. **Forgetting to subtract the operating-point force in the controller.** The PI controller output δu is a *deviation* from u₀. In the nonlinear simulation, the actual applied force is u = u₀ + δu. Omitting u₀ means the controller starts from zero force instead of the cruising force, causing the car to decelerate instantly at t = 0.

3. **Using the wrong time constant in `step_response`.** The `python-control` step response applies a unit step. For a 5 m/s set-point change, multiply the response by 5. Alternatively, use `forced_response` with an explicit input signal at the correct magnitude.

4. **Integrator initialization.** At t = 0, the integrator state should be initialized such that the total controller output equals u₀ (the steady-state force maintaining v₀). This means the initial integrator value is u₀/K_i (or equivalently, the integrator contains the bias needed to maintain the operating point). Initializing the integrator to zero is the most common cause of initial transients in simulated PI controllers.

5. **ODE solver step size for `solve_ivp`.** The default `RK45` method with adaptive stepping handles this system well (it is not stiff), but `max_step` should be set to ~0.1 s to ensure the controller output is evaluated frequently enough. Without `max_step`, the adaptive solver might take very large steps during slowly-varying portions, missing the effect of sudden disturbance onsets.

6. **Confusing deviation and absolute variables.** The linear TF model operates on deviation variables δv and δu. The nonlinear model operates on absolute variables v and u. When comparing outputs, ensure that the linear model's δv is shifted by v₀ to give absolute velocity.

7. **Anti-windup in the ODE loop.** The anti-windup correction modifies the integrator state at each timestep. This means the ODE integration cannot treat the controller as a simple algebraic function — the integrator state is an additional state variable that must evolve alongside the plant state. The recommended approach is to include the integrator state as a second ODE variable: `y = [v, x_i]` where `x_i` is the integrator state, and the anti-windup term appears in `dx_i/dt`.

---

## 8. Recommended simulation execution sequence

### Phase 1: Foundation verification (30 minutes)

**Goal:** Confirm that the plant model and controller construction are correct before running any scenario.

| Step | Action | Verification |
|---|---|---|
| 1.1 | Build `VehicleParams` from Section 2 values | Print and compare against tabulated values |
| 1.2 | Construct plant TF: G(s) = K/(τs + 1) | Check poles, DC gain against Section 2 |
| 1.3 | Compute open-loop step response | Plot and verify τ ≈ 64 s, K ≈ 0.040 |
| 1.4 | Construct all controller TFs (P, PI variants, PID) | Print closed-loop poles; compare with Section 3 |
| 1.5 | Compute closed-loop step response for PI IMC (λ=5) | Verify τ_CL = 5 s, zero overshoot, zero e_ss |

If any check fails, **stop and debug** before proceeding. Errors here propagate into every subsequent simulation.

### Phase 2: Linear model — complete scenario battery (1 hour)

**Goal:** Generate the full comparison dataset using the fast, reliable linear model.

| Step | Scenario | Key output |
|---|---|---|
| 2.1 | S1: Step 25 → 30, all controllers | Plots B1, B2; metrics table |
| 2.2 | D1: Grade 4°, all controllers | Plot C1; metrics table |
| 2.3 | E3: λ sweep {1,2,5,10,20} | Plot B3 |
| 2.4 | Bode plot of open-loop plant | Plot A2 |
| 2.5 | Closed-loop pole map | Plot B4 |
| 2.6 | Compute all metrics for S1 and D1 | Metrics JSON; plot F2 |

At this point, the linear simulation is complete. All controller comparison results are available, and the metrics should match Section 3 predictions.

### Phase 3: Nonlinear model — validation and stress (1.5 hours)

**Goal:** Verify that the linearized design works on the physical model and explore nonlinear phenomena.

| Step | Scenario | Key output |
|---|---|---|
| 3.1 | S3: Small step (1 m/s), linear vs. nonlinear | Plot D2 (should overlap) |
| 3.2 | S1: Step 25 → 30, PI IMC (λ=5), linear vs. nonlinear | Quantify divergence |
| 3.3 | E1: Varying step sizes, linear vs. nonlinear | Plot D1 (key academic figure) |
| 3.4 | D1: Grade 4°, PI IMC (λ=5), nonlinear | Compare with linear result from Phase 2 |
| 3.5 | D3: Grade pulse, P vs. PI, nonlinear | Plot C2 |
| 3.6 | D2: Steep grade (8°), nonlinear | Plot C3 |

### Phase 4: Practical considerations (1 hour)

**Goal:** Demonstrate real-world implementation concerns.

| Step | Scenario | Key output |
|---|---|---|
| 4.1 | S2: Large step (15 → 30), with/without anti-windup | Plot F1 |
| 4.2 | E2: Anti-windup comparison (3 modes) | Overlay plot |
| 4.3 | R1: Mass variation, step response | Plot E1 |
| 4.4 | R2: Operating-point variation | Plot E2 |
| 4.5 | R3: Drag coefficient variation under grade | Plot E3 |

### Phase 5: Final compilation (30 minutes)

**Goal:** Aggregate all results into the final output set.

| Step | Action |
|---|---|
| 5.1 | Regenerate all plots with final styling |
| 5.2 | Export complete metrics tables (all scenarios × all controllers) |
| 5.3 | Generate summary comparison table: all controllers × all metrics for S1 and D1 |
| 5.4 | Verify that all expected outcomes from Section 3 §10 are confirmed or explained |

**Total estimated implementation time:** ~4–5 hours for a competent Python programmer familiar with the project context. The phased approach ensures that intermediate results are validated before building further complexity.

---

## 9. Preparation for next section

### Simulation outputs to carry forward into analysis

The simulation phase should produce the following deliverables for the analysis/evaluation section (Section 5):

**Quantitative data:**

1. **Metrics comparison table** — all controllers × {rise time, settling time, overshoot, e_ss, IAE, ITAE} for both the servo test (S1) and the regulatory test (D1). This table is the backbone of the controller comparison argument.

2. **Robustness bounds** — the range of mass values over which PI IMC (λ=5) meets all specifications, and the speed at which it violates each spec. Quantifies the claim from Section 3 that "the controller should work for m = 1200 to 2000 kg."

3. **Linearization accuracy data** — the step size at which linear and nonlinear responses diverge by more than 5%, 10%, 20%. This numerically bounds the linearization validity.

4. **Anti-windup improvement** — overshoot with and without anti-windup for the large step test (S2). Quantifies the practical value of anti-windup.

**Qualitative observations:**

1. **Which controller meets ALL specifications simultaneously?** (Expected: PI IMC, λ=5)
2. **Does the ZN-tuned PI actually produce ~25% overshoot as predicted?** (Expected: yes)
3. **Does the P controller's steady-state error match the 7.8% prediction?** (Expected: yes)
4. **At what perturbation size does the linearization visibly break down?** (Expected: ~±5 m/s from operating point)
5. **Does aggressive tuning (λ=1) cause actuator saturation in the nonlinear model?** (Expected: likely for large steps)

**Figures for inclusion:**

The most impactful figures for the project report, in priority order:

1. **Plot B1** — All-controller velocity comparison (the main result)
2. **Plot D1** — Linear vs. nonlinear at varying step sizes (the academic highlight)
3. **Plot C1** — All-controller disturbance rejection comparison
4. **Plot B3** — IMC λ-sweep (demonstrates tuning methodology)
5. **Plot E1** — Mass variation robustness
6. **Plot F1** — Anti-windup demonstration

These six figures, combined with the metrics summary table, tell the complete story: the recommended controller works, alternatives are inferior for specific reasons, the model is valid within specific bounds, and practical concerns have been addressed.
