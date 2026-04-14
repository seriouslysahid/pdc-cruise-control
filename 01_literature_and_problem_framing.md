# Literature review, problem framing, and modeling foundation for car cruise control

**Cruise control is the single most widely used introductory feedback control example in engineering education**, and for good reason: it maps a universally familiar physical system onto the exact mathematical framework taught in Process Dynamics and Control courses. This document establishes the foundational context for a CHE F342 project on car cruise control — framing the control problem, surveying the canonical academic treatments, comparing three levels of model fidelity, recommending the optimal modeling approach, defining the assumption framework, and outlining the project strategy for the sections that follow.

---

## 1. Problem framing

### Cruise control as a dynamic control system

A car traveling on a highway is a dynamic system whose velocity evolves over time according to the balance between engine driving force and resistive forces (aerodynamic drag, rolling friction, gravity on inclines). Cruise control automates the regulation of this velocity: the driver specifies a desired speed, and the controller adjusts the throttle to maintain that speed despite disturbances such as changes in road grade, headwind gusts, and varying vehicle load.

This system maps directly onto the standard feedback control loop taught in CHE F342. The process (vehicle dynamics) transforms an input (throttle/force) into an output (velocity) through a dynamic relationship governed by mass, drag, and friction. A controller observes the difference between the desired and actual velocity (the error signal) and computes a corrective throttle adjustment. The mathematical structure is identical to the feedback loops governing chemical reactors, heat exchangers, and distillation columns — making cruise control an ideal cross-disciplinary case study.

### Control system variable definitions

| Variable | Symbol | Cruise control component | Description |
|---|---|---|---|
| **Controlled Variable (CV)** | v | Vehicle velocity | The quantity to be maintained at a target value; measured in m/s or km/h |
| **Manipulated Variable (MV)** | u | Throttle position / engine force | The input adjusted by the controller; modeled as direct force (N) or normalized throttle position (0–1) |
| **Set Point (SP)** | v_ref | Desired velocity | Set by the driver; the target value for the CV |
| **Primary Disturbance** | θ | Road grade (incline angle) | Introduces a gravity component mg sin θ that opposes or assists motion |
| **Secondary Disturbances** | — | Wind, mass changes, tire pressure | Unmodeled or partially modeled perturbations affecting the force balance |

### Block diagram mapping

The cruise control system maps onto the standard feedback block diagram as follows:

| Block diagram element | Cruise control component | Typical model |
|---|---|---|
| **Plant** | Vehicle dynamics (mass + drag) | G(s) = K/(τs + 1), first-order |
| **Actuator** | Engine/throttle/drivetrain | G_e(s) = K_e/(τ_e s + 1), optional first-order lag |
| **Sensor** | Speedometer (wheel encoder or GPS) | G_m(s) = 1/(τ_m s + 1) or unity (ideal) |
| **Controller** | PI or PID algorithm | G_c(s) = K_p(1 + 1/(τ_I s)) |
| **Disturbance input** | Road grade θ | Enters as F_d = mg sin θ in the force balance |

In Jeffrey Kantor's CBE 30338 course at Notre Dame (the closest PDC-specific treatment), the plant is decomposed into cascaded transfer functions: an engine block G_e = 600/(s + 1) with a **1-second time constant**, a vehicle dynamics block G_v = 0.5/(5s + 1) with a **5-second time constant**, and a sensor block G_m = 1/(0.2s + 1) with a **0.2-second time constant**. This multi-block structure is identical to the standard PDC feedback loop with process, actuator, and measurement dynamics.

### Why cruise control is an appropriate case study for CHE F342

The pedagogical power of cruise control stems from five properties that make it uniquely suited to a Process Dynamics and Control course:

1. **Universal familiarity.** Every student has experienced the system, making the physics intuitive. No domain-specific chemistry knowledge is required to understand the dynamics.

2. **Scalable complexity.** The simplest model is a first-order ODE (accessible to undergraduates), while the full nonlinear model with engine torque curves and aerodynamic drag challenges graduate students. This scalability allows the project to be calibrated precisely to CHE F342 expectations.

3. **Complete illustration of feedback concepts.** Set-point tracking, disturbance rejection (hills), steady-state error, integral action, actuator saturation, and anti-windup compensation all arise naturally within a single application.

4. **Direct analogy to process control.** The vehicle behaves like a CSTR with nonlinear kinetics — a first-order system with a nonlinear resistance term (quadratic drag, analogous to nonlinear reaction rate) that must be linearized around an operating point. The deviation-variable formulation, Taylor-series linearization, and transfer function derivation follow the exact workflow of Seborg et al. Chapter 4.

5. **All major design methods apply.** P, PI, PID, root locus, Bode, Nyquist, state-space pole placement, and model predictive control can all be demonstrated on this single system. For CHE F342, the PI/PID design pathway with IMC or direct synthesis tuning is the most relevant.

---

## 2. Literature and existing work survey

### Canonical textbook treatments

Cruise control has appeared as a running example, design problem, or motivating case study in virtually every major control systems textbook published since the 1990s:

**Åström and Murray, *Feedback Systems: An Introduction for Scientists and Engineers* (2nd ed., 2021)**
The most complete textbook treatment of cruise control modeling. Uses the system as the **very first extended example** (Section 3.1), threading it through chapters on dynamic behavior, PID control, and state-space design. Provides:
- A full nonlinear force-balance model with quadratic drag, rolling resistance, and road grade
- A nonlinear engine torque curve T(ω) = T_m[1 − β(ω/ω_m − 1)²] with five gear ratios
- Explicit linearization around an operating point with numerical coefficients
- PI controller design with anti-windup for actuator saturation
- Robustness analysis under mass uncertainty (m = 1200–2000 kg)
- **Parameter set:** m = 1600 kg, C_d = 0.32, A = 2.4 m², ρ = 1.3 kg/m³, C_rr = 0.01, T_m = 190 N·m, ω_m = 420 rad/s, β = 0.4
- Freely available at cds.caltech.edu/~murray/books/AM08/

**University of Michigan CTMS (Control Tutorials for MATLAB and Simulink)**
The most widely referenced free online controls resource. Builds its **entire tutorial suite** around cruise control, covering PID design, root locus, Bode analysis, state-space control, digital control, and Simulink implementation using a single consistent model:
- **Simplified linear model:** m(dv/dt) = u − bv with m = 1000 kg, b = 50 N·s/m
- **Design specifications:** Rise time < 5 s, overshoot < 10%, steady-state error < 2%
- **Benchmark controller:** K_p = 800, K_i = 40 (PI)
- URL: ctms.engin.umich.edu

**Seborg, Edgar, Mellichamp, and Doyle, *Process Dynamics and Control* (4th ed., 2016)**
The CHE F342 standard text. Does not contain explicit cruise control examples — its cases focus on chemical reactors, distillation, and heat exchangers — but every concept (linearization, deviation variables, transfer functions, IMC tuning, integral action) transfers directly. The textbook's treatment of FOPDT models and IMC-based PID tuning provides the exact methodology applicable to this project.

**Dorf and Bishop, *Modern Control Systems* (13th ed., 2017)**
Assigns cruise control as **Design Problem DP1.1**, revisited across multiple chapters using root locus, frequency response, and state-space methods. Provides a running design exercise that progressively adds complexity.

**Nise, *Control Systems Engineering* (8th ed., 2019)**
Uses cruise control in its introductory chapter to motivate feedback control concepts. Also used in MIT OCW 2.004 (Lecture 2) as a first dynamic systems example.

### PDC-specific educational resources

**Jeffrey Kantor, CBE 30338 (Notre Dame)**
Course materials at jckantor.github.io/CBE30338/ demonstrate cruise control with the multi-block transfer function structure (engine, vehicle, sensor dynamics with PI controller) that directly mirrors Seborg et al.'s treatment of feedback loops. Uses:
- Engine dynamics: G_e = 600/(s + 1), τ_e = 1 s
- Vehicle dynamics: G_v = 0.5/(5s + 1), τ_v = 5 s
- Sensor dynamics: G_m = 1/(0.2s + 1), τ_m = 0.2 s
- PI controller design using the cascade transfer function structure

**Engineering LibreTexts — Chemical Process Dynamics and Controls**
Section 6.8 treats cruise control for an electric vehicle, applying PDC concepts including linearization, eigenvalue analysis, and PID design. Demonstrates the connection between automotive and chemical process control.

**Rajamani, *Vehicle Dynamics and Control* (Springer, 2nd ed., 2012)**
The definitive graduate-level treatment across Chapters 4–7, covering engine models, tire dynamics (Pacejka "Magic Formula"), and adaptive cruise control. Provides the upper bound of complexity that is explicitly beyond the scope of an undergraduate project but valuable for understanding what the simplified models omit.

### Standard benchmark parameters

Two benchmark parameter sets dominate the academic literature:

| Parameter set | m (kg) | Drag model | Key constants | Source |
|---|---|---|---|---|
| **CTMS benchmark** | 1000 | Linear: bv | b = 50 N·s/m | Michigan CTMS |
| **Åström–Murray** | 1600 | Nonlinear: ½ρC_dAv² | C_d=0.32, A=2.4, ρ=1.3 | Feedback Systems §3.1 |

The Åström–Murray parameter set is preferred for this project because it uses physically meaningful, independently measurable parameters rather than a lumped damping coefficient.

### Common simplifications in undergraduate treatments

The following simplifications recur across the undergraduate literature:

1. **Lumped linear drag** (bv instead of ½ρC_dAv²) — analytically tractable but physically inaccurate; the parameter b has no direct measurable counterpart
2. **Neglected powertrain dynamics** — the control input is treated as a force applied directly at the tire-road interface, bypassing engine, transmission, and drivetrain
3. **Flat road assumption** — grade θ is set to zero for baseline modeling, introduced later as a step disturbance for testing
4. **Unity feedback** — the velocity sensor is assumed ideal (no lag, no noise, perfect accuracy)
5. **Constant mass** — fuel consumption, passengers, and cargo variations are neglected
6. **Single-degree-of-freedom** — the vehicle is a point mass; no lateral, vertical, or rotational dynamics

These simplifications are appropriate for demonstrating core PDC concepts. The key question is *which nonlinearity to retain* — and the answer is the quadratic drag term, because linearizing it demonstrates the most important mathematical technique in the course.

---

## 3. Modeling fidelity comparison

### Level 1: Simplified linear first-order model

**Governing equation:**

m(dv/dt) = u − bv

**Transfer function:**

G(s) = V(s)/U(s) = (1/b) / ((m/b)s + 1) = K/(τs + 1)

with K = 1/b (steady-state gain) and τ = m/b (time constant).

**Numerical values (CTMS benchmark):** m = 1000 kg, b = 50 N·s/m → τ = 20 s, K = 0.02 (m/s)/N.

| Aspect | Assessment |
|---|---|
| **Advantages** | Analytically tractable; exact closed-form step response v(t) = (u/b)(1 − e^{−bt/m}); standard first-order dynamics; PI closed-loop analysis yields explicit ω_n, ζ formulas |
| **Disadvantages** | Physically inaccurate — real drag scales with v², not v; the parameter b is a lumped approximation with no direct physical measurement; does not demonstrate linearization; gain and time constant are independent of operating point (unrealistic) |
| **Complexity** | 2 parameters (m, b); 1 state; fully linear |
| **Suitability** | Appropriate for introductory exercises and quick demonstrations. **Insufficient for a project-level CHE F342 submission** because it skips the linearization step, which is the most important mathematical technique to demonstrate. |

### Level 2: Nonlinear force-balance model

**Governing equation:**

m(dv/dt) = F_engine − ½ρC_dAv² − C_rr mg − mg sin θ

**After linearization at operating point (v₀, u₀):**

G(s) = (b/m) / (s + a/m) = K/(τs + 1)

where b_eff = ρC_dAv₀ (effective damping), τ = m/b_eff, K = 1/b_eff for the simplified direct-force input model. Both K and τ depend on the operating velocity v₀ — a physically meaningful feature.

**Numerical values (Åström–Murray, v₀ = 25 m/s):** b_eff = 1.3 × 0.32 × 2.4 × 25 = 24.96 N·s/m → τ ≈ 64.1 s, K ≈ 0.0401 (m/s)/N.

| Aspect | Assessment |
|---|---|
| **Advantages** | Physically meaningful parameters (all independently measurable); demonstrates linearization (core PDC skill); operating-point-dependent K and τ capture real physics; enables linear vs. nonlinear simulation comparison; disturbance (grade) enters formally as a separate TF channel; maps directly onto Seborg et al.'s nonlinear process modeling framework |
| **Disadvantages** | Requires linearization before transfer function analysis (but this is a *feature*, not a bug, for a PDC project); still a single-state-variable model (no powertrain dynamics unless explicitly added) |
| **Complexity** | 5–7 parameters (m, ρ, C_d, A, C_rr, g, θ); 1 state; nonlinear ODE linearized to first-order TF |
| **Suitability** | **Optimal for CHE F342 project.** Demonstrates the complete PDC workflow: nonlinear modeling → operating-point analysis → Taylor-series linearization → Laplace-domain transfer function → controller design. Optionally adding a first-order engine dynamics block creates a second-order cascade that enriches the design without excessive complexity. |

### Level 3: Higher-fidelity automotive longitudinal dynamics

**Components included:** Throttle actuator lag (τ_a ≈ 0.05–0.2 s), intake manifold filling dynamics (τ_e ≈ 0.2–1.0 s), engine torque maps as 2D lookup tables, drivetrain compliance and gear ratios, tire slip dynamics (Pacejka model), brake system hydraulic lag (τ_brake ≈ 0.1–0.5 s).

**Plant structure:** G(s) = G_actuator · G_engine · G_drivetrain · G_vehicle — a cascade of 3–5 subsystems.

| Aspect | Assessment |
|---|---|
| **Advantages** | Captures real powertrain and tire dynamics; essential for production ACC design, HIL testing, and platoon string-stability analysis; very high physical fidelity |
| **Disadvantages** | 14–35 state variables; 20–100+ parameters requiring specialized identification; not analytically tractable; requires CarSim, IPG CarMaker, or MATLAB Vehicle Dynamics Blockset; parameter identification burden far exceeds undergraduate feasibility |
| **Complexity** | Extreme; graduate/industry level |
| **Suitability** | **Unnecessarily complex for CHE F342.** The additional fidelity does not add pedagogical value for a PDC course — it obscures the core concepts (linearization, transfer functions, PID design) beneath layers of automotive-specific detail. |

### Summary comparison

| Feature | Linear first-order | Nonlinear force-balance | High-fidelity |
|---|---|---|---|
| Parameters | 2 (m, b) | 5–7 (m, ρ, C_d, A, C_rr, g, θ) | 20–100+ |
| States | 1 | 1 | 5–35 |
| Analytical solution | Yes | After linearization | No |
| Physical interpretability | Low (b is lumped) | High (all measurable) | Very high |
| Demonstrates linearization | No | **Yes** | Overkill |
| Demonstrates PDC workflow | Partially | **Completely** | Beyond scope |
| Undergraduate suitability | Introductory exercises | **Project sweet spot** | Graduate/industry |

---

## 4. Recommended modeling approach

### Recommendation: Nonlinear force-balance model with linearization

The **nonlinear force-balance model** (Level 2) using the **Åström and Murray parameter set** is recommended as the primary modeling approach for this project.

### Justification

**Sufficient rigor.** The model is derived from first principles (Newton's second law), uses physically meaningful and independently measurable parameters, includes the dominant nonlinearity (quadratic aerodynamic drag), and produces a linearized transfer function that is operating-point-dependent — reflecting real physics. It is substantially more rigorous than the simplified linear model used in the CTMS tutorials while remaining analytically tractable.

**Academic defensibility.** The model and parameter set are drawn directly from Åström and Murray's *Feedback Systems*, which is the most widely cited controls textbook in the field. The linearization methodology follows the standard Taylor-series approach taught in Seborg et al. Chapter 4. Every step — from physical modeling through transfer function derivation — has direct textbook precedent and can be defended against any examiner question.

**Alignment with CHE F342 objectives.** The model demonstrates five core PDC competencies that define the course:

1. **Nonlinear dynamic modeling** — constructing an ODE from physical principles
2. **Linearization** — Taylor-series expansion of the v² drag term around an operating point
3. **Transfer function derivation** — Laplace transform of the linearized ODE
4. **Controller design** — PI/PID design using the derived plant model
5. **Performance analysis** — comparing linear and nonlinear simulation to assess model validity

The linearization step is the single most important demonstration, because it is the technique that distinguishes PDC analysis from simple linear systems courses. The simplified linear model skips this step entirely.

**Avoidance of unnecessary overcomplexity.** The model has only 5–7 parameters, maintains a single state variable (velocity), and produces a first-order transfer function after linearization. Adding a first-order engine dynamics block optionally extends the plant to second order — sufficient to demonstrate cascade dynamics and enriched controller design without introducing the parameter identification challenges of higher-fidelity models.

### Recommended parameter set

The Åström and Murray parameter set is adopted as the project standard:

| Parameter | Symbol | Value | Units | Source |
|---|---|---|---|---|
| Vehicle mass | m | 1600 | kg | Åström & Murray §3.1 |
| Drag coefficient | C_d | 0.32 | — | Typical sedan |
| Frontal area | A | 2.4 | m² | Typical sedan |
| Air density | ρ | 1.3 | kg/m³ | Sea-level approximation |
| Rolling resistance | C_rr | 0.01 | — | Standard tire on asphalt |
| Gravitational acceleration | g | 9.8 | m/s² | Standard |
| Operating velocity | v₀ | 25 | m/s | Highway cruising (90 km/h) |

This parameter set is preferred over the CTMS benchmark (m = 1000, b = 50) because every parameter has a direct physical interpretation and can be independently measured or looked up from engineering reference tables. The operating point v₀ = 25 m/s (90 km/h) is recommended as a representative mid-range highway speed, consistent with Åström and Murray's analysis.

---

## 5. Engineering assumptions framework

### Assumption 1: Flat road with grade as a testable disturbance

**Statement:** The baseline model assumes θ = 0 (flat road). Road grade is introduced as a step disturbance after the baseline model and controller are established.

**Why reasonable:** Highway roads are designed to minimize grade. The vast majority of highway driving occurs on grades less than 3° (5.2%). Treating grade as a perturbation rather than a baseline condition is physically accurate and standard practice.

**Mathematical implication:** At θ₀ = 0, the gravity term mg sin θ₀ = 0 drops out of the baseline ODE, and the linearized disturbance transfer function simplifies to G_d(s) = −mg cos(0)/(ms + b_eff) = −mg/(ms + b_eff). The grade enters as a separate input channel in the block diagram.

**Limitation:** The model does not capture the effect of sustained grades (e.g., mountain passes) where the operating point itself shifts. A controller designed for flat-road operation should be tested under grade disturbances to verify robustness.

### Assumption 2: Constant vehicle mass

**Statement:** Mass m = 1600 kg is treated as a fixed parameter throughout the modeling and controller design phases.

**Why reasonable:** Vehicle mass changes slowly (passengers boarding, fuel consumption) relative to the control dynamics (time constants of seconds to minutes). Within any single cruise control engagement, mass is effectively constant.

**Mathematical implication:** Mass appears in the time constant τ = m/b_eff and the disturbance gain K_d = g/a. A fixed m yields fixed τ and K_d, simplifying the analysis. The PI controller designed for m = 1600 kg will be tested for robustness across m = 1200–2000 kg in the simulation phase.

**Limitation:** At extreme mass variations (empty vs. fully loaded SUV), the time constant τ changes by up to ±25%, which alters the transient response. Integral action in the PI controller compensates for this by adjusting the steady-state throttle regardless of the actual mass.

### Assumption 3: Quadratic aerodynamic drag, linearized at the operating point

**Statement:** Aerodynamic drag is modeled as F_aero = ½ρC_dAv², then linearized via Taylor expansion about v₀ = 25 m/s to obtain F_aero ≈ ½ρC_dAv₀² + ρC_dAv₀·δv.

**Why reasonable:** The v² dependence is experimentally well-established from wind-tunnel testing and dimensional analysis. Linearization is valid for perturbations small relative to v₀ — specifically, when (δv/v₀)² ≪ 1. At v₀ = 25 m/s, perturbations of ±5 m/s (20%) produce linearization errors of ~4%, which is acceptable for controller design.

**Mathematical implication:** The linearized drag produces an effective velocity-dependent damping coefficient b_eff = ρC_dAv₀ = 24.96 N·s/m at the operating point. This coefficient determines both the process time constant (τ = m/b_eff ≈ 64.1 s) and the process gain (K = 1/b_eff ≈ 0.040 (m/s)/N). Crucially, both K and τ depend on v₀, making the linearized model operating-point-dependent.

**Limitation:** The linearization becomes increasingly inaccurate for large perturbations. At δv = 12.5 m/s (50% of v₀), the linearization error reaches 25%. The simulation phase should compare linearized and nonlinear model responses at multiple step sizes to quantify this degradation.

### Assumption 4: Neglected powertrain dynamics (primary model)

**Statement:** The control input u is treated as a force applied directly at the tire-road interface, with no engine, transmission, or drivetrain dynamics between the controller output and the applied force.

**Why reasonable:** The dominant vehicle dynamics (time constant τ ≈ 64 s) are 60–600× slower than typical powertrain dynamics (engine lag τ_e ≈ 0.1–1 s). The engine response is essentially instantaneous relative to the vehicle response. This is the standard assumption in Åström and Murray's primary treatment, with engine dynamics added only as an optional extension.

**Mathematical implication:** The plant transfer function is first-order: G(s) = K/(τs + 1). Adding engine dynamics would create a second-order cascade G(s) = K_eK_v/[(τ_es + 1)(τ_vs + 1)], with the fast engine pole (at s ≈ −1) dominated by the slow vehicle pole (at s ≈ −0.016). The controller tuning does not change significantly because IMC and direct synthesis automatically account for all plant poles.

**Limitation:** Neglecting engine dynamics means the model cannot capture the brief throttle-to-force delay (0.1–1 s). If engine dynamics are found to affect controller performance in simulation, a first-order lag G_e(s) = K_e/(τ_es + 1) can be added as a cascade extension.

### Assumption 5: Single-degree-of-freedom, lumped-parameter, rigid body

**Statement:** The vehicle is modeled as a point mass m moving along one axis (longitudinal). No lateral motion, vertical suspension, pitch/roll, or structural flexibility is considered.

**Why reasonable:** Cruise control regulates longitudinal velocity only. Lateral dynamics (steering), vertical dynamics (suspension), and rotational dynamics (yaw, pitch, roll) are decoupled from the longitudinal force balance at the level of approximation relevant to PDC. Every undergraduate-level textbook treatment makes this assumption.

**Mathematical implication:** The system has one state variable (velocity v) and one governing ODE. The model produces a single-input, single-output (SISO) transfer function amenable to all classical control design techniques.

**Limitation:** Tire slip (the difference between wheel rotational speed and vehicle translational speed), which couples longitudinal and tire dynamics, is neglected. This is only relevant under hard acceleration or braking — conditions outside the normal cruise control operating envelope.

### Assumption 6: Constant rolling resistance coefficient

**Statement:** C_rr = 0.01 is treated as constant and independent of velocity.

**Why reasonable:** For standard passenger car tires on paved roads, C_rr varies by less than 5% over the 15–35 m/s operating range. Temperature-dependent variations are slow and small relative to the control dynamics.

**Mathematical implication:** Rolling resistance F_roll = C_rr·m·g is a constant force that appears in the steady-state force balance but vanishes from the linearized dynamic model (because ∂F_roll/∂v = 0). It determines the operating-point throttle force u₀ but has no influence on the perturbation dynamics or the transfer function.

**Limitation:** At very low speeds (< 5 m/s) or on soft surfaces (gravel, sand), C_rr can increase significantly. These conditions are outside the cruise control operating range.

### Assumption 7: Ideal velocity sensing

**Statement:** The velocity measurement is assumed instantaneous, noiseless, and perfectly accurate — equivalent to unity feedback gain G_m(s) = 1.

**Why reasonable:** Modern wheel-speed sensors (ABS encoder rings) provide velocity measurements at 50–100 Hz with resolution of ~0.05 m/s. The sensor bandwidth (50+ Hz) is 3000× higher than the closed-loop bandwidth (~0.016 Hz), making the sensor dynamics negligible. Measurement noise is addressed qualitatively in the controller design section.

**Mathematical implication:** Unity feedback simplifies the closed-loop transfer function: G_CL(s) = G_c·G/(1 + G_c·G) with no sensor pole. If sensor dynamics are included (e.g., G_m = 1/(0.2s + 1)), the effective loop transfer function gains an additional fast pole that does not significantly affect controller design.

**Limitation:** Real sensors introduce quantization noise (±0.05 m/s) and occasional outliers. This primarily affects derivative action in PID controllers, not the recommended PI design.

### Assumptions summary table

| # | Assumption | Key consequence | Revisited in simulation? |
|---|---|---|---|
| 1 | Flat road (θ = 0 baseline) | Grade enters as test disturbance | Yes — step grade response |
| 2 | Constant mass (m = 1600 kg) | Fixed τ and K | Yes — mass sweep 1200–2000 kg |
| 3 | Linearized quadratic drag | b_eff = ρC_dAv₀; operating-point-dependent | Yes — linear vs. nonlinear comparison |
| 4 | No powertrain dynamics | First-order plant | Optionally — cascade extension |
| 5 | Point mass, 1-DOF | SISO transfer function | No — structural assumption |
| 6 | Constant C_rr | Vanishes from linearized dynamics | No — minor parameter |
| 7 | Ideal sensor | Unity feedback | Qualitatively in controller design |

---

## 6. Preliminary project strategy

### Overall technical direction

The project follows the **standard PDC workflow** applied to the cruise control system:

1. **Model:** Derive the nonlinear force-balance ODE from Newton's second law using the Åström and Murray parameter set
2. **Linearize:** Taylor-expand the quadratic drag about v₀ = 25 m/s, define deviation variables, subtract the steady-state equation
3. **Transfer function:** Apply Laplace transforms to obtain G(s) = K/(τs + 1) and the disturbance transfer function G_d(s)
4. **Controller design:** Design P, PI, and PID controllers; recommend PI with IMC tuning; compare with Ziegler–Nichols as a secondary method
5. **Simulate:** Implement both the linearized and full nonlinear models in Python; run set-point tracking, disturbance rejection, and robustness scenarios
6. **Analyze:** Compare controllers quantitatively; validate linearization accuracy; discuss limitations; recommend the best controller with justification

### Mathematical modeling approach for Section 2

The next section should derive the governing equations in this sequence:

1. State Newton's second law with all four forces (engine, aerodynamic drag, rolling resistance, grade)
2. Calculate the steady-state equilibrium (u₀ at v₀ = 25 m/s on flat road)
3. Perform the Taylor-series linearization of the v² term — this is the **critical derivation step** and must be shown in full
4. Define deviation variables (δv, δu, δθ), subtract the steady-state equation, and derive the linearized ODE
5. Apply Laplace transforms to obtain the process and disturbance transfer functions
6. Express in standard first-order form and compute numerical values for τ, K, and K_d
7. Verify dimensions of every term and check limiting cases (v₀ → 0, m → ∞, C_d → 0)
8. Optionally introduce engine dynamics as a cascaded first-order lag

### Pitfalls to avoid before derivation begins

1. **Do not skip the linearization.** Using the simplified linear model (m·dv/dt = u − bv) directly avoids the most important mathematical step in the project. The quadratic drag must be retained and linearized explicitly.

2. **Do not confuse deviation and absolute variables.** The transfer function relates δV(s) and δU(s) (deviations from the operating point), not absolute V(s) and U(s). Zero initial conditions apply to δv(0) = 0, not v(0) = 0. The steady-state subtraction step must be shown explicitly to make this distinction clear.

3. **Do not mix parameter sets.** The CTMS benchmark (m = 1000, b = 50) and the Åström–Murray set (m = 1600, C_d = 0.32, ...) produce different numerical results. Choose one set and use it consistently throughout. This project uses the Åström–Murray set.

4. **Do not omit dimensional analysis.** Verifying that every term in the ODE has units of Newtons, and that τ emerges in seconds and K in (m/s)/N, is the simplest and most effective way to catch algebraic errors and demonstrate engineering rigor.

5. **Do not neglect the steady-state force calculation.** The equilibrium force u₀ = ½ρC_dAv₀² + C_rr·m·g provides a critical physical sanity check — it should correspond to a realistic engine power output (typically 10–20 kW at highway speed).

6. **Do not present the transfer function without physical interpretation.** The time constant τ = m/(ρC_dAv₀) has a clear physical meaning (ratio of vehicle inertia to aerodynamic damping), and the gain K = 1/(ρC_dAv₀) tells you how much speed change results from a unit force change. These interpretations distinguish an understanding of the physics from mere algebra.

---

## 7. Reference guidance

### Primary references (essential)

| Reference | Content relevant to this project | Access |
|---|---|---|
| **Åström & Murray, *Feedback Systems*, 2nd ed. (2021)** | Complete nonlinear cruise control model, linearization, PI control with anti-windup, robustness analysis | Free PDF: cds.caltech.edu/~murray/books/AM08/ |
| **Seborg, Edgar, Mellichamp & Doyle, *Process Dynamics and Control*, 4th ed. (2016)** | Linearization methodology (Ch. 4), transfer functions (Ch. 5), PID design (Ch. 12), IMC tuning (Ch. 12) | CHE F342 course textbook |
| **University of Michigan CTMS** | Complete MATLAB/Simulink tutorials for all control design methods on cruise control | ctms.engin.umich.edu |
| **Kantor, CBE 30338 course materials** | PDC-specific cruise control with multi-block TF structure | jckantor.github.io/CBE30338/ |

### Secondary references (valuable)

| Reference | Contribution |
|---|---|
| **Dorf & Bishop, *Modern Control Systems*, 13th ed.** | Multi-chapter cruise control design problem (root locus, Bode, state-space) |
| **Nise, *Control Systems Engineering*, 8th ed.** | Introductory cruise control example; used in MIT OCW 2.004 |
| **Rajamani, *Vehicle Dynamics and Control*, 2nd ed. (2012)** | Graduate-level vehicle dynamics; defines the upper complexity bound |
| **Engineering LibreTexts, Chemical Process Dynamics and Controls §6.8** | EV cruise control with PDC methodology |
| **Python-control library** (python-control.readthedocs.io) | Murray's nonlinear cruise control simulation with linearization and PI design |

### Useful search terms for further research

- "Cruise control transfer function derivation"
- "Vehicle longitudinal dynamics linearization"
- "First-order plant PI controller IMC tuning"
- "Åström Murray cruise control model parameters"
- "FOPDT PID tuning comparison IMC Ziegler-Nichols"
- "Process dynamics and control automotive example"
- "Nonlinear ODE linearization Taylor series operating point"
- "Cruise control disturbance rejection road grade"

### Software and implementation tools

| Tool | Role in project |
|---|---|
| **python-control** | Transfer function creation, closed-loop algebra, step/Bode/root-locus analysis |
| **scipy.integrate** | Nonlinear ODE simulation (solve_ivp with RK45) |
| **numpy** | Numerical computation and array operations |
| **matplotlib** | All visualization — time responses, Bode plots, comparison figures |

---

## Conclusion: foundation for a top-grade CHE F342 submission

The literature survey and modeling analysis converge on a clear project strategy: adopt the **nonlinear force-balance model** with the **Åström and Murray parameter set** (m = 1600 kg, C_d = 0.32, A = 2.4 m², ρ = 1.3 kg/m³, C_rr = 0.01), linearize around the highway operating point **v₀ = 25 m/s** to derive an operating-point-dependent first-order transfer function, and design controllers using the resulting G(s) = K/(τs + 1).

This approach directly demonstrates the **five core PDC competencies** examinable in CHE F342: nonlinear dynamic modeling, Taylor-series linearization, transfer function derivation, PI/PID controller design with systematic tuning, and closed-loop performance analysis. The key differentiator for a top-grade submission is the comparison between linearized and nonlinear simulation results — a comparison that quantifies the accuracy of the linearization approximation and demonstrates why feedback control provides inherent robustness to modeling uncertainty. As Åström and Murray emphasize: *"One of the amazing properties of control systems is that they can often be designed based on simple models."*