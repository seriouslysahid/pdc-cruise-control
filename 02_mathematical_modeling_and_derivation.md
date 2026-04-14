# Mathematical modeling and system derivation for car cruise control

**A car maintaining highway speed is a first-order dynamic system governed by the competition between engine thrust and aerodynamic drag.** This section derives the complete mathematical model from first principles — Newton's second law through Laplace-domain transfer functions — following the methodology of Åström and Murray's *Feedback Systems*, Seborg et al.'s *Process Dynamics and Control*, and the University of Michigan CTMS tutorials. Using the Åström and Murray parameter set (m = 1600 kg, C_d = 0.32, A = 2.4 m², ρ = 1.3 kg/m³, C_rr = 0.01) adopted in Section 1, the linearized vehicle plant at v₀ = 25 m/s is a first-order transfer function with a **time constant of approximately 64 seconds** and a **DC gain of 0.040 (m/s)/N** — or equivalently, τ ≈ 73 s and K ≈ 101 (m/s)/throttle when engine torque coupling is included. Every intermediate step is shown with dimensional verification and physical interpretation.

---

## 1. System definition

### Physical system

The system under study is a passenger sedan traveling longitudinally along a road that may be inclined at angle θ from horizontal. The vehicle is modeled as a **point mass** m moving in one dimension (longitudinal direction only), subject to four forces along the direction of motion: engine/traction force (driving), aerodynamic drag (resistive), rolling resistance (resistive), and gravitational grade force (disturbance).

### Selected model scope and fidelity

As recommended in Section 1 §4, the project adopts the **nonlinear force-balance model** (Level 2 fidelity) with the **Åström and Murray parameter set**. This model:
- Derives from Newton's second law with physically meaningful, independently measurable parameters
- Retains the quadratic aerodynamic drag nonlinearity (the v² term) to demonstrate Taylor-series linearization
- Treats the engine input as a direct force (Newtons) in the primary derivation, with the Åström–Murray engine torque model presented as an extension
- Produces a first-order transfer function after linearization

### Variable and parameter definitions

**State and input variables:**

| Symbol | Name | Units | Description |
|---|---|---|---|
| v | Vehicle velocity | m/s | State variable (controlled variable) |
| u | Engine/traction force | N | Manipulated input (direct-force model) |
| θ | Road grade angle | rad | Disturbance input |
| t | Time | s | Independent variable |

**Physical parameters (Åström–Murray set):**

| Symbol | Name | Value | Units |
|---|---|---|---|
| m | Vehicle mass | 1600 | kg |
| C_d | Drag coefficient | 0.32 | — (dimensionless) |
| A | Frontal area | 2.4 | m² |
| ρ | Air density | 1.3 | kg/m³ |
| C_rr | Rolling resistance coefficient | 0.01 | — (dimensionless) |
| g | Gravitational acceleration | 9.8 | m/s² |

**Derived quantities (defined during derivation):**

| Symbol | Name | Expression | Value at v₀ = 25 m/s |
|---|---|---|---|
| b_eff | Effective damping coefficient | ρC_dAv₀ | 24.96 N·s/m |
| u₀ | Steady-state force | ½ρC_dAv₀² + C_rr mg | 468.8 N |
| τ | Process time constant | m/b_eff | 64.1 s |
| K | Process gain | 1/b_eff | 0.0401 (m/s)/N |
| K_d | Disturbance gain | g·m/b_eff | 628 (m/s)/rad |

**Deviation variables (defined at linearization):**

| Symbol | Definition | Meaning |
|---|---|---|
| δv | v − v₀ | Velocity perturbation from operating point |
| δu | u − u₀ | Force perturbation from operating point |
| δθ | θ − θ₀ | Grade perturbation from operating point |

### Sign conventions

- **Positive v:** Forward motion along the road surface
- **Positive u:** Force in the direction of motion (driving force)
- **Positive θ:** Uphill grade (gravity opposes motion)
- **Resistive forces** (drag, rolling resistance, uphill grade): subtract from the net force in Newton's second law

---

## 2. Governing physics and force balance derivation

### Newton's second law

The vehicle is a point mass m subject to four forces along the direction of motion. Newton's second law gives:

$$m\frac{dv}{dt} = F_{\text{engine}} - F_{\text{aero}} - F_{\text{roll}} - F_{\text{grade}}$$

Each force term has a clear physical origin and well-established mathematical form.

### Aerodynamic drag: F_aero = ½ρC_dAv²

Aerodynamic drag arises from the pressure differential created as the vehicle pushes through air. The standard drag equation is derived from dimensional analysis and validated by extensive wind-tunnel testing:

$$F_{\text{aero}} = \tfrac{1}{2}\rho C_d A v^2$$

where ρ is air density (1.3 kg/m³ at sea level), C_d is the drag coefficient (0.32 for a typical sedan — a dimensionless shape factor characterizing the vehicle's aerodynamic profile), A is the frontal cross-sectional area (2.4 m²), and v is the vehicle speed.

**The quadratic dependence on v is the fundamental nonlinearity** that necessitates linearization for transfer function-based controller design. The Laplace transform cannot be applied to v² directly — this point is critical and is the central mathematical challenge of the derivation.

**Sign convention:** Drag always opposes motion, so it enters with a negative sign in the force balance (subtracted from the driving force).

**Assumption:** The drag coefficient C_d is constant and independent of velocity. This is accurate for the 15–35 m/s operating range; C_d varies significantly only at very low Reynolds numbers (< 5 m/s) or near the sound barrier, neither of which is relevant here (Section 1, Assumption 3).

### Rolling resistance: F_roll = C_rr · m · g · cos(θ)

Rolling resistance models the energy dissipated through continuous tire deformation as the tire rolls on the road surface:

$$F_{\text{roll}} = C_{rr} \cdot m \cdot g \cdot \cos(\theta)$$

The coefficient C_rr ≈ 0.01 is nearly constant for standard passenger car tires on paved roads over the relevant speed range. The cos(θ) factor accounts for the reduced normal force on an incline, though for small grades (θ < 10°), cos(θ) > 0.985 and is negligibly close to unity.

**Key property for linearization:** Rolling resistance is **independent of velocity** for v > 0. This means ∂F_roll/∂v = 0 — it has no velocity-dependent component. As shown in §5, this causes rolling resistance to **vanish entirely from the linearized dynamic model**, contributing only to the steady-state force balance.

**Assumption:** C_rr is constant and independent of velocity, temperature, and tire pressure (Section 1, Assumption 6).

### Grade force: F_grade = m · g · sin(θ)

The grade force is simply the component of gravitational force along the road surface:

$$F_{\text{grade}} = m \cdot g \cdot \sin(\theta)$$

**Sign convention:** Positive θ (uphill) produces a positive grade force that opposes forward motion. Negative θ (downhill) assists motion.

**Role in control:** θ is treated as an **exogenous disturbance input** rather than a state variable, following Åström and Murray's formulation. The controller does not know θ directly — it only observes the resulting velocity change and responds through feedback.

**Assumption:** Flat road baseline (θ₀ = 0), with grade introduced as a perturbation disturbance for testing (Section 1, Assumption 1).

### Engine/traction force: F_engine = u

In the primary (simplified) derivation, the engine force is the **direct manipulated input**:

$$F_{\text{engine}} = u \quad \text{(units: Newtons)}$$

This treatment, used by the CTMS tutorials and many undergraduate texts, bypasses all powertrain dynamics (engine lag, transmission, drivetrain inertia) and assumes that the controller output produces an immediate force at the tire-road interface. This is justified by the large time-scale separation: the vehicle dynamics time constant (τ ≈ 64 s) is 60–600× larger than typical engine lag (τ_e ≈ 0.1–1 s), making the engine response effectively instantaneous (Section 1, Assumption 4).

In Åström and Murray's more complete model, the engine force depends on throttle position u ∈ [0, 1], gear ratio α_n, and a nonlinear engine torque curve:

$$F_{\text{engine}} = \alpha_n \cdot u \cdot T(\alpha_n \cdot v)$$

This formulation is presented as an extension in §8 after the primary derivation is complete.

---

## 3. Final nonlinear dynamic equation

### Complete governing ODE

Combining all force terms with the flat-road baseline (θ = 0) and direct force input:

$$m\frac{dv}{dt} = u - \tfrac{1}{2}\rho C_d A v^2 - C_{rr} m g$$

With road grade included as a general disturbance:

$$m\frac{dv}{dt} = u - \tfrac{1}{2}\rho C_d A v^2 - C_{rr} m g \cos(\theta) - m g \sin(\theta)$$

### Physical interpretation of each term

| Term | Role | Magnitude at v₀ = 25 m/s, θ = 0 |
|---|---|---|
| m(dv/dt) | Inertial force (acceleration × mass) | 0 at steady state |
| u | Driving force (manipulated input) | 468.8 N at equilibrium |
| ½ρC_dAv² | Aerodynamic drag (resistive, nonlinear) | 312.0 N (67% of total resistance) |
| C_rr mg | Rolling resistance (resistive, constant) | 156.8 N (33% of total resistance) |
| mg sin θ | Grade force (disturbance) | 0 N on flat road |

### Sources of nonlinearity

The ODE contains **one source of nonlinearity** in the state variable v: the **quadratic drag term** ½ρC_dAv². All other terms are either linear in v (none), constant (C_rr mg), linear in the input (u), or trigonometric in the disturbance (sin θ, cos θ).

The system is a **first-order nonlinear ODE** with one state variable (v), one manipulated input (u), and one disturbance input (θ). Åström and Murray describe this system in Chapter 3 of *Feedback Systems*: "The system is nonlinear because of the torque curve, the gravity term, and the nonlinear character of rolling friction and aerodynamic drag."

---

## 4. Steady-state and operating-point analysis

### The equilibrium condition

The steady state is the constant velocity v₀ maintained by a constant force u₀ on a flat road (θ₀ = 0). Setting dv/dt = 0:

$$0 = u_0 - \tfrac{1}{2}\rho C_d A v_0^2 - C_{rr} m g$$

Solving for the steady-state force:

$$u_0 = \tfrac{1}{2}\rho C_d A v_0^2 + C_{rr} m g$$

### Numerical evaluation at v₀ = 25 m/s

Computing each term with the Åström–Murray parameters:

$$F_{\text{aero},0} = \tfrac{1}{2} \times 1.3 \times 0.32 \times 2.4 \times 25^2 = \tfrac{1}{2} \times 1.3 \times 0.32 \times 2.4 \times 625 = \textbf{312.0 N}$$

$$F_{\text{roll},0} = 0.01 \times 1600 \times 9.8 = \textbf{156.8 N}$$

$$u_0 = 312.0 + 156.8 = \textbf{468.8 N}$$

### Operating-point comparison across speeds

| v₀ (m/s) | v₀ (km/h) | F_aero (N) | F_roll (N) | u₀ (N) | Drag fraction |
|---|---|---|---|---|---|
| 20 | 72 | 199.7 | 156.8 | 356.5 | 56% |
| **25** | **90** | **312.0** | **156.8** | **468.8** | **67%** |
| 30 | 108 | 449.3 | 156.8 | 606.1 | 74% |
| 35 | 126 | 611.5 | 156.8 | 768.3 | 80% |

The table illustrates a key physical fact: **aerodynamic drag grows quadratically with speed** while rolling resistance remains constant. At highway speeds, drag dominates the force budget — at 25 m/s (90 km/h), drag accounts for 67% of total resistance; at 35 m/s (126 km/h), 80%.

### Why v₀ = 25 m/s is appropriate

The operating point v₀ = 25 m/s (90 km/h) is chosen because:

1. It is a representative mid-range highway cruising speed (within 80–120 km/h typical range)
2. It is consistent with Åström and Murray's analysis, facilitating comparison with published results
3. It places the operating point in the aerodynamic-drag-dominated regime (drag > 60% of total resistance), where the quadratic nonlinearity is significant and linearization provides meaningful insight
4. Perturbations of ±5 m/s remain within the normal cruise control operating envelope

### Power verification

The steady-state power P = F · v = 468.8 × 25 = **11.7 kW ≈ 15.7 hp**. This is entirely reasonable for highway cruising in a sedan — typical vehicles produce 80–150 kW peak, so highway cruising at 25 m/s requires only 8–15% of maximum power.

### Full engine model operating point (extension)

When using Åström and Murray's full engine model with throttle input u ∈ [0, 1] in gear 4 (gear ratio α₄ = 12), the equilibrium throttle at v₀ = 25 m/s is found by solving:

$$\alpha_4 \cdot u_0 \cdot T(\alpha_4 \cdot v_0) = 468.8 \text{ N}$$

At ω = α₄ · v₀ = 12 × 25 = 300 rad/s, the engine torque is T(300) ≈ 183.8 N·m (computed from the torque curve in §8). Thus:

$$u_0 = \frac{468.8}{12 \times 183.8} = \frac{468.8}{2205.6} \approx 0.213$$

At the textbook's default operating point v₀ = 20 m/s, u₀ ≈ 0.169, consistent with the Åström–Murray published value.

---

## 5. Linearization

### Methodology

Linearization converts the nonlinear ODE into a linear approximation valid for small perturbations around the operating point. The approach follows **Seborg et al.'s Chapter 4 framework**: expand each nonlinear term in a first-order Taylor series about the operating point, then subtract the steady-state equation to isolate the deviation dynamics.

This is mathematically equivalent to the **Jacobian linearization** approach: the linearized coefficients are the partial derivatives of the right-hand side f(v, u, θ) evaluated at (v₀, u₀, θ₀).

### Step 1: Define deviation variables

$$\delta v = v - v_0, \quad \delta u = u - u_0, \quad \delta\theta = \theta - \theta_0$$

These deviation variables measure perturbations from the steady-state operating point. The transfer function will relate δV(s) and δU(s), not absolute V(s) and U(s).

### Step 2: Taylor-expand the quadratic drag term (critical step)

The drag force ½ρC_dAv² is the only nonlinear term in v. Its Taylor expansion about v₀ is:

$$v^2 = (v_0 + \delta v)^2 = v_0^2 + 2v_0 \cdot \delta v + (\delta v)^2$$

For small perturbations, the second-order term (δv)² is negligible compared to the first-order term 2v₀·δv. Quantitatively: at v₀ = 25 m/s with δv = 2.5 m/s (10% perturbation), (δv)² = 6.25 while 2v₀·δv = 125 — the neglected term is 5% of the retained term. The approximation:

$$v^2 \approx v_0^2 + 2v_0 \cdot \delta v$$

Substituting into the drag expression:

$$\tfrac{1}{2}\rho C_d A v^2 \approx \tfrac{1}{2}\rho C_d A v_0^2 + \rho C_d A v_0 \cdot \delta v$$

The first term is the **steady-state drag** (a constant, 312.0 N). The second term is the **linearized perturbation drag**, proportional to δv with the coefficient:

$$b_{\text{eff}} = \rho C_d A v_0 = 1.3 \times 0.32 \times 2.4 \times 25 = \textbf{24.96 N·s/m}$$

This coefficient acts as an **effective velocity-dependent damping** — it represents the incremental drag penalty for each m/s of speed perturbation at the operating point.

### Step 3: Linearize the grade and rolling resistance terms

**Grade force:** m · g · sin(θ) is expanded about θ₀ = 0:

$$m g \sin(\theta) \approx m g \sin(\theta_0) + m g \cos(\theta_0) \cdot \delta\theta = 0 + m g \cdot \delta\theta = m g \cdot \delta\theta$$

**Rolling resistance:** C_rr · m · g · cos(θ) is expanded about θ₀ = 0:

- Derivative with respect to v: ∂/∂v [C_rr · m · g · cos(θ)] = **0** (no velocity dependence)
- Derivative with respect to θ: −C_rr · m · g · sin(θ₀) = **0** at θ₀ = 0

**Rolling resistance therefore vanishes entirely from the linearized model.** It contributes only to the steady-state force balance (setting u₀) but has no influence on the perturbation dynamics. This is an elegant consequence of the deviation-variable formulation: constant forces that shift the operating point but do not depend on the state variable cancel completely during linearization.

### Step 4: Substitute and subtract the steady-state equation

The full ODE with all linearized terms:

$$m\frac{d(v_0 + \delta v)}{dt} = (u_0 + \delta u) - \tfrac{1}{2}\rho C_d A v_0^2 - \rho C_d A v_0 \cdot \delta v - C_{rr} m g - m g \cdot \delta\theta$$

Since v₀ is constant, d(v₀ + δv)/dt = d(δv)/dt. The steady-state equation states:

$$0 = u_0 - \tfrac{1}{2}\rho C_d A v_0^2 - C_{rr} m g$$

Subtracting the steady-state equation from the dynamic equation eliminates all constant terms (u₀, ½ρC_dAv₀², C_rr mg):

$$m\frac{d(\delta v)}{dt} = \delta u - \rho C_d A v_0 \cdot \delta v - m g \cdot \delta\theta$$

### Step 5: Write in standard coefficient form

Dividing through by m and using the notation of Åström and Murray:

$$\frac{d(\delta v)}{dt} = -a \cdot \delta v + b \cdot \delta u - c \cdot \delta\theta$$

where for the **simplified direct-force model**:

| Coefficient | Expression | Value at v₀ = 25 m/s | Units | Physical meaning |
|---|---|---|---|---|
| **a** | ρC_dAv₀/m | 24.96/1600 = 0.01560 | s⁻¹ | Velocity damping rate |
| **b** | 1/m | 1/1600 = 6.25 × 10⁻⁴ | (m/s²)/N | Input force sensitivity |
| **c** | g | 9.8 | m/s² | Grade disturbance sensitivity |

### Equivalence with Jacobian linearization

These coefficients are exactly the partial derivatives of the right-hand side function:

$$f(v, u, \theta) = \frac{1}{m}\left[u - \tfrac{1}{2}\rho C_d A v^2 - C_{rr} m g - m g \sin\theta\right]$$

evaluated at (v₀, u₀, θ₀):

$$a = -\frac{\partial f}{\partial v}\bigg|_0 = \frac{\rho C_d A v_0}{m}, \qquad b = \frac{\partial f}{\partial u}\bigg|_0 = \frac{1}{m}, \qquad c = -\frac{\partial f}{\partial \theta}\bigg|_0 = g \cos(\theta_0) = g$$

The Taylor series and Jacobian approaches are two descriptions of the same mathematical operation and produce identical results.

### Full engine model linearization (extension)

When using the Åström and Murray model with throttle input and engine torque curve, the driving force F = α_n · u · T(α_n · v) depends on v through the torque curve. The coefficient a acquires an additional term from this velocity dependence:

$$a_{\text{full}} = \frac{\rho C_d A v_0 - u_0 \cdot \alpha_4^2 \cdot T'(\alpha_4 v_0)}{m}$$

At v₀ = 25 m/s in gear 4, the engine torque slope T'(300) ≈ +0.103 N·m/(rad/s) (positive because ω = 300 < ω_m = 420, placing the operating point on the rising portion of the torque curve). The engine torque contribution partially offsets the aerodynamic damping:

$$u_0 \cdot \alpha_4^2 \cdot T'(\alpha_4 v_0) = 0.213 \times 144 \times 0.103 \approx 3.17 \text{ N·s/m}$$

The full linearization coefficients:

| Coefficient | Expression | Value at v₀ = 25 m/s |
|---|---|---|
| **a_full** | (24.96 − 3.17)/1600 | 0.01362 s⁻¹ |
| **b_full** | α₄ · T(α₄v₀)/m = 2205.6/1600 | 1.3785 (m/s²)/throttle |
| **c** | g | 9.8 m/s² |

**Cross-validation:** At the textbook's operating point v₀ = 20 m/s, these computations yield a = 0.01012, b = 1.3203, c = 9.8 — matching Åström and Murray's published values "a = 0.0101, b = 1.32, and b_g = 9.8" to four significant figures.

---

## 6. Transfer function derivation

### Laplace transform of the linearized ODE

Taking the Laplace transform of the linearized ODE with **zero initial conditions for deviation variables** (the system starts at equilibrium, so δv(0) = 0):

$$s \cdot \delta V(s) = -a \cdot \delta V(s) + b \cdot \delta U(s) - c \cdot \delta\Theta(s)$$

**Assumption:** Zero initial conditions for deviation variables. This does not mean v(0) = 0 — it means the system starts at its operating point, so v(0) = v₀ and δv(0) = v(0) − v₀ = 0. This assumption is valid whenever the system is in steady state before the input changes.

### Algebraic solution

Collecting δV(s) terms on the left:

$$(s + a) \cdot \delta V(s) = b \cdot \delta U(s) - c \cdot \delta\Theta(s)$$

Solving:

$$\delta V(s) = \frac{b}{s + a} \cdot \delta U(s) - \frac{c}{s + a} \cdot \delta\Theta(s)$$

This reveals the **superposition structure** of the linear system: the velocity deviation is the sum of contributions from the input change (through the process transfer function) and the disturbance (through the disturbance transfer function).

### Process transfer function

$$G(s) = \frac{\delta V(s)}{\delta U(s)}\bigg|_{\delta\Theta=0} = \frac{b}{s + a}$$

Converting to **standard first-order form** G(s) = K/(τs + 1) by dividing numerator and denominator by a:

$$G(s) = \frac{b/a}{(1/a)s + 1} = \frac{K}{\tau s + 1}$$

where:

$$K = \frac{b}{a} = \frac{1/m}{\rho C_d A v_0 / m} = \frac{1}{\rho C_d A v_0}$$

$$\tau = \frac{1}{a} = \frac{m}{\rho C_d A v_0}$$

### Numerical values — simplified direct-force model at v₀ = 25 m/s

$$\tau = \frac{m}{\rho C_d A v_0} = \frac{1600}{24.96} \approx \textbf{64.1 s}$$

$$K = \frac{1}{\rho C_d A v_0} = \frac{1}{24.96} \approx \textbf{0.0401 (m/s)/N}$$

$$\boxed{G(s) = \frac{0.0401}{64.1s + 1}}$$

### Disturbance transfer function

$$G_d(s) = \frac{\delta V(s)}{\delta\Theta(s)}\bigg|_{\delta U=0} = \frac{-c}{s + a} = \frac{-K_d}{\tau s + 1}$$

where:

$$K_d = \frac{c}{a} = \frac{g}{\rho C_d A v_0 / m} = \frac{mg}{\rho C_d A v_0} \approx \frac{1600 \times 9.8}{24.96} \approx \textbf{628 (m/s)/rad}$$

$$\boxed{G_d(s) = \frac{-628}{64.1s + 1}}$$

The **negative sign** indicates that a positive grade change (uphill) causes velocity to *decrease*, as expected physically.

The **same time constant τ** governs both the process and disturbance responses. This is a fundamental property of single-state-variable linear systems: the vehicle responds to hills on the same timescale as it responds to throttle changes. The denominator of both transfer functions comes from the system's characteristic equation (s + a = 0), which is determined by the plant dynamics, not by the input channel.

### Numerical values — full Åström–Murray engine model at v₀ = 25 m/s

$$\tau_{\text{full}} = \frac{1}{a_{\text{full}}} = \frac{1}{0.01362} \approx \textbf{73.4 s}$$

$$K_{\text{full}} = \frac{b_{\text{full}}}{a_{\text{full}}} = \frac{1.3785}{0.01362} \approx \textbf{101.2 (m/s)/throttle}$$

### Comparison with CTMS benchmark

At v₀ = 20 m/s with the full engine model, the textbook gives P(s) = 1.32/(s + 0.0101) with τ ≈ 99 s and K ≈ 131 (m/s)/throttle. The CTMS tutorial uses a different, simpler model: G(s) = 1/(1000s + 50) = 0.02/(20s + 1) with m = 1000 kg and lumped linear damping b = 50 N·s/m.

---

## 7. Dynamic characteristics

### System order

The plant is a **first-order system** with one pole and no finite zeros. The relative order (number of poles minus number of zeros) is 1.

### Process gain K = 0.0401 (m/s)/N

The DC gain tells you how much steady-state speed change results from a unit force change. At 25 m/s, an additional 25 N of engine force produces 25 × 0.0401 = 1.0 m/s of eventual steady-state speed increase. The gain is small because the vehicle operates at highway speed where aerodynamic damping is strong — each m/s of speed increase creates significant additional drag that limits the achievable speed change.

**Operating-point dependence:** K = 1/(ρC_dAv₀) is inversely proportional to v₀. At lower speeds, K increases (the same force produces a larger speed change). At very low speeds (v₀ → 0), K → ∞, and the system approaches a pure integrator — any constant force produces constant acceleration with no equilibrium.

### Time constant τ = 64.1 s

The time constant is the time for the vehicle to reach 63.2% of a new equilibrium speed after a throttle step. **64 seconds is exceptionally long by control-engineering standards** — far too slow for practical cruise control without feedback.

**Physical interpretation:** τ = m/(ρC_dAv₀) is the **ratio of vehicle inertia to aerodynamic damping**. This expression explains two intuitive driving phenomena:

1. **Heavier vehicles have longer time constants** because τ ∝ m. A 2000 kg SUV at the same speed has τ ≈ 80 s — 25% longer. More kinetic energy must be added or removed to change speed, while the aerodynamic damping force is independent of mass.

2. **Higher speeds produce shorter time constants** because τ ∝ 1/v₀. At 35 m/s, τ drops to ~46 s. The derivative of quadratic drag (d(v²)/dv = 2v) grows linearly with speed, producing stronger aerodynamic "self-correction" at highway speeds. At very low speeds, damping vanishes and the vehicle behaves as a pure integrator.

### Pole location

The single pole is at:

$$s = -\frac{1}{\tau} = -a = -0.01560 \text{ s}^{-1}$$

This pole lies in the **open left-half plane**, confirming the plant is **inherently open-loop stable**. Physical origin: aerodynamic drag increases with velocity, creating a restoring force that drives the system back toward equilibrium after any speed perturbation.

### Expected open-loop behavior

A step force change produces a first-order exponential approach to steady state:

$$\delta v(t) = K \cdot \delta u \cdot (1 - e^{-t/\tau})$$

| Time | % of final value | Velocity change for 500 N step |
|---|---|---|
| t = τ = 64 s | 63.2% | 12.7 m/s |
| t = 2τ = 128 s | 86.5% | 17.4 m/s |
| t = 3τ = 192 s | 95.0% | 19.1 m/s |
| t = 5τ = 320 s | 99.3% | 19.9 m/s |

A 500 N step produces K × 500 = 20.1 m/s of eventual steady-state speed change, but reaching 95% of this takes over 3 minutes — reinforcing the need for feedback control.

---

## 8. Model validation and sanity checks

### Dimensional consistency

Every term in the ODE must have dimensions of force (Newtons = kg·m/s²):

| Term | Dimensional analysis | Result |
|---|---|---|
| ½ρC_dAv² | [kg/m³]·[—]·[m²]·[m²/s²] | kg·m/s² ✓ |
| C_rr · m · g | [—]·[kg]·[m/s²] | kg·m/s² ✓ |
| m · g · sin(θ) | [kg]·[m/s²]·[—] | kg·m/s² ✓ |

Derived quantities:

| Quantity | Dimensional analysis | Result |
|---|---|---|
| τ = m/(ρC_dAv₀) | [kg] / ([kg/m³]·[m²]·[m/s]) | s ✓ |
| K = 1/(ρC_dAv₀) | 1 / ([kg/m³]·[m²]·[m/s]) | (m/s)/N ✓ |
| K_d = mg/(ρC_dAv₀) | [kg]·[m/s²] / ([kg/m³]·[m²]·[m/s]) | (m/s)/rad ✓ |

### Limiting-case verification

| Limit | τ behavior | K behavior | Physical interpretation | Correct? |
|---|---|---|---|---|
| v₀ → 0 | τ → ∞ | K → ∞ | No aerodynamic damping → pure integrator; constant force produces constant acceleration | ✓ |
| m → ∞ | τ → ∞ | K unchanged | Infinite inertia → vehicle never reaches steady state | ✓ |
| C_d → 0 | τ → ∞ | K → ∞ | No drag → pure integrator (rolling friction is constant, cannot establish velocity-dependent equilibrium) | ✓ |
| C_d → ∞ | τ → 0 | K → 0 | Enormous drag → instant equilibrium; no achievable speed change | ✓ |
| A → ∞ | τ → 0 | K → 0 | Enormous frontal area → same as very high drag | ✓ |

### Numerical plausibility

- **u₀ = 468.8 N at 90 km/h → P = 11.7 kW ≈ 15.7 hp:** Entirely reasonable for sedan highway cruising
- **τ = 64 s → 5τ ≈ 5 min to settle:** Consistent with everyday experience of gradually building or losing speed on long highway grades
- **K = 0.040 (m/s)/N → 25 N for 1 m/s increase:** Physically plausible — a 1 m/s speed increase requires overcoming ~25 N of additional drag at 25 m/s
- **K_d = 628 (m/s)/rad → a 4° grade causes ~44 m/s open-loop drop:** This is unrealistically large because it assumes no throttle adjustment; it correctly motivates the need for feedback control

### Cross-validation with published results

The textbook values at v₀ = 20 m/s (a = 0.0101, b = 1.32, b_g = 9.8) have been independently reproduced to four significant figures using the derivation above, confirming the mathematical correctness of the linearization procedure.

---

## 9. Common pitfalls and errors

### Derivation mistakes students make

**1. Sign error in drag force.** Writing m(dv/dt) = u + ½ρC_dAv² instead of subtracting drag. This reverses the sign of the damping coefficient a, producing a right-half-plane pole (s = +|a|) and a falsely unstable system. **Check:** Drag opposes motion — increasing v must increase the retarding force, so drag subtracts in the force balance.

**2. Failing to linearize v².** Students sometimes leave v² in the ODE and attempt to take a Laplace transform directly. The Laplace transform is defined only for linear operations; L{v²} ≠ V(s)² or any simple algebraic expression. The v² term *must* be expanded as v₀² + 2v₀·δv before transforming. **This is the single most common error and the question most likely to be asked in a viva.**

**3. Confusing deviation and absolute variables.** The transfer function G(s) = K/(τs+1) relates *deviation* quantities δV(s) and δU(s), not absolute V(s) and U(s). Zero initial conditions apply to δv(0) = 0 (the system starts at its operating point), not to v(0) = 0. The steady-state subtraction step is what creates the deviation-variable framework.

**4. Forgetting the steady-state subtraction.** After Taylor-expanding every term, the zeroth-order terms (v₀², u₀, C_rr mg) must cancel by the equilibrium condition. Students who linearize correctly but skip this subtraction end up with residual constant terms in their deviation ODE, producing incorrect transfer functions.

**5. Dimensional inconsistencies.** Mixing km/h with m/s, degrees with radians for θ, or forgetting the factor of ½ in the drag formula. Every term in the force balance must carry units of Newtons. A quick dimensional check at each stage prevents cascading errors.

**6. Ignoring the engine torque's velocity dependence.** In the Åström and Murray model, the coefficient a includes a term from ∂T/∂ω because engine torque varies with speed. Students who treat F_engine as independent of v compute a = ρC_dAv₀/m instead of the full expression a_full = [ρC_dAv₀ − u₀α_n²T'(α_nv₀)]/m. The error is quantitatively significant: at v₀ = 20 m/s, neglecting the engine term gives a = 0.0125 instead of 0.0101 — a **24% overestimate**.

**7. Writing incorrect transfer function forms.** Common errors:
- G(s) = Ks/(τs+1) — spurious zero
- G(s) = K/(Kτs+1) — K incorrectly placed in denominator
- G(s) = K/(τs−1) — wrong sign producing instability

The correct standard form is G(s) = K/(τs+1) with both K > 0 and τ > 0 for a stable first-order system.

### Modeling inconsistencies to avoid

- Using the CTMS parameters (m = 1000, b = 50) in one place and the Åström–Murray parameters (m = 1600, C_d = 0.32) in another. **Choose one set and use it consistently.**
- Claiming the model includes "engine dynamics" while using the direct-force input (no engine transfer function is present unless explicitly added as a cascade).
- Presenting linearization results without demonstrating the linearization step — professors can detect when transfer function parameters are looked up rather than derived.

---

## 10. Preparation for next section

### Outputs for controller design (Section 3)

The following results from this derivation are carried directly into the controller design phase:

**Primary plant model (simplified direct-force input):**

$$G(s) = \frac{K}{\tau s + 1} = \frac{0.0401}{64.1s + 1} \qquad \text{at } v_0 = 25 \text{ m/s}$$

- Pole at s = −0.01560 (stable, left-half plane)
- No finite zeros; relative order = 1
- DC gain: K = 0.0401 (m/s)/N
- Time constant: τ = 64.1 s

**Disturbance transfer function:**

$$G_d(s) = \frac{-628}{64.1s + 1}$$

- Same pole as G(s)
- DC disturbance gain: K_d = 628 (m/s)/rad (very large — disturbances are severe)

**Full Åström–Murray engine-coupled model:**

$$G_{\text{full}}(s) = \frac{101.2}{73.4s + 1} \qquad \text{(throttle input, } v_0 = 25 \text{ m/s)}$$

**Nonlinear ODE (for simulation):**

$$m\frac{dv}{dt} = u - \tfrac{1}{2}\rho C_d A v^2 - C_{rr} m g - m g \sin(\theta)$$

with steady-state operating point u₀ = 468.8 N at v₀ = 25 m/s.

### What Section 3 should address

The controller design section should use the process transfer function G(s) = K/(τs + 1) as the starting point and address:

1. **Why this plant needs feedback control** — the 64-second open-loop time constant is too slow, and the disturbance gain is too large for uncontrolled operation
2. **What controller structure is appropriate** — the first-order plant with no zeros and no dead time dictates the controller choice
3. **How to tune the controller** — using the derived K and τ values with systematic tuning methods (IMC, direct synthesis, or ITAE correlations)
4. **What performance specifications to target** — quantitative targets for rise time, overshoot, settling time, and steady-state error based on the CTMS benchmark and cruise control application requirements

### Engine dynamics as an optional cascade extension

If engine dynamics are included, the plant becomes a second-order cascade:

$$G_{\text{plant}}(s) = G_e(s) \cdot G_v(s) = \frac{K_e \cdot K_v}{(\tau_e s + 1)(\tau_v s + 1)}$$

Jeffrey Kantor's CBE 30338 materials use G_e = 600/(s+1) with τ_e = 1 s and G_v = 0.5/(5s+1) with τ_v = 5 s. Åström and Murray model the engine as a **static nonlinearity** (torque curve) rather than a dynamic lag:

$$T(\omega) = T_m\left[1 - \beta\left(\frac{\omega}{\omega_m} - 1\right)^2\right]$$

with T_m = 190 N·m (peak torque), ω_m = 420 rad/s (≈ 4000 RPM), and β = 0.4 (rolloff parameter). At the v₀ = 25 m/s operating point in gear 4 (ω = 300 rad/s), the engine operates below peak torque, producing T(300) ≈ 183.8 N·m with a positive slope T'(300) ≈ 0.103 N·m/(rad/s). This positive slope means the engine provides a small **anti-damping effect** — as speed increases, torque increases slightly, partially offsetting aerodynamic drag growth. This effect is embedded in the coefficient a_full and explains why τ_full (73.4 s) exceeds τ_simplified (64.1 s): the reduced effective damping allows the system to oscillate more slowly toward equilibrium.

For a CHE F342 project, the recommended approach is to present the simplified direct-force model as the **primary derivation** (cleanly separating linearization pedagogy from engine modeling details), then introduce the engine torque curve and/or first-order lag as **extensions** that demonstrate how cascade transfer functions arise naturally from subsystem modeling.