# Controller design and tuning strategy for car cruise control

**The cruise control plant derived in Section 2 is a first-order stable system with a 64-second time constant and a DC gain of 0.040 (m/s)/N—an exceptionally sluggish plant by control-engineering standards, where the dominant challenge is not stability but speed of response and disturbance rejection.** This section designs P, PI, and PID controllers for the linearized plant G(s) = K/(τs + 1), compares their performance characteristics, selects a tuning methodology appropriate for CHE F342, and addresses practical implementation concerns. All controller designs are anchored to the specific numerical parameters derived previously: τ = 64.1 s, K = 0.0401 (m/s)/N for the simplified direct-force model at v₀ = 25 m/s, and τ_full = 73.4 s, K_full = 101.2 (m/s)/throttle for the Åström and Murray engine-coupled model. The disturbance transfer function G_d(s) = −628/(64.1s + 1) governs the vehicle's response to road grade changes and defines the disturbance rejection requirements.

---

## 1. Control objective restatement

### The feedback control problem

The control objective is to regulate vehicle velocity v at a driver-specified set point v_ref despite unmeasured disturbances (road grade θ, wind gusts), parameter uncertainty (mass variations from passengers/cargo, tire pressure changes affecting C_rr), and the inherently sluggish open-loop dynamics of the plant.

In deviation-variable form, the problem is: design a controller G_c(s) such that the closed-loop system

$$\delta V(s) = \frac{G_c(s) \cdot G(s)}{1 + G_c(s) \cdot G(s)} \cdot \delta V_{\text{ref}}(s) + \frac{G_d(s)}{1 + G_c(s) \cdot G(s)} \cdot \delta\Theta(s)$$

satisfies quantitative performance specifications for both the set-point tracking channel and the disturbance rejection channel.

### Quantitative performance specifications

Following the University of Michigan CTMS benchmark and standard automotive cruise control expectations:

| Specification | Target value | Rationale |
|---|---|---|
| **Rise time** (10→90%) | < 5 seconds | Driver comfort; perceptible acceleration within a few seconds |
| **Overshoot** | < 10% | Avoid speed limit violations; passenger comfort |
| **Settling time** (±2%) | < 15 seconds | Reach target speed before typical highway merge/exit |
| **Steady-state error** | < 2% (ideally zero) | Maintain set speed within speedometer resolution |
| **Disturbance rejection** | < 5% speed deviation for a 4° grade change | Maintain speed on typical highway inclines |

### Qualitative performance goals

Beyond numerical targets, cruise control must provide **smooth, monotonic speed transitions** (no jerky acceleration), **robustness to mass uncertainty** (the controller should work for vehicle mass ranging from 1200 to 2000 kg without retuning), and **actuator effort within physical limits** (throttle position u ∈ [0, 1], corresponding to finite engine force). The controller should also degrade gracefully under model mismatch rather than becoming unstable.

---

## 2. Plant behavior and control implications

### Stability characteristics

The plant G(s) = K/(τs + 1) = 0.0401/(64.1s + 1) has a single pole at s = −1/τ = −0.0156 s⁻¹ in the **open left-half plane**. The plant is **inherently open-loop stable**—a bounded throttle input always produces a bounded velocity response that settles to a finite steady-state value. This stability arises physically from the velocity-dependent aerodynamic damping: any speed perturbation creates a restoring drag force proportional to δv that drives the system back toward equilibrium.

**Control implication:** Stability is not the design challenge. The plant will not go unstable for any finite positive controller gain. The challenge is entirely about *performance*—making the 64-second sluggish response fast enough for practical cruise control while maintaining adequate damping.

### Pole-zero structure

- **One real pole** at s = −0.0156 (simplified model) or s = −0.0136 (full engine model)
- **No finite zeros** — the plant is *minimum phase* and has no right-half-plane zeros
- **Relative order** = 1 (one more pole than zeros)

The absence of zeros means the plant imposes no fundamental limitations on achievable closed-loop bandwidth from the zero structure. There are no non-minimum-phase penalties, no undershoot constraints, and no Bode integral limitations arising from right-half-plane zeros. This makes the plant exceptionally well-suited to classical feedback control—any desired closed-loop pole location is theoretically achievable with sufficient controller gain.

### Open-loop behavior

The open-loop step response is a simple first-order exponential approach to steady state with time constant τ ≈ 64 s. Key characteristics:

- **63.2% of final value reached at t = 64 s** — far too slow for practical cruise control
- **95% of final value at t = 3τ ≈ 192 s** (over 3 minutes)
- **99% settling at t = 5τ ≈ 320 s** (over 5 minutes)

A step force change of 500 N produces only 0.040 × 500 = 20 m/s of eventual steady-state speed change, but reaching 90% of this takes approximately 147 seconds. This sluggishness is the primary control design driver.

The **open-loop gain** at DC is K = 0.040 (m/s)/N, meaning the system has low sensitivity to input changes. This is a consequence of operating at highway speed where aerodynamic damping is strong. At lower speeds, K increases (approaching infinity as v₀ → 0), and the system becomes more responsive but also more integrator-like.

### Disturbance sensitivity

The disturbance transfer function G_d(s) = −628/(64.1s + 1) has a **very large DC gain** of 628 (m/s)/rad. A seemingly small road grade of θ = 0.05 rad (≈ 2.9°, a moderate highway incline) would produce an open-loop steady-state velocity drop of 628 × 0.05 = **31.4 m/s** — the vehicle would practically stop on the hill if the throttle were not adjusted. This massive disturbance sensitivity is the strongest practical argument for feedback control and, specifically, for integral action.

The disturbance and process transfer functions share the same denominator (same pole at s = −1/τ), meaning the vehicle responds to grade changes on the same 64-second timescale as throttle changes. The controller must act faster than this natural timescale to reject disturbances before they cause unacceptable speed deviations.

### Implications for controller selection

The plant analysis reveals four key design drivers:

1. **The time constant must be shortened by at least an order of magnitude** — from 64 s to roughly 3–5 s to meet the rise-time specification. This requires high loop gain.
2. **Steady-state error elimination is essential** — the disturbance gain is so large that even small grade changes cause enormous open-loop velocity drops. Integral action is strongly motivated.
3. **There is no stability challenge** — the plant is stable first-order with no zeros; any linear controller will produce a stable closed-loop for positive gains.
4. **Derivative action adds limited value** — the plant is already first-order; adding a derivative term to a first-order system provides lead compensation that can improve speed of response marginally, but the noise amplification cost may not be justified.

These observations strongly favor a **PI controller** as the primary design choice, with P and PID included for comparison.

---

## 3. P controller design

### Design rationale

A proportional controller G_c(s) = K_p applies a corrective force proportional to the velocity error e = v_ref − v. For the first-order plant G(s) = K/(τs + 1), the closed-loop transfer function with proportional control is:

$$G_{\text{CL}}(s) = \frac{K_p K}{\tau s + 1 + K_p K} = \frac{K_p K / (1 + K_p K)}{[\tau / (1 + K_p K)] s + 1}$$

This is still a **first-order system** with:

- **Closed-loop time constant:** τ_CL = τ/(1 + K_pK)
- **Closed-loop DC gain:** K_CL = K_pK/(1 + K_pK) < 1

### Expected closed-loop behavior

The P controller shortens the time constant by the factor (1 + K_pK). To reduce τ from 64.1 s to approximately 5 s (a closed-loop time constant comparable to the desired transient speed), we need:

$$1 + K_p K \approx \tau / 5 = 12.8 \implies K_p \approx 11.8 / 0.040 \approx 295 \text{ N/(m/s)}$$

With K_p = 295, the closed-loop time constant becomes τ_CL ≈ 5.0 s and the closed-loop DC gain is K_CL = 295 × 0.040/(1 + 295 × 0.040) = 11.8/12.8 = **0.922**. This means a **7.8% steady-state error** for a unit step reference — the vehicle would settle at 92.2% of the target speed, never reaching v_ref.

> **Note on rise time vs. time constant:** For a first-order system, the rise time (10→90%) is t_r = 2.2 × τ_CL. At τ_CL = 5 s, the actual rise time is approximately **11 s**, which exceeds the CTMS specification of < 5 s. The CTMS specification was formulated for a lighter plant (m = 1000 kg, τ = 20 s). Meeting t_r < 5 s on our heavier plant requires τ_CL < 2.3 s (i.e., K_p > 700 or λ < 2.3 s), which is achievable but trades robustness for speed. This tradeoff is examined in the simulation phase (Section 4, Scenario E3: λ sweep).

### Strengths

- **Simplicity:** Single tuning parameter; trivial to implement and debug
- **Stable for all K_p > 0:** The closed-loop pole remains at s = −(1 + K_pK)/τ, always in the left-half plane
- **No oscillatory behavior:** The closed-loop is first-order; there is no possibility of overshoot
- **Immediate response improvement:** Even moderate K_p values dramatically reduce the time constant

### Weaknesses for this plant

- **Persistent steady-state error:** The offset e_ss = 1/(1 + K_pK) is inherent to P-only control of a type-0 plant. Reducing it to 2% would require K_p ≈ 49/K = 1225 N/(m/s), which corresponds to a throttle force correction of 1225 N per m/s of error — demanding roughly 2.5× the steady-state cruising force for a 1 m/s error. While theoretically stable, such gains may cause actuator saturation on even modest transients.
- **Disturbance rejection limited by gain:** For a grade disturbance, the closed-loop steady-state velocity deviation is ΔV_ss = −K_d·Δθ/(1 + K_pK). With K_p = 295 and a 4° grade (Δθ = 0.07 rad), ΔV_ss ≈ −628 × 0.07/12.8 ≈ **−3.4 m/s**, a 13.6% drop at 25 m/s. This exceeds the 5% specification.
- **The error is speed-dependent:** Because K varies with operating point (K ∝ 1/v₀), the steady-state error changes with cruise speed—an unacceptable inconsistency for a real system.

### Theoretical tuning methodology

For a first-order plant with P control, tuning is straightforward: choose K_p to achieve the desired closed-loop time constant.

$$K_p = \frac{\tau / \tau_{\text{CL,desired}} - 1}{K}$$

This direct relationship leaves only the tradeoff between speed (smaller τ_CL) and steady-state error (larger K_p reduces error but never eliminates it). There is no stability concern, no resonance peak, and no gain margin issue for a first-order plant with P control.

### Expected limitations in cruise control application

P-only control is **inadequate** for cruise control because:

1. Drivers expect zero steady-state speed error (the speedometer should read exactly the set speed)
2. The disturbance rejection is insufficient for typical highway grades
3. The cruise speed-dependent offset is inconsistent and would confuse users

**Verdict:** P control is useful as a didactic baseline and for demonstrating the limitations of proportional-only feedback, but it should not be the recommended controller for the final design.

---

## 4. PI controller design

### Design rationale

A PI controller G_c(s) = K_p(1 + 1/(τ_Is)) = K_p(τ_Is + 1)/(τ_Is) introduces integral action that accumulates error over time, driving the steady-state error to zero for step inputs in both the reference and disturbance channels.

The open-loop transfer function with PI control becomes:

$$L(s) = G_c(s) \cdot G(s) = \frac{K_p(τ_I s + 1)}{τ_I s} \cdot \frac{K}{τ s + 1} = \frac{K_p K (\tau_I s + 1)}{\tau_I s (\tau s + 1)}$$

The integrator (1/s term) makes the loop transfer function **type 1**, guaranteeing zero steady-state error for step reference and step disturbance inputs by the final value theorem.

### Why integral action helps

The core problem identified in the P controller analysis — persistent offset that worsens with disturbances — is precisely what integral action corrects. The integral term accumulates the time history of the error signal, continuously adjusting the control effort until the error is driven to zero. Physically, the integrator "remembers" that the vehicle has been below the set speed and keeps increasing throttle until the error vanishes, even if the required steady-state throttle differs from the nominal u₀ due to an unknown grade disturbance.

For the cruise control plant specifically:

1. **Zero steady-state offset** for both set-point changes and grade disturbances — the vehicle reaches and holds exactly v_ref
2. **Automatic compensation** for uncertain parameters — the integrator absorbs steady-state model errors in C_d, C_rr, m, etc.
3. **Type-1 system** tracks ramp references with bounded error — relevant if the set-point transitions slowly rather than as steps

### Closed-loop analysis

The closed-loop transfer function is:

$$G_{\text{CL}}(s) = \frac{K_p K (\tau_I s + 1)}{\tau \tau_I s^2 + (1 + K_p K)\tau_I s + K_p K}$$

This is a **second-order system** with a zero at s = −1/τ_I. The characteristic equation ττ_Is² + (1 + K_pK)τ_Is + K_pK = 0 has two roots (closed-loop poles). Dividing by ττ_I to write in standard form:

$$s^2 + \frac{1 + K_p K}{\tau} s + \frac{K_p K}{\tau \tau_I} = 0$$

Comparing with s² + 2ζω_ns + ω_n²:

$$\omega_n = \sqrt{\frac{K_p K}{\tau \tau_I}}, \qquad \zeta = \frac{(1 + K_p K)}{2\tau} \cdot \frac{1}{\omega_n} = \frac{(1 + K_p K)}{2\sqrt{K_p K \cdot \tau / \tau_I}}$$

The control designer now has **two tuning knobs** (K_p and τ_I) to independently set the natural frequency (speed of response) and damping ratio (overshoot behavior).

### Expected performance improvements over P

| Metric | P controller (K_p = 295) | PI controller (properly tuned) |
|---|---|---|
| Steady-state error (step reference) | 7.8% | **0%** |
| Steady-state error (4° grade disturbance) | −3.4 m/s | **0 m/s** |
| Response order | First-order (no overshoot) | Second-order (possible overshoot) |
| Rise time | ~5 s | ~5 s (tunable) |
| Robustness to parameter changes | Offset varies with K | Offset always eliminated |

### Potential drawbacks

- **Overshoot is now possible:** The second-order closed-loop can produce overshoot if ζ < 1. With aggressive integral action (small τ_I), the system can become underdamped, causing the vehicle to overshoot v_ref before settling. For cruise control, overshoot translates to momentarily exceeding the speed limit—an undesirable outcome.
- **Integrator windup:** If the engine throttle saturates (u = 1), the integrator continues accumulating error, leading to large control signals that cause overshoot when the constraint is released. Anti-windup measures are necessary for practical implementation.
- **Slower initial response compared to pure P at the same K_p:** The integral term builds up gradually, so the early transient response is dominated by the proportional term. The integral contribution manifests over the timescale of τ_I.

### Theoretical tuning methodology

For a PI controller on a first-order plant, the classical **pole-zero cancellation** approach sets τ_I = τ (the process time constant), which cancels the plant pole with the controller zero. The closed-loop then reduces to:

$$G_{\text{CL}}(s) = \frac{K_p K}{\tau s + K_p K} = \frac{1}{(\tau / K_p K) s + 1}$$

A first-order response with closed-loop time constant τ_CL = τ/(K_pK). To achieve τ_CL = 5 s:

$$K_p = \frac{\tau}{K \cdot \tau_{\text{CL}}} = \frac{64.1}{0.040 \times 5} = 320 \text{ N/(m/s)}$$

This pole-zero cancellation approach is the basis of the **Internal Model Control (IMC)** tuning method and yields a clean first-order closed-loop response with zero overshoot and zero steady-state error. It is equivalent to IMC with tuning parameter λ = τ_CL = 5 s.

**Alternative tuning** without cancellation: choosing τ_I < τ (faster integral action) speeds up disturbance rejection at the cost of introducing oscillatory modes.

**CTMS benchmark comparison:** The CTMS benchmark uses K_p = 800 and K_i = K_p/τ_I = 40, giving τ_I = 20 s on a plant with τ = 20 s — this is exact pole-zero cancellation, the same structural approach as our IMC design. The difference is in aggressiveness: the CTMS sets λ = τ/(K_p K) = 20/(800 × 0.02) = 1.25 s, yielding a rise time of 2.2 × 1.25 ≈ 2.75 s (meeting their < 5 s specification). Our λ = 5 s design is more conservative, prioritizing robustness over speed — a deliberate choice justified by our 3.2× heavier plant and the corresponding rise time implications discussed above.

---

## 5. PID controller design

### Design rationale

A PID controller G_c(s) = K_p(1 + 1/(τ_Is) + τ_Ds) adds derivative action to the PI structure. The derivative term reacts to the *rate of change* of the error signal, providing anticipatory control action — it "predicts" where the error is heading and applies a corrective signal proportional to the rate of approach.

The open-loop transfer function becomes:

$$L(s) = K_p \cdot \frac{\tau_I \tau_D s^2 + \tau_I s + 1}{\tau_I s} \cdot \frac{K}{\tau s + 1}$$

The PID controller adds two zeros and one integrator pole to the loop transfer function, for a total of one integrator pole, two controller zeros, and one plant pole.

### Role of derivative action in this plant

For a first-order plant, derivative action provides **phase lead** that can speed up the transient response beyond what PI alone achieves. The derivative term detects sudden changes in velocity (e.g., the vehicle starting to decelerate as it hits a grade) and responds *before* the error accumulates enough for the proportional and integral terms to react.

Specifically, when the vehicle encounters a hill and begins decelerating, dv/dt becomes negative. The derivative term contributes K_pτ_D(dv/dt) to the control effort, immediately increasing throttle in proportion to the deceleration rate. This is analogous to a driver who feels the car slowing and presses the accelerator harder before the speedometer drops significantly.

### When derivative action helps

Derivative action provides the greatest benefit for plants with:

- **Higher-order dynamics** — adding lead compensation to compensate for phase lag from multiple poles
- **Large transport delays** — providing predictive action to overcome dead time
- **Dominant oscillatory modes** — damping underdamped resonances

For the cruise control system, derivative action offers modest benefit in the following scenarios:

1. **Second-order cascade model** (with engine dynamics): When the plant includes an engine lag G_e(s) = K_e/(τ_es + 1), the overall plant is second-order, and derivative action can improve the phase margin, enabling higher proportional gain and faster response.
2. **Rapid disturbance rejection**: The derivative detects the onset of a grade disturbance from the initial deceleration, providing faster correction than PI alone.

### When derivative action hurts

1. **Measurement noise amplification** — this is the dominant concern. The derivative term has a transfer function proportional to s, which amplifies high-frequency noise. Vehicle speed measurements from wheel encoders contain quantization noise and vibration artifacts. The derivative of a noisy signal is much noisier: if the velocity measurement has noise amplitude σ, the derivative has noise amplitude approximately σ/Δt where Δt is the sampling interval. For a 10 Hz speed sensor with ±0.1 m/s noise, the unfiltered derivative could produce ±1 m/s² noise spikes — larger than the typical acceleration signals the controller is trying to detect.

2. **Derivative kick** — a step change in the reference v_ref produces an impulse (theoretically infinite derivative) at the instant of the step. This spike drives the throttle to its physical limit instantaneously, creating a harsh jerk felt by passengers. This is mitigated by applying derivative action only to the process variable (velocity), not to the error signal: use D(s) = −K_pτ_Ds · V(s) rather than D(s) = K_pτ_Ds · E(s). This is called **derivative on PV** (process variable) and is standard practice.

3. **Marginal benefit on a first-order plant** — the plant has only one pole; there is no excess phase lag that demands lead compensation. The derivative is solving a problem that barely exists for this plant structure.

### Noise sensitivity considerations

The practical derivative term is always implemented with a **derivative filter**:

$$D(s) = \frac{K_p \tau_D s}{(\tau_D / N) s + 1}$$

where N is the filter coefficient (typically 5–20). This limits the derivative gain at high frequencies to K_pN (instead of infinity), rolling off above ω = N/τ_D. The filter adds a pole that makes the controller proper and physically realizable.

For cruise control, the primary noise source is velocity measurement. Wheel-based speed sensors (ABS ring encoders) have resolution of approximately 0.1 m/s at highway speeds, with higher-frequency content from wheel eccentricity and road surface roughness. A derivative filter with N = 10 is a reasonable starting point: it provides derivative action up to about 10/τ_D rad/s while attenuating noise above this frequency.

### Practical implementation concerns

- **Three tuning parameters** (K_p, τ_I, τ_D) increase the tuning complexity relative to PI, with diminishing performance returns for this plant.
- The derivative time constant τ_D should be set significantly smaller than τ_I (typically τ_D = τ_I/4 to τ_I/8) to avoid creating closely spaced zeros that interact poorly with the plant pole.
- The **ideal PID** structure is not physically realizable (improper transfer function). The series or parallel form with derivative filter must be used in practice.
- For the first-order plant, the closed-loop with PID control has **three poles and two zeros** — overshoot and oscillatory behavior depend on all five root locations.

---

## 6. Controller comparison and recommendation

### Systematic comparison for this cruise control system

| Performance metric | P | PI | PID |
|---|---|---|---|
| **Steady-state error (step ref)** | Persistent (≈ 8% at K_p=295) | **Zero** | **Zero** |
| **Steady-state error (grade disturbance)** | Persistent (13.6% at 4° grade) | **Zero** | **Zero** |
| **Rise time (10→90%)** | ~11 s at τ_CL = 5 (tunable via K_p) | ~11 s at λ = 5 (tunable via λ) | ~9–11 s (marginally faster) |
| **Overshoot** | **0%** (first-order CL) | 0–15% (depends on tuning) | 0–10% (better damping possible) |
| **Disturbance rejection speed** | Moderate | Good | **Slightly better** |
| **Noise sensitivity** | **None** | **None** | Significant (derivative) |
| **Tuning complexity** | 1 parameter | 2 parameters | 3 parameters + filter |
| **Robustness** | High (simple, stable) | **High** (integral handles uncertainty) | Moderate (derivative sensitive) |
| **Actuator effort** | Moderate | Moderate (integral builds gradually) | Higher (derivative spikes) |
| **Academic value** | Baseline only | Core design | Diminishing returns for 1st-order |

### Tracking performance

All three controllers can achieve comparable rise times by adjusting K_p. However, only PI and PID achieve zero steady-state tracking error. For cruise control, precise speed holding is a non-negotiable requirement, immediately disqualifying the P controller for the final design.

### Disturbance rejection

This is where the controllers differ most meaningfully. The P controller can only attenuate disturbances by the factor 1/(1 + K_pK), leaving a permanent velocity offset on hills. The PI controller fully rejects step disturbances (zero steady-state error), though the transient deviation depends on K_p and τ_I. The PID controller provides marginally faster initial disturbance rejection due to derivative action detecting the deceleration onset, but the steady-state performance is identical to PI.

Quantitatively, for a 4° (0.07 rad) step grade change with PI tuned via IMC (τ_CL = 5 s, τ_I = τ = 64.1 s):

- **Peak velocity deviation** ≈ −K_d · Δθ · (τ_CL/τ) ≈ −628 × 0.07 × (5/64.1) ≈ **−3.4 m/s** (transient)
- **Steady-state velocity deviation** = **0 m/s** (integral action eliminates offset)
- **Recovery time** ≈ 3–5 × τ_CL ≈ **15–25 s**

### Recommendation

**The PI controller is the recommended design for this cruise control system.** The justification is:

1. **Zero steady-state error** for both set-point and disturbance inputs — the minimum requirement for cruise control
2. **Adequate transient performance** — rise time and settling time specifications are achievable with proper tuning
3. **No noise amplification** — unlike PID, PI does not differentiate the noisy velocity signal
4. **Minimal tuning complexity** — two parameters (K_p, τ_I) rather than three plus a filter
5. **Plant structure alignment** — the first-order plant does not benefit meaningfully from derivative action; PI matches the intrinsic system order
6. **Academic precedent** — Åström and Murray, CTMS, Seborg et al., and Kantor all use PI as the primary cruise control controller with strong results
7. **Robustness** — integral action inherently compensates for parameter uncertainty (mass, drag coefficient, rolling resistance), making the controller practical across operating conditions

PID should be included in the simulation study as a **comparison case** to quantify the (expected marginal) improvement from derivative action and to demonstrate understanding of the full PID structure. This comparison strengthens the academic submission.

---

## 7. Tuning methodology selection

### Internal Model Control (IMC)

**Mechanism:** IMC-based PID tuning uses an explicit process model to derive controller parameters that place the closed-loop poles at a desired location parameterized by a single tuning knob λ (the desired closed-loop time constant).

For a first-order plant G(s) = K/(τs + 1), the IMC-PI controller parameters are:

$$K_p = \frac{\tau}{K\lambda}, \qquad \tau_I = \tau$$

where λ is the desired closed-loop time constant. The integral time equals the process time constant (pole-zero cancellation), and the proportional gain is determined solely by λ.

| Aspect | Assessment |
|---|---|
| **Suitability for this plant** | **Excellent** — designed specifically for first-order (and FOPDT) systems |
| **Advantages** | Single tuning parameter λ; guarantees zero overshoot when τ_I = τ; physically interpretable; closed-loop time constant equals λ; robust to model mismatch if λ is conservative |
| **Disadvantages** | Requires process model (we have one); performance degrades if model is significantly wrong; pole-zero cancellation produces slow disturbance rejection if τ is large |
| **Tuning aggressiveness** | Directly controlled by λ — smaller λ = more aggressive; typical range: 0.1τ ≤ λ ≤ τ |
| **Academic defensibility** | **High** — taught in Seborg et al. Chapter 12; directly connects to model-based control philosophy of CHE F342; demonstrates mastery of transfer function concepts |

**For this plant:** With τ = 64.1 s, choosing λ = 5 s gives K_p = 64.1/(0.040 × 5) = 320 N/(m/s) and τ_I = 64.1 s. For the engine-coupled model with K = 101 (m/s)/throttle, K_p = 73.4/(101 × 5) = 0.145 throttle/(m/s).

### Ziegler–Nichols (ZN)

**Mechanism:** The ZN method comes in two forms: (1) the process reaction curve method (open-loop step test), and (2) the ultimate gain method (closed-loop oscillation test). Both were designed for industrial process control with deadtime-dominant plants.

For a first-order system *without dead time*, the ZN method has specific limitations:

| Aspect | Assessment |
|---|---|
| **Suitability for this plant** | **Poor to moderate** — ZN was designed for FOPDT (first-order plus dead time) plants; the cruise control plant has negligible dead time |
| **Advantages** | Widely known; no model needed for ultimate-gain method; quick initial tuning |
| **Disadvantages** | Designed for quarter-decay-ratio response (25% overshoot) — too aggressive for cruise control comfort; tuning rules assume significant dead time; without dead time, the reaction curve method produces degenerate results (the tangent line at the inflection point has no meaningful intercept) |
| **Tuning aggressiveness** | **Very aggressive** — designed for quarter-decay-ratio (ζ ≈ 0.22), which produces ≈25% overshoot. Unacceptable for cruise control comfort |
| **Academic defensibility** | **Moderate** — every student knows ZN, but applying it to a dead-time-free first-order plant is technically inappropriate. The examiner may question why ZN was used when the plant model is known (model-based methods are preferred when a model is available) |

**For this plant:** If the engine lag τ_e ≈ 1 s is treated as an approximation to dead time (Siebert's approximation for small lags), ZN can be applied with L = τ_e = 1 s and T = τ = 64.1 s. The resulting PI parameters are: K_p = 0.9T/(KL) = 0.9 × 64.1/(0.040 × 1) ≈ 1442 N/(m/s), τ_I = L/0.3 = 3.3 s. These values are **extremely aggressive** — the proportional gain is 4.5× higher than the IMC design, and the integral time is 19× faster. This will produce large overshoot and oscillatory behavior, confirming that ZN is poorly suited to this plant.

### Cohen–Coon

**Mechanism:** Cohen–Coon tuning is a refinement of the ZN reaction curve method that explicitly accounts for the dead-time-to-time-constant ratio θ/τ. It provides improved tuning for plants with small θ/τ ratios compared to ZN.

| Aspect | Assessment |
|---|---|
| **Suitability for this plant** | **Moderate** — better than ZN for small deadtime ratios, but still assumes measurable dead time |
| **Advantages** | Less aggressive than ZN for plants with small θ/τ; produces less overshoot |
| **Disadvantages** | Still requires identification of dead time (ambiguous for this plant); designed for quarter-decay criterion; more complex formulas with limited intuitive benefit over IMC |
| **Tuning aggressiveness** | Moderate to aggressive (targets roughly quarter-decay) |
| **Academic defensibility** | **Moderate** — known but less commonly taught in process control than IMC or ZN; may be seen as a sensible middle ground |

### ITAE (Integral of Time-weighted Absolute Error)

**Mechanism:** ITAE tuning minimizes the integral ∫₀^∞ t|e(t)| dt, which penalizes errors that persist over time more heavily than early transient errors. Correlations relate ITAE-optimal PID parameters to the plant model parameters.

| Aspect | Assessment |
|---|---|
| **Suitability for this plant** | **Good** — applicable to FOPDT models; produces well-damped responses with low overshoot |
| **Advantages** | Objective optimality criterion; excellent balance of speed and damping; low overshoot (~5%); well-published correlations for set-point tracking and disturbance rejection |
| **Disadvantages** | Requires FOPDT model identification (need estimate of dead time); separate correlations for set-point tracking vs. disturbance rejection may give different parameters; correlations are approximate |
| **Tuning aggressiveness** | **Conservative to moderate** — typically produces lower overshoot than ZN or Cohen–Coon |
| **Academic defensibility** | **High** — demonstrates understanding of performance index optimization; presented in Seborg et al.; distinguishes between servo and regulatory tuning |

### Direct Synthesis

**Mechanism:** Direct synthesis specifies the desired closed-loop transfer function and algebraically determines the required controller. For a desired first-order closed-loop G_CL(s) = 1/(λs + 1):

$$G_c(s) = \frac{G_{\text{CL,desired}}}{(1 - G_{\text{CL,desired}}) \cdot G(s)} = \frac{1/(λs + 1)}{[1 - 1/(λs + 1)] \cdot G(s)} = \frac{1}{λ s \cdot G(s)}$$

For G(s) = K/(τs + 1): G_c(s) = (τs + 1)/(Kλs), which is a PI controller with K_p = τ/(Kλ) and τ_I = τ — identical to IMC for a first-order plant.

| Aspect | Assessment |
|---|---|
| **Suitability for this plant** | **Excellent** — mathematically equivalent to IMC for this plant structure |
| **Advantages** | Transparent design philosophy; directly shows the connection between desired performance and controller parameters; elegant derivation |
| **Disadvantages** | Yields the same result as IMC; less widely applied to higher-order systems without modification |
| **Academic defensibility** | **Very high** — demonstrates model-based design thinking; strongly aligned with CHE F342 objectives |

### Recommended tuning methodology

**Primary method: IMC-based PI tuning** (equivalently, direct synthesis)

The recommendation is based on:

1. **Perfect plant-method match:** IMC is designed for first-order (and FOPDT) plants; our plant is first-order
2. **Single-parameter simplicity:** Only λ needs to be chosen; all other parameters follow from the model
3. **Physical interpretability:** λ is the desired closed-loop time constant — the designer directly specifies how fast the car should respond
4. **Guaranteed properties:** Zero overshoot (with pole-zero cancellation), zero steady-state error, and predictable robustness margins
5. **Academic alignment:** Taught in Seborg et al. (which is the CHE F342 text); demonstrates model-based design; directly uses the transfer function derived in Section 2
6. **Tuning range guidance:** Choose λ between 1 s (aggressive, for demonstrating fast response) and 10 s (conservative, for demonstrating robustness); simulate multiple values to show the speed–robustness tradeoff

**Secondary method: ITAE correlations** as a cross-check and comparison point. This provides an independent tuning approach with a rigorous optimality criterion, strengthening the project by showing that two different methods yield comparable (but non-identical) results and discussing why they differ.

**ZN is recommended NOT as a primary tuning method** but as a comparison case in simulation, to demonstrate *why* aggressive tuning is inappropriate for this plant and to show academic awareness of the method's limitations.

---

## 8. Practical implementation considerations

### Actuator saturation

The throttle position is physically bounded: u ∈ [0, 1] (fully closed to fully open). In force terms, the maximum engine force depends on speed and gear but is typically in the range of 3000–6000 N for a sedan engine. The IMC-tuned PI controller with K_p = 320 N/(m/s) would demand a force correction of 320 × 5 = 1600 N for a 5 m/s error — within the engine's capability. However, for large set-point changes (e.g., accelerating from 20 to 30 m/s), the initial force demand can exceed the engine's maximum output, causing the throttle to saturate at u = 1.

During saturation, the actual control effort is clamped at u_max while the controller "thinks" it is applying more. The proportional term adjusts immediately when the error decreases, but the integral term has accumulated an artificially large integral that must "unwind" before the control signal drops below the saturation limit. This causes **overshoot** — the vehicle accelerates past v_ref because the integrator keeps the throttle open too long.

### Integrator windup and anti-windup

**Integrator windup** is the most critical practical concern for PI/PID cruise control. Three standard anti-windup strategies are:

1. **Clamping (conditional integration):** Stop integrating when the actuator is saturated. Simple to implement; widely used in automotive ECUs. Limitation: the integrator does not unwind, so there can be a delay when exiting saturation.

2. **Back-calculation:** Feed the difference between the desired and actual (saturated) control signal back to the integrator through a gain 1/τ_t (tracking time constant). When the actuator is not saturated, the feedback is zero and the controller operates normally. During saturation, the feedback drives the integrator toward a value consistent with the saturated output. This is the method recommended by Åström and Murray for cruise control (Chapter 11 of *Feedback Systems*).

3. **Integrator reset:** Set the integrator state to the value that would produce the saturated output, so u_controller = u_saturated at all times. Aggressive but effective; can cause chattering near the saturation boundary.

For the simulation study, **back-calculation anti-windup** should be implemented and compared against no anti-windup to demonstrate the impact.

### Derivative kick

When the set-point changes stepwise (driver inputs a new target speed), the error e(t) = v_ref(t) − v(t) has a discontinuous jump, and its derivative is theoretically infinite. The derivative term produces a large spike ("kick") that slams the throttle to its limit instantaneously.

**Mitigation: Derivative on process variable (PV)**. Compute the derivative term as −K_pτ_D(dv/dt) instead of K_pτ_D(de/dt). Since the process variable v(t) is continuous (it cannot jump instantaneously due to vehicle inertia), the derivative is always finite. The derivative now responds only to measured velocity changes (including disturbances) but not to set-point changes — which is desirable because the set-point change itself does not need an impulsive response. This is standard practice in all modern PID implementations.

For PI control (recommended), derivative kick is not an issue because there is no derivative term.

### Measurement noise

Vehicle speed is typically measured by wheel-speed sensors (ABS encoders) that produce pulse trains proportional to wheel rotation rate. The noise characteristics include:

- **Quantization noise:** Finite encoder resolution produces ±0.05–0.1 m/s steps
- **Vibration-induced noise:** Road roughness, engine vibration, and wheel imbalance create oscillations at frequencies above 5–10 Hz
- **Tire slip:** Under acceleration/braking, wheel speed does not equal vehicle speed; this is a bias error rather than noise

For PI control, noise passes through the controller gain (proportional term) and is integrated (integral term). The proportional term transmits noise directly to the throttle; the integral term averages noise over time, smoothing it. Neither is catastrophically noise-sensitive.

For PID control, the derivative term amplifies noise, making filtering mandatory. A first-order derivative filter with cutoff frequency ω_f = N/τ_D (N ≈ 10) attenuates noise above ω_f while preserving derivative action at frequencies below ω_f.

### Filtering needs

Even with PI control, a **low-pass filter on the velocity measurement** is standard practice:

$$G_f(s) = \frac{1}{\tau_f s + 1}$$

with τ_f = 0.1–0.5 s (cutoff frequency 2–10 Hz). This filters road-induced vibrations and quantization artifacts before they reach the controller. The filter introduces a small additional lag, which must be accounted for in the controller tuning. If τ_f ≪ τ_CL (filter time constant much smaller than closed-loop time constant), the impact on stability and performance is negligible.

### Sampling and discretization

Modern cruise control is implemented digitally with a finite sampling rate. Typical automotive ECU sampling rates for cruise control are 10–50 Hz (T_s = 0.02–0.1 s). Since the closed-loop time constant is τ_CL ≈ 5 s, the sampling theorem is satisfied by a factor of at least 50:1 (Nyquist ratio), and discretization effects are negligible. The continuous-time PI controller can be discretized using the **Tustin (bilinear) transformation** or **backward Euler** method without performance degradation.

For the CHE F342 project, continuous-time analysis is sufficient. Discretization can be mentioned as an engineering consideration but does not need to be simulated.

---

## 9. Common pitfalls and academic weaknesses

### Typical controller design mistakes

1. **Using textbook PID tuning rules blindly on a dead-time-free plant.** Ziegler–Nichols is designed for plants with significant dead time. Applying ZN to G(s) = K/(τs + 1) without dead time produces meaninglessly aggressive gains. Students must justify why their chosen tuning method is appropriate for the specific plant structure.

2. **Adding derivative action without justification.** A first-order plant does not benefit meaningfully from derivative action. Students who use PID "because it's more advanced" without analyzing whether the derivative provides genuine performance improvement reveal a lack of engineering judgment. If PID is used, the improvement over PI must be quantified.

3. **Ignoring actuator saturation in controller design.** Designing a controller with K_p = 5000 achieves a 1-second closed-loop time constant on paper, but the required throttle force would exceed the engine's capability for any meaningful error. Physical constraints must inform the tuning choice.

4. **Not converting between model variants correctly.** The simplified model uses force input (N), while the full Åström and Murray model uses throttle position (dimensionless). Controller gains have different units and magnitudes for these two models. Mixing parameters between models produces nonsensical results.

5. **Setting τ_I too small (overly aggressive integral action).** Small τ_I produces fast integral correction but also introduces oscillatory behavior. A classic student error is choosing τ_I = 1 s for a plant with τ = 64 s, creating a highly underdamped closed-loop with 30–50% overshoot.

6. **Neglecting to test disturbance rejection.** Many student projects tune the controller only for set-point tracking and never simulate a grade disturbance. Since disturbance rejection is the primary operational function of cruise control (drivers rarely change the set speed, but constantly encounter hills), this is a critical omission.

### Weak justifications professors may criticize

- **"We used PID because it is the most general controller"** — this is not engineering reasoning. The choice of controller structure should be justified by analyzing the plant and the control requirements.
- **"We tuned K_p, K_i, K_d by trial and error in simulation"** — unacceptable for a PDC course; systematic tuning methods must be applied and justified using transfer function analysis.
- **"The controller is stable because the step response converges"** — stability should be analyzed using pole locations, Routh criterion, or Nyquist/Bode analysis, not inferred from a single simulation run.
- **"Overshoot doesn't matter for cruise control"** — this reveals a lack of application awareness; exceeding the speed limit, even momentarily, is a genuine operational concern.
- **"We chose λ = 1 s for the fastest possible response"** — without discussing the actuator effort, robustness, and noise implications of aggressive tuning, this is an incomplete justification.

### Poor assumptions to avoid

- Assuming the plant gain K is constant across all operating points (it varies inversely with v₀)
- Treating the controller output as unbounded (ignoring throttle position limits)
- Assuming perfect velocity measurement (real sensors have noise and quantization)
- Linearizing around one operating point and claiming the controller works for all speeds without testing
- Treating grade disturbances as negligible (they produce the largest real-world performance challenges)

---

## 10. Preparation for next section (simulation)

### Controller and tuning parameters to carry forward

The following specific controller configurations should be simulated:

| Controller | Method | K_p [N/(m/s)] | τ_I [s] | τ_D [s] | λ [s] | Notes |
|---|---|---|---|---|---|---|
| P | Direct design | 295 | — | — | — | τ_CL ≈ 5 s; baseline |
| PI (IMC, moderate) | IMC | 320 | 64.1 | — | 5 | τ_I = τ (pole-zero cancellation) |
| PI (IMC, aggressive) | IMC | 1603 | 64.1 | — | 1 | Demonstrates fast but demanding response |
| PI (IMC, conservative) | IMC | 160 | 64.1 | — | 10 | Demonstrates robustness at cost of speed |
| PI (ITAE) | ITAE correlation | TBD in sim | TBD | — | — | Cross-check against IMC tuning |
| PID (IMC) | IMC for SOPDT | TBD | TBD | TBD | 5 | Only if engine lag modeled as cascade |
| PI (ZN) | ZN reaction curve | 1442 | 3.3 | — | — | Expected overshoot ~25%; demonstration only |

For the full Åström and Murray engine-coupled model (K = 101, τ = 73.4 s), the corresponding IMC-PI parameters are:

| Controller | K_p [throttle/(m/s)] | τ_I [s] | λ [s] |
|---|---|---|---|
| PI (IMC, moderate) | 0.145 | 73.4 | 5 |
| PI (IMC, aggressive) | 0.727 | 73.4 | 1 |
| PI (IMC, conservative) | 0.073 | 73.4 | 10 |

### Simulation scenarios to test

Based on the design analysis, the following scenarios capture the critical performance dimensions:

1. **Set-point step change (servo test):** Step v_ref from 25 to 30 m/s (20% change). Measure rise time, overshoot, settling time, steady-state error. Compare all controllers.

2. **Grade disturbance rejection (regulatory test):** At constant v_ref = 25 m/s, apply a step grade of θ = 4° (0.07 rad) at t = 50 s. Measure peak deviation, recovery time, steady-state error. This tests the core operational scenario.

3. **Combined servo + regulatory:** Step change in v_ref followed by a grade disturbance. Tests real-world sequential events.

4. **Robustness to mass uncertainty:** Repeat key tests with m = 1200 kg (light car) and m = 2000 kg (loaded SUV). The IMC controller was tuned for m = 1600 kg; the simulation should verify acceptable performance at off-design conditions.

5. **Anti-windup demonstration:** Large set-point change (e.g., 15 to 30 m/s) that saturates the throttle. Compare PI with and without anti-windup.

6. **Tuning aggressiveness comparison:** Vary λ = {1, 2, 5, 10, 20} s with IMC-PI and overlay the step responses. This demonstrates the single-parameter tuning tradeoff predicted by the design.

7. **Linear vs. nonlinear model comparison:** Run the same controller on both the linearized plant G(s) = K/(τs+1) and the full nonlinear ODE. This validates (or exposes limitations of) the linearization at the operating point and at perturbations away from v₀.

### Expected outcomes to verify

- PI (IMC, λ = 5 s) should meet the overshoot (< 10%), steady-state error (0%), and disturbance rejection specifications. Rise time at λ = 5 is ~11 s, exceeding the CTMS target of < 5 s; the λ-sweep (Scenario E3) should identify the λ value that meets the rise time spec while retaining acceptable robustness
- P controller should show ~8% steady-state error at K_p = 295
- ZN-tuned PI should show ≈25% overshoot, confirming its unsuitability
- PID should show marginal improvement over PI on the first-order plant, but noise sensitivity if derivative is active
- Anti-windup should visibly reduce overshoot for large set-point changes
- Robustness tests should show that IMC-tuned PI performs acceptably across the 1200–2000 kg mass range
- Nonlinear simulation should match the linear simulation closely for small perturbations (±3 m/s) but diverge for large perturbations (±10 m/s), validating both the linearization accuracy and the controller's robustness
