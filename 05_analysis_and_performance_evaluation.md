# Analysis, performance evaluation, and engineering interpretation for car cruise control

**Simulation data without rigorous interpretation is a table of numbers — the engineering value lies entirely in converting quantitative results into justified conclusions, reasoned tradeoff assessments, and defensible controller recommendations.** This section establishes the analytical framework for evaluating the P, PI, and PID controllers designed in Section 3 against the cruise control performance specifications, using the simulation outputs defined in Section 4. Every interpretive guideline is grounded in the specific plant G(s) = 0.0401/(64.1s + 1), the specific disturbance transfer function G_d(s) = −628/(64.1s + 1), and the specific operating context of a 1600 kg sedan at 25 m/s on a highway. The goal is to produce analysis that is not merely correct but *academically distinguished* — analysis that demonstrates engineering judgment, not just mathematical competence.

---

## 1. Controller comparison framework

### Evaluation hierarchy

Not all performance metrics carry equal weight for cruise control. The evaluation criteria should be applied in a **strict priority hierarchy** reflecting the application's actual requirements:

| Priority | Criterion | Weight | Rationale |
|---|---|---|---|
| **1 (mandatory)** | Steady-state error | Pass/fail | Cruise control that doesn't hold set speed is functionally useless. This is a constraint, not an optimization target. |
| **2 (mandatory)** | Stability | Pass/fail | Closed-loop must be BIBO stable for all tested conditions. Another constraint. |
| **3 (primary)** | Disturbance rejection — peak deviation | High | Maintaining speed on hills is the primary operational function. Drivers rarely change set speed; they constantly encounter grades. |
| **4 (primary)** | Disturbance rejection — recovery time | High | Prolonged speed deviations are uncomfortable and potentially unsafe (e.g., slow truck on uphill merge). |
| **5 (secondary)** | Rise time / settling time | Medium | Must meet specification (< 5 s / < 15 s), but exact values beyond the threshold matter less than disturbance performance. |
| **6 (secondary)** | Overshoot | Medium | Speed limit violation concern. Must stay < 10%, but 0% is not categorically better than 5% if other metrics improve. |
| **7 (tertiary)** | Control effort / actuator usage | Lower | High control effort is acceptable if physically realizable. Only matters when it approaches actuator limits. |
| **8 (tertiary)** | Robustness to parameter uncertainty | Lower | Important for practical deployment but secondary to nominal performance in an academic context. |
| **9 (context)** | Implementation complexity | Lowest | Relevant only as a tiebreaker. A 2-parameter PI is preferable to a 4-parameter PID+filter only if performance is comparable. |

### Fair comparison principles

**Same scenario, same conditions.** Every controller must be evaluated on identical scenarios (same step magnitude, same disturbance, same simulation duration, same model) with the same metric computation code. An unfair comparison arises when, for example, PI is tested on a 5 m/s step but PID is tested on a 2 m/s step.

**Apples-to-apples tuning.** Controllers should be compared at equivalent design intent, not equivalent parameter values. Comparing P at K_p = 295 (designed for τ_CL ≈ 5 s) against PI at K_p = 10 (poorly tuned) proves nothing. Each controller should be tuned to its best achievable performance using the recommended methodology (IMC for PI, direct design for P). The comparison then reveals *structural* differences, not tuning quality differences.

**Multi-metric assessment.** No controller should be declared "best" on a single metric. A controller with 2 s rise time but 30% overshoot is not better than one with 4 s rise time and 3% overshoot — the latter better serves cruise control passengers. The evaluation must consider the full metric vector.

**Include both servo and regulatory performance.** A controller optimized exclusively for set-point tracking may perform poorly under disturbances, and vice versa. The comparison must include both Scenario S1 (step tracking) and Scenario D1 (grade rejection) at minimum.

### Tradeoff weighting for this application

When metrics conflict — as they inevitably will — the resolution follows the application context:

- **Rise time vs. overshoot:** For cruise control, *overshoot is more costly than slow rise time*. A 7-second rise time (slightly above spec) with 0% overshoot is preferable to a 3-second rise time with 12% overshoot (above spec). The reasoning: passengers tolerate gradual acceleration but notice and dislike speed spikes, and speed limit enforcement creates a hard constraint on overshoot.

- **Disturbance rejection vs. control effort:** Better disturbance rejection (smaller peak deviation, faster recovery) at the cost of higher control effort is acceptable *until the actuator saturates*. A controller that uses 80% of available throttle to reject a 4° grade is superior to one that uses 40% but allows twice the speed drop.

- **Speed vs. robustness:** The IMC λ parameter directly parameterizes this tradeoff. For an academic project, presenting the *entire tradeoff curve* (λ sweep, Scenario E3) is more valuable than claiming one λ is universally optimal. The recommended operating point λ = 5 s reflects a judgment that moderate speed with good robustness is appropriate for highway driving.

---

## 2. Performance interpretation guidelines

### Rise time

**Physical meaning:** The time for the vehicle to accelerate from 10% to 90% of the requested speed change. A 5 m/s step with t_r = 4 s means the car goes from 25.5 to 29.5 m/s in 4 seconds — an average acceleration of 1.0 m/s² (gentle but perceptible).

**High rise time (> 10 s):** The controller is too sluggish. The driver perceives a long delay between pressing "set speed" and reaching the target. While not dangerous, it degrades user experience and may cause the driver to override cruise control. Implies that the proportional gain K_p is too low or the closed-loop time constant τ_CL is too large relative to the specification.

**Low rise time (< 2 s):** The controller is demanding aggressive acceleration — approximately 2.5 m/s² for a 5 m/s step, equivalent to moderate braking force applied in reverse. This requires high engine force (potentially saturating the throttle), produces noticeable g-forces for passengers, and indicates that the proportional gain is very high. Academically, fast rise time appears good on paper, but excessive speed comes at the cost of actuator effort, noise sensitivity, and reduced robustness margin.

**Cruise control context:** Rise time matters primarily for the initial set-speed engagement and for speed adjustments. Highway drivers typically expect to reach the new set speed within 5–10 seconds. The 5-second specification from CTMS balances responsiveness with comfort.

**Tradeoff:** For the PI controller with IMC tuning (τ_I = τ, pole-zero cancellation), rise time is directly proportional to λ: t_r ≈ 2.3 × λ for a first-order response (the time to reach 90% is 2.303τ_CL). At λ = 5 s, t_r ≈ 11.5 s for the *closed-loop* — however, this is the *time constant*, not the rise time. For a first-order response, t_r (10%→90%) = τ_CL × ln(9) ≈ 2.2 × λ. At λ = 5, t_r ≈ 11 s, which is above the CTMS specification. This suggests that λ = 5 may actually be too conservative for the rise-time spec, and λ ≈ 2–3 s may be needed. This is a point where simulation should provide the definitive answer, because the pole-zero cancellation gives exact first-order response only if the model is perfect.

> **Analytical note on the rise time specification:** The CTMS specification of t_r < 5 s was formulated for a plant with τ = 20 s (the CTMS benchmark uses m = 1000 kg, b = 50 N·s/m). Our plant has τ = 64.1 s — a 3.2× slower plant. Achieving the same rise time requires 3.2× more aggressive control. This is achievable (the plant is stable for any gain), but the 5-second target may be more demanding than it appears for our heavier, more aerodynamically different vehicle. If simulations show that meeting t_r < 5 s requires actuator saturation or unacceptable control effort, it is academically defensible to relax this specification and justify the relaxation in terms of the different plant parameters.

### Settling time

**Physical meaning:** The time after which the velocity stays within ±2% of the final value. For a 5 m/s step, the ±2% band means the velocity must remain within ±0.1 m/s of the target — essentially, within the resolution of a typical speedometer.

**High settling time (> 30 s):** The system either has insufficient damping (oscillating within the band repeatedly) or the integral action is very slow. In cruise control, this means the speedometer visibly fluctuates for an extended period after a change.

**Low settling time (< 5 s):** The system reaches steady state quickly, but if settling time is significantly less than rise time, something is wrong with the computation (settling starts from the step onset, not from when the response enters the band for the first time).

**For the P controller:** Settling time ≈ 3τ_CL to 4τ_CL (within the 2% band of the *offset steady-state value*, not the reference). At K_p = 295, τ_CL ≈ 5 s, so settling time ≈ 15–20 s. Note that the P controller "settles" to the wrong value — it settles to 92.2% of the reference.

**For the PI controller (IMC, pole-zero cancellation):** The closed-loop is first-order with time constant λ. Settling to within 2% requires t_s ≈ 3.9λ. At λ = 5, t_s ≈ 20 s. At λ = 2, t_s ≈ 8 s. This meets the 15 s specification only for λ ≤ ~3.8 s.

**Tradeoff with overshoot:** For underdamped systems (PI without pole-zero cancellation, or ZN-tuned PI), settling time can be extended by oscillatory behavior. The response may enter the 2% band, exit, re-enter, and exit again. In this case, settling time can far exceed the rise time. The ZN-tuned PI, with its quarter-decay-ratio design target, may require 4–6 oscillation cycles to settle, producing settling times of 30+ seconds despite a fast rise time.

### Overshoot

**Physical meaning:** The maximum velocity excursion above the set speed, as a percentage of the step magnitude. 10% overshoot on a 5 m/s step means the vehicle briefly reaches 30.5 m/s on the way to 30 m/s — a 0.5 m/s exceedance.

**Why overshoot matters more than textbooks suggest:** In the academic PID design context, overshoot is often treated as one metric among many. For cruise control specifically, overshoot has *asymmetric consequences*:

- A vehicle set to cruise at 120 km/h that overshoots to 126 km/h may trigger a speed camera or violate a traffic law. The cost of overshoot is not just a comfort issue — it is a **hard constraint** in the speed-limit direction.
- Conversely, undershoot (slow approach from below) has no penalty beyond delayed arrival at set speed.
- This asymmetry means that **0% overshoot is genuinely preferable to 5% overshoot**, even if the 5% overshoot controller has slightly faster rise time. The IMC pole-zero cancellation design achieves exactly 0% overshoot by producing a first-order closed-loop — this is a significant advantage.

**For P controller:** Overshoot is structurally impossible (first-order closed-loop with real pole).

**For PI controller (IMC, τ_I = τ):** Overshoot is zero by design (pole-zero cancellation yields first-order closed-loop). However, if the model is inexact (τ_I ≠ τ_actual), the cancellation is imperfect and a small overshoot may appear. This is where the nonlinear simulation reveals the truth: the plant's effective time constant varies with speed, so the pole-zero cancellation is inherently approximate.

**For PI controller (ZN-tuned):** Overshoot ≈ 25% by design (quarter-decay-ratio criterion). This is unacceptable for cruise control. The simulation should confirm this prediction, providing a quantitative argument against ZN tuning for this application.

**For PID controller:** Overshoot depends on tuning. The derivative action can either increase or decrease overshoot depending on the relative magnitudes of τ_D and the other parameters. Properly tuned PID on a first-order plant should not produce significant overshoot.

### Steady-state error

**Physical meaning:** The permanent velocity difference between the set speed and the actual cruising speed after all transients have decayed. 8% error at 25 m/s set-point means the vehicle cruises at 23 m/s — a 2 m/s discrepancy visible on any speedometer.

**Why this is a binary pass/fail for cruise control:** 

The entire purpose of cruise control is to maintain a precise speed. A system with persistent error is not performing its primary function. There is no "acceptable" non-zero steady-state error for production cruise control. The 2% CTMS specification is already generous — production systems target < 0.5%.

**Analytical prediction:**
- P controller: e_ss = 1/(1 + K_pK) = 1/(1 + 295 × 0.0401) = 7.8% — fails specification
- PI controller: e_ss = 0% for any positive K_p — passes by the internal model principle
- PID controller: e_ss = 0% (contains integral action) — passes

This metric **immediately eliminates the P controller from contention** for the recommended design. The P controller is included only as a pedagogical baseline demonstrating why integral action is necessary.

**Subtle point for the P controller's disturbance rejection:** Under a grade disturbance, the P controller's steady-state error is *even worse* — the disturbance adds an additional offset ΔV_ss = −K_d·Δθ/(1 + K_pK) on top of any reference tracking error. With a 4° grade, this adds −3.4 m/s, and the combined offset from reference tracking and disturbance rejection makes the P controller wholly inadequate.

### Integral error criteria (IAE, ISE, ITAE)

**IAE (Integral Absolute Error):** ∫|e(t)| dt. Treats all errors equally regardless of timing. A controller with a brief large error and quick recovery can have the same IAE as one with a small but persistent error.

**ISE (Integral Squared Error):** ∫e²(t) dt. Penalizes large errors disproportionately (squared). A 2 m/s error for 1 second contributes as much as a 1 m/s error for 4 seconds. ISE-optimal controllers tend to produce fast initial responses with possible overshoot.

**ITAE (Integral Time-weighted Absolute Error):** ∫t·|e(t)| dt. Penalizes late errors more than early errors. This is the most physically meaningful criterion for cruise control because:

1. A brief error spike immediately after a set-point change is tolerable (driver expects a transition period)
2. A persistent error minutes later is unacceptable (driver expects the system to have settled)
3. ITAE captures exactly this asymmetry

**How to interpret comparative ITAE values:**

- If PI has ITAE = 40 and PID has ITAE = 38, the difference is negligible (5%) and does not justify the added complexity of PID
- If PI IMC (λ=5) has ITAE = 40 and PI ZN has ITAE = 120, the 3× difference is significant and reflects the ZN controller's oscillatory settling behavior
- The P controller's ITAE will be *very large* (technically infinite for an infinite simulation horizon) because the persistent offset contributes increasingly over time. For practical comparison, compute ITAE over a fixed horizon (e.g., 80 s) and note that the P controller's ITAE grows without bound while PI/PID's stabilizes.

**Servo vs. regulatory ITAE:** Compute these separately. A controller may have excellent servo ITAE but poor regulatory ITAE (or vice versa). For cruise control, the regulatory ITAE (disturbance rejection) is more operationally relevant.

### Control effort

**Physical meaning:** The force (or throttle position) the controller demands from the engine. High control effort means large throttle excursions; peak control effort determines whether the actuator saturates.

**What to look for in control effort signals:**

1. **Peak magnitude:** Does it exceed the engine's capability? For the simplified model, u_max ≈ 3000–6000 N for a sedan. For the Åström and Murray throttle model, u_max = 1.0 (fully open). If the peak control effort exceeds u_max, the linear simulation is giving physically impossible results, and the nonlinear simulation with saturation becomes the only valid prediction.

2. **Smoothness:** Rapid oscillations in control effort indicate chattering, which degrades drivetrain components and produces jerky ride quality. The total variation metric Σ|u(t_i+1) − u(t_i)| captures this. A smooth monotonic throttle ramp is preferable to an oscillating signal that achieves the same rise time.

3. **Steady-state value after disturbance:** After a grade disturbance, the new steady-state control effort should equal the force needed to maintain speed on the incline. For a 4° grade: u_new = u₀ + m·g·sin(4°) ≈ 469 + 1094 = 1563 N. This 3.3× increase is within engine capability but significant. The control effort plot visually shows the controller "working harder" on the hill.

**Tradeoff:** Higher K_p (faster response) produces larger peak control effort. The relationship is approximately linear: peak control effort ≈ K_p × initial error. With K_p = 320 and a 5 m/s step, peak effort ≈ 1600 N — well within engine limits. With K_p = 1603 (λ = 1) and a 5 m/s step, peak effort ≈ 8015 N — potentially saturating the engine for some vehicles.

### Disturbance rejection speed

**Physical meaning:** How quickly the velocity recovers to the set-point after a grade disturbance onset. Measured as either (a) the time to return within 2% of v_ref, or (b) the time from peak deviation to recovery.

**Strong vs. weak disturbance rejection:**

| Characteristic | Strong rejection | Weak rejection |
|---|---|---|
| Peak deviation | < 5% of v_ref (< 1.25 m/s at 25 m/s) | > 10% of v_ref |
| Recovery time | < 15 s | > 60 s |
| Post-disturbance oscillation | None or minimal | Multiple oscillation cycles |
| Steady-state offset | Zero (PI/PID) | Permanent (P) |

**Which controller features improve disturbance rejection:**

1. **Integral action** — eliminates steady-state disturbance offset. Without it, the P controller permanently concedes speed on hills. This is the single most important feature.
2. **Higher proportional gain** — reduces peak deviation. The deviation is approximately ΔV_peak ≈ −c·Δθ·τ_CL/τ, which decreases as K_p increases (because τ_CL decreases).
3. **Faster integral action (smaller τ_I)** — speeds up recovery but may introduce oscillation. This is the τ_I tradeoff: τ_I = τ (pole-zero cancellation) gives smooth but slower recovery; τ_I < τ gives faster recovery with oscillatory risk.
4. **Derivative action** — detects deceleration onset and responds immediately. Provides the fastest initial response to grade changes but amplifies noise. Benefit is modest for this first-order plant.

### Oscillation and damping

**Physical meaning:** Oscillatory velocity (the car accelerating and decelerating repeatedly around the set-point) is the most uncomfortable and undesirable behavior in cruise control. Passengers experience alternating acceleration and deceleration as a "surging" or "hunting" sensation.

**Damping ratio interpretation for the PI closed-loop:**

| ζ range | Behavior | Cruise control assessment |
|---|---|---|
| ζ > 1 | Overdamped, no overshoot | Ideal for cruise control comfort |
| 0.7 ≤ ζ ≤ 1.0 | Critically/slightly underdamped | Acceptable; minimal overshoot (< 5%) |
| 0.5 ≤ ζ < 0.7 | Underdamped, moderate oscillation | Borderline; drivers notice surging |
| ζ < 0.5 | Highly underdamped, significant oscillation | Unacceptable; feels like a broken system |
| ζ ≈ 0.22 | Quarter-decay ratio (ZN design target) | **Definitively unacceptable** for cruise control |

**For the IMC-tuned PI with pole-zero cancellation (τ_I = τ):** The closed-loop is first-order — there is no oscillation and no meaningful damping ratio (the concept applies only to second-order or higher systems). This is a **structural advantage** of the IMC design: it cannot oscillate because the characteristic equation has only one root.

**For the ZN-tuned PI (τ_I = 3.3 s):** The two closed-loop poles will be complex conjugates with ζ ≈ 0.22, producing pronounced oscillation. The simulation should show 4–6 visible oscillation cycles before settling, which creates an excellent visual contrast against the smooth IMC response.

---

## 3. Expected behavioral trends

### P controller — predicted behavior

#### Set-point tracking (Scenario S1: 25 → 30 m/s)

The P controller produces a **smooth first-order exponential approach** to a steady-state value *below* the reference. With K_p = 295 and K = 0.0401:

- Closed-loop time constant: τ_CL = 64.1/(1 + 11.8) = 5.0 s
- Rise time (10→90%): t_r = 2.2 × 5.0 = 11.0 s
- Final value: 25 + 5 × 0.922 = 29.61 m/s (not 30 m/s)
- Steady-state error: 0.39 m/s (7.8%)
- Overshoot: 0% (guaranteed for first-order)

**Why this happens:** The P controller output is K_p × e(t). As the error shrinks, the control force diminishes. Eventually, the system reaches a point where the control force exactly balances the increased drag at the new (sub-reference) speed. The error can never reach zero because zero error would produce zero corrective force, which cannot maintain the higher speed against increased drag.

#### Disturbance rejection (Scenario D1: 4° grade)

The P controller will show a **permanent velocity drop** of approximately 3.4 m/s:

- The grade introduces F_grade = mg sin(4°) ≈ 1094 N
- The P controller can only partially compensate: ΔV_ss = −1094 × 0.0401/12.8 ≈ −3.4 m/s
- The vehicle cruises at ~21.6 m/s instead of 25 m/s on the slope

**Academically, this is the most powerful single result for motivating integral action.** If the simulation confirms a 3.4 m/s drop matching the analytical prediction, it simultaneously validates the model and demonstrates P control's fundamental limitation.

#### Robustness (Scenario R1: mass variation)

The P controller's performance varies with mass but in a predictable, bounded way:
- Lighter vehicle (m = 1200 kg): Faster response (shorter τ_CL), slightly *smaller* steady-state error (because the plant gain K increases slightly due to lower b_eff at the same speed)
- Heavier vehicle (m = 2000 kg): Slower response, same error magnitude (error depends on K_pK, and both K and τ change proportionally with mass, but K_pK is approximately constant)

The P controller's robustness is adequate in terms of stability — it cannot destabilize for any positive mass — but the performance variation is modest because the error is dominated by the structural limitation rather than the mass-dependent dynamics.

### PI controller — predicted behavior

#### Set-point tracking (Scenario S1)

The PI controller (IMC, λ = 5, τ_I = τ) produces a **clean first-order response to the exact reference value**:

- Closed-loop time constant: λ = 5 s (by IMC design)
- Rise time: t_r ≈ 2.2 × 5 = 11 s (approximately, assuming perfect pole-zero cancellation)
- Final value: exactly 30 m/s (zero steady-state error)
- Overshoot: 0% (if τ_I = τ exactly); possibly 1–3% in nonlinear simulation (imperfect cancellation)

For the aggressive tuning (λ = 1): t_r ≈ 2.2 s, zero error, 0% overshoot in linear model but possible actuator saturation in nonlinear model.

For the ZN tuning (K_p = 1442, τ_I = 3.3 s): Fast rise time but ~25% overshoot, oscillatory settling, zero final error.

**Why IMC produces zero overshoot while ZN produces large overshoot:** The IMC design cancels the plant pole with the controller zero, reducing the closed-loop to first order. The ZN design does not cancel the pole; its very small τ_I places the controller zero far from the plant pole, creating two complex conjugate closed-loop poles with low damping. The fundamental difference is structural (first-order vs. underdamped second-order closed-loop), not merely a matter of gain magnitude.

#### Disturbance rejection (Scenario D1)

The PI controller shows a **transient velocity drop followed by complete recovery**:

- Peak deviation: approximately −3.4 m/s for λ = 5 (the peak occurs before the integral term can accumulate sufficient correction)
- Recovery time: approximately 3–5λ = 15–25 s
- Steady-state error: exactly 0 m/s (integral action)

The peak deviation is large (13.6% at 25 m/s) because the IMC design with τ_I = τ produces slow disturbance rejection — the pole-zero cancellation that gives clean servo response also cancels the "wrong" pole for disturbance rejection. This is a known limitation of IMC tuning with pole-zero cancellation on processes with large time constants. A faster τ_I (e.g., τ_I = τ/3) would reduce peak deviation at the cost of introducing oscillation into the servo response.

**This is a key tradeoff to discuss:** IMC with τ_I = τ optimizes servo response at the expense of regulatory response. For cruise control, where regulatory performance (disturbance rejection) is arguably more important than servo performance, this tradeoff may not be optimal. The analysis should explicitly acknowledge this and discuss whether a slightly detuned τ_I (e.g., 0.5τ to 0.8τ) improves regulatory performance enough to justify the slight servo degradation.

#### Robustness (Scenario R1)

The PI controller's integral action guarantees zero steady-state error at *all* mass values — this is invariant to model uncertainty. The transient response changes:

- Lighter vehicle: Controller is effectively more aggressive (higher K_pK product in the effective closed-loop), faster response, potentially slight overshoot if the effective τ_CL becomes very short
- Heavier vehicle: Controller is effectively more conservative, slower response, overdamped

The critical test is whether the controller remains *within specification* at extreme masses. Expected outcome: the IMC design at λ = 5 s provides sufficient margin that m = 1200–2000 kg stays within specs, though likely with degraded rise time at 2000 kg.

### PID controller — predicted behavior

#### Set-point tracking (Scenario S1)

The PID controller should produce a response similar to PI with marginally faster rise time due to derivative action. For the first-order plant:

- The derivative term contributes K_pτ_D × (impulse at step onset) followed by K_pτ_D × (−dv/dt) as the velocity ramps up
- With derivative on PV (to avoid derivative kick), the contribution is −K_pτ_D(dv/dt), which provides a braking/moderating effect during the ramp, slightly *slowing* the initial response but reducing overshoot
- Net effect on a first-order plant: small, nuanced, highly dependent on τ_D relative to τ and λ

**Expected outcome:** PID performance is within 5–10% of PI on all servo metrics for this plant. If the simulation shows PID substantially outperforming PI, the derivative term τ_D is likely compensating for a model mismatch between the linear design assumptions and the nonlinear plant — which would be an interesting finding to analyze but not a reason to prefer PID in general.

#### Disturbance rejection

This is where PID has its best theoretical argument. The derivative term detects the velocity deceleration at the onset of a grade disturbance and immediately applies corrective force, *before* the proportional and integral terms ramp up:

- At the instant the grade applies, dv/dt becomes negative
- The derivative contribution: −K_pτ_D(dv/dt) > 0, adding throttle
- This provides a "head start" on disturbance rejection

**Expected improvement:** 10–20% reduction in peak deviation compared to PI with same K_p and τ_I. Recovery time similar. The derivative advantage is real but modest because the first-order plant has only one dynamic mode — there is no additional phase lag for the derivative to compensate.

**Expected cost:** Noise amplification in the control effort signal. The u(t) plot for PID should show higher-frequency content (from derivative of velocity noise) compared to the smooth PI control effort. If measurement noise is added to the simulation, the PID control effort becomes visibly noisier.

---

## 4. Disturbance rejection analysis framework

### Evaluation methodology

Disturbance rejection should be evaluated with the same rigor as servo performance, using three complementary perspectives:

#### Perspective 1: Time-domain metrics

Compute from Scenario D1 (4° grade step):

| Metric | Definition | Target |
|---|---|---|
| Peak deviation | max|v(t) − v_ref| for t > t_disturbance | < 1.25 m/s (5% of 25 m/s) |
| Time to peak | Time from disturbance onset to maximum deviation | Shorter is not necessarily better — fast disturbances that are quickly corrected may peak early |
| Recovery time (2%) | Time from disturbance onset until |v(t) − v_ref| < 0.5 m/s permanently | < 30 s |
| Steady-state offset | v(t→∞) − v_ref | 0 for PI/PID; nonzero for P |
| Regulatory ITAE | ∫t|v(t) − v_ref| dt from t_disturbance onward | Lower is better |

#### Perspective 2: Frequency-domain analysis

The closed-loop disturbance transfer function is:

$$T_d(s) = \frac{G_d(s)}{1 + G_c(s) G(s)}$$

The magnitude |T_d(jω)| at different frequencies shows how different disturbance frequencies are attenuated. Key features:

- At ω = 0 (DC): |T_d(0)| = 0 for PI/PID (infinite loop gain from integrator), nonzero for P. This is the transfer function expression of zero-offset disturbance rejection.
- At ω → ∞: |T_d(jω)| → |G_d(jω)| (loop gain goes to zero; no disturbance rejection at high frequencies). This means rapid disturbances (pothole bumps, sudden gusts) are not attenuated — they pass through to the velocity. This is acceptable because such disturbances are typically small and transient.
- At intermediate frequencies: The loop gain determines the attenuation. Higher K_p provides more attenuation in the crossover region.

A Bode plot of |T_d(jω)| for each controller provides a compact visualization of disturbance rejection across all frequencies. This is more informative than a single step-response metric and demonstrates frequency-domain analysis competence.

#### Perspective 3: Physical interpretation

The grade disturbance applies a force F_grade = mg sin θ ≈ 1094 N for θ = 4°. The controller must increase throttle by this amount *on top of* the cruising force to maintain speed. The analysis should track:

1. **How quickly does the controller increase throttle?** (Visible in the u(t) subplot)
2. **What fraction of the available engine force does the disturbance require?** (1094/u_max — if this is close to 1, the controller has no margin for additional disturbances)
3. **Does the controller anticipate or react?** (The derivative term in PID provides anticipation; PI is purely reactive)

### Strong vs. weak disturbance rejection signatures

**Strong rejection:** Small, brief, monotonic deviation followed by smooth return to v_ref. The control effort ramps up promptly and settles at the new steady-state level (higher throttle for uphill) without oscillation. The velocity plot shows a shallow dip and recover.

**Weak rejection:** Large deviation, slow recovery, possible oscillatory approach to v_ref (for aggressively tuned controllers), or permanent offset (for P). The control effort either responds sluggishly (conservative tuning) or oscillates (aggressive integral action). The velocity plot shows either a deep prolonged dip or persistent oscillation.

**Pathological rejection:** The controller *increases* the deviation before correcting (sign error), or the control effort saturates and the vehicle decelerates uncontrollably (actuator limit interaction). These indicate bugs or fundamentally inappropriate controller design.

---

## 5. Robustness and sensitivity analysis framework

### Parameter uncertainty interpretation

The cruise control plant has three primary uncertain parameters:

| Parameter | Nominal | Range | Effect on plant |
|---|---|---|---|
| Vehicle mass *m* | 1600 kg | 1200–2000 kg (±25%) | Directly scales τ; does not change K for the simplified model |
| Drag coefficient *C_d* | 0.32 | 0.24–0.40 (±25%) | Changes both K and τ inversely |
| Operating speed *v₀* | 25 m/s | 15–35 m/s (±40%) | Strong effect: K ∝ 1/v₀, τ ∝ 1/v₀ |

The operating speed v₀ produces the largest variation in plant parameters because the linearized gain and time constant both depend on v₀. A controller designed at 25 m/s operates on a plant with K = 0.040 and τ = 64 s; at 15 m/s, the plant has K ≈ 0.067 and τ ≈ 107 s; at 35 m/s, K ≈ 0.029 and τ ≈ 46 s. The gain varies by a factor of 2.3× across the operating range. This is a severe model mismatch from the controller's perspective.

### Robustness assessment criteria

**Criterion 1: Stability robustness.** Does the closed-loop remain stable across the entire parameter range? For PI control on a first-order plant: yes, unconditionally. The closed-loop characteristic equation has all positive coefficients for any K_p > 0 and τ_I > 0, regardless of K and τ. This is provable via Routh stability and should be stated explicitly.

**Criterion 2: Performance robustness.** Do the specifications remain satisfied across the parameter range? This is the more demanding test. Use the following assessment framework:

| Specification | Sensitive to | Expected robustness |
|---|---|---|
| e_ss = 0 | Nothing (integral action) | **Perfectly robust** — this is the structural advantage of integral control |
| Rise time < 5 s | mass, v₀ (through τ) | Moderate — rise time ∝ λ for IMC, but true only if τ_I = τ (model match). At off-design points, imperfect cancellation changes the effective closed-loop dynamics |
| Overshoot < 10% | τ_I mismatch with actual τ | Moderate — at m = 1200 kg (τ ≈ 48 s), τ_I = 64.1 is too large, causing imperfect cancellation with a residual slow mode. At m = 2000 kg (τ ≈ 80 s), τ_I = 64.1 is too small, potentially causing slight oscillation |
| Disturbance < 5% | K_p relative to actual disturbance gain | Low — peak deviation is largely determined by the disturbance gain K_d and the closed-loop time constant. K_d = g/a, which varies with v₀ |

**Criterion 3: Graceful degradation.** When the controller operates outside its design range, does performance degrade gradually or catastrophically? For PI on a first-order plant, degradation is always gradual — there is no cliff edge. At extreme parameter mismatches, the response becomes slower or slightly oscillatory but never unstable. This is a significant robustness advantage of the simple plant structure.

### Acceptable vs. unacceptable robustness weaknesses

**Acceptable:**
- Rise time increases from 5 s to 8 s at m = 2000 kg (minor spec violation, no safety impact)
- Slight overshoot (3–5%) appears at m = 1200 kg (due to imperfect pole-zero cancellation)
- Peak disturbance deviation increases from 3.4 to 4.5 m/s at v₀ = 35 m/s

**Unacceptable:**
- Steady-state error appears at any mass/speed (would indicate an implementation bug, since integral action should guarantee zero error)
- Sustained oscillation at any operating condition (indicates the controller has driven the closed-loop into a poorly damped regime)
- Actuator saturation at nominal operating conditions (indicates the proportional gain is set too high for the available engine force)

### How to present robustness results

The most effective presentation format is a **parameter sweep overlay** (Scenarios R1, R2, R3 from Section 4):

1. **One figure per uncertain parameter** with multiple curves (one per parameter value) on the same axes
2. **A specification compliance table** showing pass/fail for each specification at each parameter value
3. **A summary statement** such as: "The PI IMC (λ = 5) controller meets all specifications for m ∈ [1200, 2000] kg except rise time at m = 2000 kg, where t_r = 7.3 s exceeds the 5 s target. The specification violation is minor (46% exceedance) and is an acceptable tradeoff for robustness across the full mass range."

---

## 6. Decision methodology for best controller selection

### Multi-criteria decision framework

The controller selection should follow a **sequential elimination** approach:

```
Step 1: Eliminate controllers that fail mandatory criteria (e_ss, stability)
        → P controller eliminated (fails e_ss)

Step 2: Among remaining, compare on primary criteria (disturbance rejection)
        → PI and PID both achieve zero steady-state error
        → Compare peak deviation and recovery time

Step 3: For essentially equivalent primary performance, compare on secondary
        criteria (rise time, overshoot, settling time)
        → PI IMC and PID IMC likely comparable

Step 4: For essentially equivalent primary and secondary performance, use
        tertiary criteria (complexity, noise sensitivity, robustness)
        → PI wins on simplicity and noise immunity

Step 5: State the recommendation and its boundary conditions
```

### Handling the "no controller dominates" situation

In practice, no controller will score best on every metric. The typical pattern for this plant will be:

- **P:** Best simplicity, best noise immunity, worst steady-state accuracy → eliminated by mandatory criterion
- **PI IMC (λ=5):** Good overall, zero error, smooth response, slower than aggressive options → likely winner
- **PI ZN:** Fastest rise time, worst overshoot, worst oscillation → eliminated by overshoot criterion
- **PID:** Marginally better disturbance rejection, noise sensitivity, more complex → marginal improvement at significant cost

The decision should be framed as: "PI (IMC, λ = 5) is recommended as the primary controller because it is the simplest controller that satisfies all mandatory and primary performance criteria. PID provides marginal improvement in disturbance rejection (X% reduction in peak deviation) at the cost of noise sensitivity and added tuning complexity — an unfavorable tradeoff for this first-order plant."

### Academic justification structure

A strong academic justification follows this template:

1. **State the recommendation explicitly:** "We recommend PI control with IMC tuning at λ = 5 s."
2. **Cite the mandatory criteria:** "P control is eliminated because it cannot achieve zero steady-state error."
3. **Present the quantitative comparison:** "PI and PID achieve identical steady-state performance. PI achieves rise time = X s, overshoot = Y%, peak disturbance deviation = Z m/s. PID achieves rise time = X' s, overshoot = Y'%, peak disturbance deviation = Z' m/s."
4. **Articulate the tradeoff judgment:** "The X% improvement in disturbance rejection from PID does not justify the added derivative term, which amplifies measurement noise and introduces an additional tuning parameter, especially given that the first-order plant provides no structural benefit from derivative action."
5. **Acknowledge limitations:** "This recommendation assumes the simplified first-order plant model. If engine dynamics introduce significant additional lag (τ_e > 1 s), the derivative term's value increases and PID should be reconsidered."
6. **Comparative tuning discussion:** "Among tuning methods, IMC produces superior results for this plant because it directly targets the achievable closed-loop time constant. ZN tuning produces unacceptable overshoot (~25%) because it targets quarter-decay-ratio, which is inappropriately aggressive for cruise control comfort."

### What NOT to do

- **Do not say "PI is better because it is simpler."** Simplicity alone is not an engineering argument. PI is better because it *meets all specifications* while being simpler. If PID met specifications that PI could not, PID's complexity would be justified.
- **Do not present data without making a decision.** "Both controllers performed well" is not a conclusion. The project demands a recommendation — making one demonstrates engineering judgment.
- **Do not ignore the losing controllers.** Explain *why* each alternative was rejected, not just why the winner was chosen. The elimination reasoning is as important as the selection reasoning.

---

## 7. Common weak analysis mistakes

### Superficial interpretations

1. **"The PI controller has zero steady-state error, therefore it is better."** This is stating a mathematical fact, not performing analysis. The analysis should explain *why* integral action produces zero error (internal model principle), *how* this manifests physically (integral accumulates throttle until equilibrium is restored), and *what cost* integral action introduces (possible overshoot, windup risk).

2. **"The rise time of Controller A is 4.2 s, which is less than 5 s, so the specification is met."** This is verification, not analysis. Analysis explains *why* the rise time has this value (what controller gain produces what closed-loop time constant), whether the margin is comfortable (4.2 vs. 5 — only 16% margin; a 20% mass increase might push it over), and how it compares to other controllers.

3. **"PID is the best controller because it has three tuning parameters."** More parameters does not mean better control. The best controller is the simplest one that meets all requirements. This is the minimum-complexity principle of engineering design.

4. **"The simulation results match our expectations, confirming the model is correct."** Agreement between simulation and analytics does *not* validate the physical model — it validates the simulation implementation. Model validation requires comparison with *experimental data* (physical car tests), which is outside the scope of this project. The correct framing is: "The simulation results are consistent with the analytical predictions, confirming the correctness of our numerical implementation."

### Poor comparison logic

5. **Comparing different metrics across different controllers.** Example: "P has good rise time (5 s) and PI has zero error" — this doesn't help. Both metrics should be computed for both controllers.

6. **Ignoring the disturbance rejection test.** Many student projects run only set-point tracking tests and conclude PI is "obviously better than P because it has zero error." The disturbance rejection test provides *additional* and *stronger* evidence — it shows that PI recovers to the exact set speed on a hill while P permanently drops by 3.4 m/s. Omitting this test weakens the argument unnecessarily.

7. **Treating overshoot as universally bad without context.** For cruise control, small overshoot (< 5%) in the *downward* direction (undershoot during disturbance recovery) is acceptable and may indicate faster disturbance rejection. Overshoot in the *upward* direction during set-point tracking is more concerning because of speed-limit implications. The analysis should distinguish between servo overshoot and regulatory transients.

### Weak justifications professors criticize

8. **"We chose PI because the textbook recommends it."** Appeal to authority. The textbook recommends PI for a specific plant structure for specific reasons — those reasons must be stated and applied to *this* plant.

9. **"The controller parameters were tuned using IMC method."** This describes *what* was done but not *why*. The justification must include: why IMC is suitable for this plant (first-order, known model), what λ was chosen and why (balances speed and robustness), and what properties the IMC-tuned controller guarantees (zero overshoot from pole-zero cancellation, zero error from integral action).

10. **"The simulation shows the controller works."** This is a demonstration, not an analysis. Working *how well*? Against what specification? Under what conditions? With what margin? What would make it fail?

11. **Not explaining disagreements between linear and nonlinear models.** If the nonlinear simulation shows 3% overshoot while the linear model shows 0%, this *must* be explained (the pole-zero cancellation is imperfect because the plant's effective τ varies with velocity). Unexplained discrepancies signal that the student does not understand the physics.

12. **Failing to connect simulation results to physical driving experience.** A 3.4 m/s speed drop on a 4° grade is a *concrete, experienceable event* — the car slows from 90 km/h to 78 km/h on a highway hill. Professors in CHE F342 value students who translate mathematical results into physical intuition.

---

## 8. Preparation for final section

### Key conclusions to carry into report discussion and viva

The analysis phase should crystallize the following findings, listed in order of academic impact:

**Finding 1: Integral action is essential for cruise control.** The P controller's 7.8% steady-state error and 3.4 m/s disturbance offset are quantitatively confirmed by simulation, match the analytical predictions from Section 3, and are functionally unacceptable. This is the most fundamental control-engineering insight of the project and should be the headline conclusion.

**Finding 2: IMC tuning produces superior results for this first-order plant.** The λ-parameterized family of responses demonstrates a clean, monotonic tradeoff between speed and robustness. ZN tuning is quantitatively shown to be inappropriate (25% overshoot confirmed), providing a strong negative comparison that demonstrates critical evaluation of established methods.

**Finding 3: The linearization is valid within a quantifiable range.** The linear vs. nonlinear comparison (Scenario E1) provides a numerical bound on linearization accuracy — likely ±5 m/s from the operating point. This is the most academically distinctive finding because it *directly demonstrates a core PDC concept* (linearization validity) with quantitative evidence specific to this system.

**Finding 4: The PI controller with IMC tuning (λ = 5 s) meets all performance specifications** under nominal conditions and degrades gracefully under parameter uncertainty. The robustness analysis showing stability and acceptable performance over m = 1200–2000 kg demonstrates that the design is not brittle.

**Finding 5: Anti-windup is practically necessary.** The large-step simulation (Scenario E2) shows quantifiable overshoot degradation without anti-windup, validating the practical concern raised in Section 3 §8.

### Which findings matter most academically

For a CHE F342 course, the relative academic value of findings is:

| Rank | Finding | Why it matters |
|---|---|---|
| 1 | Linearization validity bound (E1) | Directly demonstrates mastery of linearization — the core mathematical technique of the course |
| 2 | P vs. PI steady-state error contrast | Demonstrates understanding of integral action and the internal model principle |
| 3 | IMC vs. ZN performance contrast | Demonstrates critical evaluation of tuning methods — goes beyond applying formulas |
| 4 | λ-sweep tradeoff (E3) | Demonstrates understanding of the single-parameter design philosophy and the speed–robustness tradeoff |
| 5 | Robustness across mass range | Demonstrates practical engineering awareness beyond nominal-point design |
| 6 | Anti-windup comparison | Demonstrates awareness of practical implementation concerns |

### Data to include in final report

1. **Metrics comparison table** — the master table with all controllers × all metrics for S1 and D1. This is the factual backbone.
2. **Six priority plots** from Section 4 §9 — each annotated with specification thresholds and key numerical values.
3. **Linearization accuracy data** — the step-size vs. divergence curve from Scenario E1.
4. **Robustness specification compliance table** — pass/fail matrix for each controller × each specification × each parameter value.
5. **Brief qualitative summary** connecting simulation findings to physical driving experience.
