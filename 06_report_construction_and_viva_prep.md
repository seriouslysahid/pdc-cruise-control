# Report construction, presentation strategy, and viva preparation for car cruise control

**The technical work across Sections 1–5 constitutes a complete engineering study: a physically grounded nonlinear model, a rigorous linearization, a justified PI controller with IMC tuning, a dual-model simulation framework, and an analytical evaluation methodology.** This final section addresses the packaging challenge — converting that technical depth into a report that reads as polished and professional, and preparing for a viva defense where every modeling choice, assumption, and conclusion can be justified under questioning. The guidance below is specific to this cruise control project and calibrated for CHE F342 undergraduate evaluation standards.

---

## 1. Professional report structure

### Recommended structure and page allocation

| Section | Pages | Purpose |
|---|---|---|
| Title page | 1 | Project title, course, team, date |
| Abstract | 0.5 | Self-contained summary of the entire project |
| Table of contents | 0.5–1 | Auto-generated |
| 1. Introduction | 1–1.5 | Problem context, objectives, scope |
| 2. Literature review | 1.5–2 | Academic grounding and prior work |
| 3. Mathematical modeling | 3–4 | Physical model, assumptions, linearization, transfer functions |
| 4. Controller design | 2–3 | P/PI/PID design, tuning methodology, expected behavior |
| 5. Simulation methodology | 1.5–2 | Tools, scenarios, metrics, code architecture |
| 6. Results | 3–4 | Figures, tables, metrics — minimal interpretation |
| 7. Discussion | 2–3 | Interpretation, comparison, limitations, recommendations |
| 8. Conclusion | 0.5–1 | Key findings, recommendations, future work |
| References | 0.5–1 | Properly formatted citations |
| Appendix | 2–4 | Code listings, additional plots, derivation details |
| **Total** | **~20–25** | |

---

### Section-by-section guidance

#### Abstract (0.5 page)

**Purpose:** Allow the reader to understand the entire project without reading further. The abstract is the most-read and least-carefully-written section in student reports.

**Content — in exactly this order:**
1. Problem statement (1–2 sentences): "This project designs and evaluates a feedback controller for automotive cruise control using the methodology of Process Dynamics and Control."
2. Modeling approach (1–2 sentences): "A nonlinear force-balance model with quadratic aerodynamic drag is linearized around a highway operating point (v₀ = 25 m/s) to derive a first-order transfer function G(s) = K/(τs + 1)."
3. Controller design (1–2 sentences): "P, PI, and PID controllers are designed and compared. PI with Internal Model Control tuning is recommended based on zero steady-state error, adequate transient performance, and robustness."
4. Key results (2–3 sentences): "Simulation on both the linearized and full nonlinear plant confirms that PI (IMC, λ = 5 s) meets all performance specifications under nominal conditions. The linearization is validated within ±5 m/s of the operating point. The controller maintains acceptable performance across vehicle masses of 1200–2000 kg."
5. Conclusion (1 sentence): "PI control with IMC tuning is the recommended design for this cruise control system, offering the optimal balance of performance, simplicity, and robustness."

**Common mistakes:**
- Writing the abstract last and rushing it (write it last, but *invest* time)
- Including methodology details that belong in the body ("We used scipy.integrate.solve_ivp with RK45...")
- Omitting quantitative results (numbers make the abstract credible)
- Exceeding half a page (abstracts should be 150–250 words)

#### 1. Introduction (1–1.5 pages)

**Purpose:** Motivate the project, state objectives, and define scope.

**Content:**
- What is cruise control and why does it matter? (2–3 sentences; do not write an essay on automotive history)
- Why is it relevant to CHE F342? (Feedback control, linearization, transfer functions — the core syllabus tools applied to a physical system)
- Control objectives — qualitative and quantitative (cite the CTMS benchmark specifications)
- Project scope: what is included (modeling, linearization, controller design, simulation) and what is excluded (hardware implementation, adaptive control, MPC)
- Brief outline of the report structure

**Level of detail:** High-level only. No equations, no derivations, no literature deep-dives. The introduction sets context; the technical content lives in subsequent sections.

**Common mistakes:**
- Spending a full page on "the history of cruise control from 1958 to present" — irrelevant padding
- Listing all equations that will be derived later — premature and redundant
- Failing to state quantitative objectives (the reader should know the success criteria *before* reading the results)

#### 2. Literature review (1.5–2 pages)

**Purpose:** Demonstrate awareness of the academic context and justify modeling/design choices by referencing established work.

**Content — draw from Section 1:**
- Canonical textbook treatments (Åström & Murray, Seborg et al., Dorf & Bishop, CTMS tutorials)
- Three levels of model fidelity and the rationale for choosing the nonlinear force-balance model
- Standard assumptions in academic cruise control models
- Prior controller design approaches (PI as the standard recommendation)

**Level of detail:** Summarize and cite; do not reproduce textbook derivations. The literature review should read as a survey that culminates in "Therefore, we adopt the nonlinear force-balance model with Åström and Murray parameters and a PI controller with IMC tuning."

**Common mistakes:**
- Listing references without connecting them to the project ("Åström and Murray wrote a textbook" — so what? Explain what they contributed to cruise control modeling)
- Reviewing literature that was not actually used (e.g., reviewing MPC papers when the project uses PID)
- Omitting the justification bridge: the review should end with a clear statement of *which* approach was selected and *why*, based on the reviewed sources

#### 3. Mathematical modeling (3–4 pages)

**Purpose:** Derive the plant model from first principles with full mathematical rigor. This is the technical core of the report.

**Content — draw from Section 2:**

The derivation should follow this arc:
1. **Force balance** — Newton's second law with all four forces identified and dimensionally verified
2. **Steady-state operating point** — equilibrium calculation with numerical values (u₀ = 468.8 N at v₀ = 25 m/s)
3. **Linearization** — Taylor expansion of v², deviation variables, subtraction of steady-state equation
4. **Transfer functions** — Laplace transform, standard first-order form, process and disturbance TFs with numerical values (τ = 64.1 s, K = 0.0401)
5. **Physical interpretation** — what τ and K mean physically; why heavy cars feel sluggish
6. **Dimensional checks** — verify every term has correct units

**Level of detail:** This section should be the most detailed in the report. Show *every* algebraic step of the linearization. The Taylor expansion of v² and the steady-state subtraction are the two steps most likely to be probed in viva — they must be explicit and clearly labeled.

**Common mistakes:**
- Jumping from the nonlinear ODE directly to the transfer function without showing linearization steps
- Presenting the transfer function without deriving it (professors can detect copy-paste from textbooks)
- Omitting dimensional analysis (the single strongest credibility signal in an engineering derivation)
- Using notation inconsistently (switching between δv, Δv, v', and v̂ for the deviation variable)

#### 4. Controller design (2–3 pages)

**Purpose:** Design the controllers with theoretical justification and derive numerical tuning parameters.

**Content — draw from Section 3:**
1. **P controller** — closed-loop TF, τ_CL and K_CL formulas, K_p = 295 derivation, steady-state error calculation, brief assessment
2. **PI controller** — rationale for integral action, closed-loop TF, second-order analysis (ω_n, ζ), IMC tuning derivation (K_p = τ/(Kλ), τ_I = τ), numerical values for λ = 5 s
3. **PID controller** — role of derivative action, noise sensitivity concern, why it adds limited value for this first-order plant
4. **Controller comparison table** — the summary table from Section 3 §6
5. **Tuning methodology** — IMC rationale, brief comparison with ZN (and why ZN is inappropriate), ITAE as secondary method

**Level of detail:** Show the closed-loop transfer function derivations for P and PI. For PID, a qualitative discussion is sufficient. The IMC tuning formula derivation should be presented in 3–4 lines of algebra.

**Common mistakes:**
- Presenting all three controllers as equally viable and refusing to recommend one (the project demands a recommendation)
- Deriving PID formulas at length without concluding that PID is unnecessary for this plant (wasted effort if the conclusion is that PI suffices)
- Presenting tuning parameters without showing where they came from

#### 5. Simulation methodology (1.5–2 pages)

**Purpose:** Describe the simulation framework clearly enough that the reader could reproduce the results.

**Content — draw from Section 4:**
1. **Dual-model approach** — linear TF model for analytic verification, nonlinear ODE for physical fidelity
2. **Python toolchain** — python-control, scipy, numpy, matplotlib (cite library versions)
3. **Simulation scenarios** — table listing all scenarios with parameters and purpose
4. **Performance metrics** — definitions of rise time, settling time, overshoot, steady-state error, IAE/ITAE
5. **Code organization** — brief description of module structure (do not dump code into the body text)

**Level of detail:** Moderate. The methodology section should be a recipe, not a tutorial. Readers should understand *what* was done and *why*, not *how to install Python*. Full code belongs in the appendix.

**Common mistakes:**
- Including 3 pages of raw Python code in the methodology section (use the appendix)
- Not specifying simulation parameters precisely (duration, step sizes, solver settings)
- Omitting the scenario rationale ("We tested a 4° grade" — why 4°? What does it represent?)

#### 6. Results (3–4 pages)

**Purpose:** Present simulation outputs — figures and tables — with minimal interpretation. The results section is a data presentation section, not a discussion section.

**Content:**
1. **Open-loop verification** — step response confirming τ ≈ 64 s, K ≈ 0.040
2. **Servo performance comparison** — all-controller velocity overlay (Plot B1), metrics table
3. **Disturbance rejection comparison** — all-controller grade response (Plot C1), metrics table
4. **Linearization validation** — linear vs. nonlinear at multiple step sizes (Plot D1)
5. **IMC tuning sweep** — λ variation (Plot B3)
6. **Robustness** — mass variation overlay (Plot E1)
7. **Anti-windup comparison** — with/without windup (Plot F1)
8. **Summary metrics table** — all controllers × all metrics for S1 and D1

**Level of detail:** Let the figures speak. Each figure needs a descriptive caption, properly labeled axes with units, and a legend. The text should state what the figure shows and highlight key numerical values, but save interpretation for the Discussion section.

**Presentation order:** Follow the logic of progressive complexity: open-loop → closed-loop nominal → controller comparison → robustness → practical concerns. This mirrors the Simulation Execution Sequence from Section 4.

**Common mistakes:**
- Combining results and discussion in a single section (makes both weaker; separate them)
- Poorly formatted plots (missing axis labels, no units, no legend, tiny fonts, undifferentiated line styles)
- Presenting 15 nearly identical plots without curation (select the 6–8 most impactful; move the rest to the appendix)
- Not including a summary comparison table (the single most useful presentation of results)

#### 7. Discussion (2–3 pages)

**Purpose:** Interpret results, compare controllers, discuss limitations, and justify the recommendation. This section distinguishes a competent report from an excellent one.

**Content — draw from Section 5:**
1. **Controller comparison** — use the evaluation hierarchy (e_ss → disturbance rejection → transient performance → complexity). P is eliminated by mandatory criteria; PI and PID compared quantitatively; PI recommended.
2. **Tuning method comparison** — IMC vs. ZN vs. ITAE. IMC wins on suitability; ZN produces unacceptable overshoot; ITAE serves as a cross-check.
3. **Linearization validity** — the step-size where linear and nonlinear diverge. This is the single highest-value discussion point for a PDC course.
4. **Robustness assessment** — which specifications hold across mass/speed/drag variation; which degrade and by how much.
5. **Practical considerations** — actuator saturation, anti-windup, noise sensitivity, implementation in digital ECU.
6. **Limitations** — explicitly state what the model does not capture (powertrain dynamics, tire slip, wind gusts, multi-gear operation) and how those limitations affect the conclusions.

**How to structure the discussion:** Lead with the strongest findings, not the weakest. Begin with the definitive conclusion ("PI is recommended") and then present the evidence. Do not build suspense — this is an engineering report, not a mystery novel.

**Common mistakes:**
- Restating results without interpreting them ("The rise time was 4.8 s" — that's a result, not a discussion)
- Discussing limitations without stating whether they affect the conclusions ("The model ignores tire slip" — does this matter for the PI recommendation? Say so.)
- Being overly cautious about the recommendation ("PI may be suitable in some cases..." — commit to the recommendation and defend it)

#### 8. Conclusion (0.5–1 page)

**Purpose:** Summarize the key findings and state the recommendation. No new information should appear here.

**Content — exactly five points:**
1. A first-order plant G(s) = 0.0401/(64.1s+1) was derived from first principles using nonlinear force-balance modeling and linearization at v₀ = 25 m/s.
2. PI control with IMC tuning (λ = 5 s, K_p = 320 N/(m/s), τ_I = 64.1 s) is recommended. It achieves zero steady-state error, zero overshoot, and adequate disturbance rejection. The rise time at λ = 5 (~11 s) exceeds the CTMS benchmark target of < 5 s; reducing λ to ~2–3 s meets the rise time spec at the cost of reduced robustness margin. The λ-sweep analysis quantifies this tradeoff.
3. P control is inadequate due to persistent 7.8% steady-state error. PID provides marginal improvement over PI at the cost of noise sensitivity and complexity.
4. The linearization is valid within approximately ±5 m/s of the operating point, confirmed by nonlinear simulation comparison.
5. The controller is robust to vehicle mass variations of 1200–2000 kg, maintaining stability and zero steady-state error across the range.

**Common mistakes:**
- Introducing new results or analysis in the conclusion
- Being vague ("The controller worked well" — state the numbers)
- Omitting the recommendation (the conclusion must answer: "What controller should be used?")
- Excessive length — if the conclusion exceeds one page, it contains too much detail

#### References

**Format:** Use a consistent citation style (IEEE numbered or Harvard author-date). At minimum, cite:

1. Åström, K.J. and Murray, R.M. (2021). *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed. Princeton University Press.
2. Seborg, D.E., Edgar, T.F., Mellichamp, D.A., and Doyle, F.J. (2016). *Process Dynamics and Control*, 4th ed. Wiley.
3. University of Michigan CTMS. "Cruise Control: System Modeling." ctms.engin.umich.edu.
4. Kantor, J.C. CBE 30338 Chemical Process Control. jckantor.github.io/CBE30338/.
5. Dorf, R.C. and Bishop, R.H. (2017). *Modern Control Systems*, 13th ed. Pearson.

**Common mistakes:**
- Citing Wikipedia or generic websites instead of textbooks
- Not citing the CTMS tutorials (the benchmark parameters m = 1000, b = 50 originate there)
- Inconsistent formatting (mixing IEEE and Harvard within the same reference list)

#### Appendix

**Content:**
- A.1: Complete Python code (well-commented, organized by module)
- A.2: Additional simulation plots not included in the main text
- A.3: Detailed ITAE/ZN tuning calculations if performed
- A.4: Full metrics tables for all scenarios

---

## 2. Writing and presentation best practices

### How to write derivations professionally

**The golden rule: every equation must be preceded by a sentence explaining what operation is being performed and followed by a sentence interpreting the result.**

Bad example:
```
m(dv/dt) = u - ½ρC_dAv² - C_rr·m·g

v² ≈ v₀² + 2v₀·δv

G(s) = K/(τs+1)
```

Good example:
```
Applying Newton's second law along the direction of motion:

  m(dv/dt) = u - ½ρC_dAv² - C_rr·m·g                    (1)

The quadratic drag term is the source of nonlinearity. Expanding v² 
in a first-order Taylor series about the operating point v₀:

  v² ≈ v₀² + 2v₀·δv                                       (2)

where δv = v − v₀ is the velocity deviation from the operating point.
Substituting (2) into (1), subtracting the steady-state force balance,
and taking the Laplace transform yields the linearized transfer function:

  G(s) = K/(τs + 1)                                         (3)

where K = 1/(ρC_dAv₀) = 0.0401 (m/s)/N is the process gain and 
τ = m/(ρC_dAv₀) = 64.1 s is the process time constant.
```

**Key practices:**
- Number all important equations for easy reference in the discussion
- Define every symbol when it first appears, with units
- State the physical meaning of key results immediately after deriving them
- Use words like "Substituting," "Rearranging," "Applying the Laplace transform" to narrate the mathematical steps
- Maintain consistent notation throughout (use δv everywhere, never switch to Δv or v')

### How much detail to include vs. omit

| Include in the body | Omit from the body (put in appendix or skip) |
|---|---|
| The v² Taylor expansion (core linearization step) | Detailed algebra for simplifying intermediate fractions |
| The steady-state force calculation with numbers | Repeated plug-and-substitute for multiple operating points |
| The transfer function in standard form | Intermediate Laplace transform algebra |
| The IMC tuning formula derivation | ZN/Cohen-Coon formula tables (cite the textbook) |
| Closed-loop TF for PI (the main result) | Closed-loop TF for PID (secondary; state the result) |
| Summary metrics table | Individual scenario metrics (put the full table in appendix) |

**Rule of thumb:** If a derivation step could be performed by any competent student following the textbook, summarize it in one sentence ("Applying the Laplace transform to the linearized ODE yields..."). If a step requires particular care or is a common source of errors (the v² expansion, the steady-state subtraction), show it in full.

### How to present assumptions and approximations

**Each assumption should be presented in a three-part structure:**

1. **Statement:** What is assumed (e.g., "Rolling resistance coefficient C_rr is constant and independent of velocity")
2. **Justification:** Why this is reasonable (e.g., "C_rr varies by less than 5% over the 15–35 m/s operating range; the variation is negligible compared to the quadratic drag variation")
3. **Consequence:** What happens if the assumption is violated and whether it affects the conclusions (e.g., "Velocity-dependent C_rr would add a small linear damping term to b_eff, changing τ by approximately 2%. This does not affect the controller recommendation.")

**Avoid:**
- Listing assumptions without justification ("We assume flat road" — why?)
- Claiming assumptions are "standard" without citing a source
- Making assumptions that contradict the model used (e.g., "We assume linear drag" while using the nonlinear force-balance model)

### How to present plots and tables effectively

**Plots:**
- Every plot needs a numbered figure caption: "Figure 3: Velocity response to a 5 m/s step set-point change for all controllers."
- Axis labels must include units: "Velocity (m/s)", "Time (s)"
- Specification thresholds should be drawn as horizontal/vertical dashed lines (e.g., v_ref, ±2% band, 10% overshoot level)
- Use the dual-subplot format (velocity above, control effort below) for all transient response plots
- Reference specific figures by number in the text: "As shown in Figure 3, the PI controller achieves zero steady-state error while the P controller settles at 29.6 m/s."

**Tables:**
- Use tables for comparative data across controllers or scenarios
- Every table needs a numbered caption
- Bold or highlight the best value in each column (e.g., bold the lowest rise time)
- Include units in column headers
- Add a "Specification" row at the top or bottom for easy pass/fail visual assessment

---

## 3. Strong technical discussion strategy

### Discussion section architecture

A strong discussion follows this five-paragraph structure (each "paragraph" may be multiple paragraphs in practice):

**Paragraph 1: State the main finding.** "The PI controller with IMC tuning at λ = 5 s is the recommended design. It achieves zero steady-state error for both set-point tracking and disturbance rejection, with a rise time of X s, Y% overshoot, and a peak disturbance deviation of Z m/s under a 4° grade."

**Paragraph 2: Explain why the recommendation holds.** Connect the finding to the plant structure: "The first-order plant with no zeros and a single stable pole is ideally suited to PI control. The integral action eliminates steady-state error (which the P controller cannot achieve, as demonstrated by the 7.8% offset in Figure X). The derivative action of PID provides less than W% improvement in disturbance rejection for this plant, which has no phase lag requiring lead compensation."

**Paragraph 3: Address the tuning methodology.** "IMC tuning is preferred over Ziegler–Nichols for this dead-time-free plant. The ZN-tuned PI produces 25% overshoot (Figure Y), which violates the < 10% specification and would cause uncomfortable speed oscillation. The IMC parameter λ directly specifies the closed-loop time constant, providing transparent, physically interpretable tuning."

**Paragraph 4: Discuss limitations and robustness.** "The linearized model is accurate to within V% for perturbations of ±5 m/s from the operating point (Figure Z). Beyond this range, the quadratic drag nonlinearity causes the linear model to overestimate the velocity response. The controller compensates for this through integral action, which adjusts the steady-state throttle regardless of model accuracy. Robustness testing across m = 1200–2000 kg confirms that all specifications except rise time are met at extreme mass values."

**Paragraph 5: Place the work in context.** "These findings are consistent with the treatments by Åström and Murray and the CTMS tutorials, which both recommend PI control for cruise control. The contribution of this project is the quantitative demonstration of linearization validity bounds and the systematic comparison of tuning methods for the Åström and Murray parameter set."

### How to discuss limitations professionally

**Do:**
- State each limitation explicitly and assess its impact
- Distinguish between limitations that affect the controller recommendation (important) and those that do not (acknowledge but dismiss)
- Frame limitations as avenues for future work rather than apologies

**Do not:**
- Hide limitations or pretend they do not exist
- Catalogue every possible limitation (most are irrelevant; be selective)
- Allow limitations to undermine the recommendation without justification

**Example (strong):**
"The model assumes constant vehicle mass, which varies with passenger and cargo loading (1200–2000 kg range). Robustness testing confirms that the PI controller maintains zero steady-state error and acceptable transient performance across this range. Mass variation therefore does not invalidate the controller recommendation, though rise time degrades by approximately 40% at the upper mass bound."

**Example (weak):**
"The model has many limitations. The mass is assumed constant, the wind is ignored, the road is flat, tire slip is neglected, and the engine model is simplified. These limitations may affect the results."

### How to justify the controller recommendation decisively

The recommendation should be stated as a **conclusion supported by evidence**, not as a tentative suggestion:

**Strong:** "PI control with IMC tuning is the recommended design for this cruise control system. The evidence supporting this recommendation is threefold: (1) PI achieves zero steady-state error, which P cannot (Figure X); (2) PI meets all transient specifications without the noise sensitivity of PID (Table Y); (3) IMC tuning produces smooth, non-oscillatory responses, unlike ZN tuning (Figure Z)."

**Weak:** "Based on the simulation results, PI seems to be a good choice for this system. PID might also work well in some cases."

---

## 4. Likely viva and discussion questions

### Category A: Modeling choices (high probability)

**A1. "Why did you choose this particular model? Why not the simpler linear model?"**

The nonlinear force-balance model with linearization was chosen because it demonstrates the *core PDC workflow*: physical modeling → nonlinear ODE → linearization → transfer function → controller design. The simpler linear model (m·dv/dt = u − bv) skips the linearization step, which is the most important technique taught in CHE F342. Additionally, the nonlinear model uses physically meaningful parameters (ρ, C_d, A, C_rr) while the linear model uses a lumped damping coefficient b that has no direct physical interpretation and cannot be independently measured.

**A2. "What are the Åström and Murray parameters and why did you use them?"**

m = 1600 kg (typical sedan), C_d = 0.32 (sedan drag coefficient), A = 2.4 m² (frontal area), ρ = 1.3 kg/m³ (sea-level air density), C_rr = 0.01 (standard tire rolling resistance). These are from Chapter 3 of Åström and Murray's *Feedback Systems*, which is the most widely cited cruise control parameter set in the controls education literature. Using established parameters ensures reproducibility and allows comparison with published results.

**A3. "Why v₀ = 25 m/s as the operating point?"**

25 m/s = 90 km/h is a representative highway cruising speed in the middle of the typical operating range (80–120 km/h). It is the same operating point used by Åström and Murray, facilitating comparison. Choosing the mid-range ensures that the linearization is valid for perturbations in both directions (acceleration and deceleration).

### Category B: Assumptions (high probability)

**B1. "What is the most important assumption in your model?"**

The linearization of the quadratic drag term v² ≈ v₀² + 2v₀·δv. This assumption converts the nonlinear ODE into a linear one, enabling the entire transfer function framework. Without it, Laplace transforms cannot be applied and the classical control design methodology does not work. The assumption is valid for perturbations small compared to v₀ — specifically, when (δv/v₀)² ≪ 1.

**B2. "What happens to your model at very low speeds?"**

At v₀ → 0, the aerodynamic damping ρC_dAv₀ → 0, and both K and τ → ∞. The plant approaches a pure integrator G(s) ≈ b/s. The PI controller designed for v₀ = 25 m/s would have excessive gain at low speeds, producing very aggressive, potentially saturating control action. The linearization also breaks down because v₀ is no longer a valid reference for "small" perturbations. In practice, cruise control systems are designed to disengage below 25–30 km/h.

**B3. "Why did rolling resistance vanish from the linearized model?"**

Rolling resistance F_roll = C_rr·m·g is constant — it does not depend on velocity. Its derivative with respect to v is zero: ∂F_roll/∂v = 0. In the deviation-variable formulation, the constant C_rr·m·g appears in both the dynamic equation and the steady-state equation. When the steady-state is subtracted (the fundamental step of deviation-variable analysis), the rolling resistance cancels completely. It sets the operating point (determining how much steady-state force is needed) but has no influence on the dynamics around that operating point.

### Category C: Linearization (very high probability — the most likely viva topic)

**C1. "Walk me through the linearization of v²."**

"We expand v² = (v₀ + δv)² = v₀² + 2v₀·δv + (δv)². For small perturbations, (δv)² is much smaller than 2v₀·δv — for example, if δv = 1 m/s and v₀ = 25 m/s, then (δv)² = 1 while 2v₀·δv = 50. The quadratic term is 2% of the linear term, so we neglect it. The linearized drag is then ½ρC_dA(v₀² + 2v₀·δv). The constant term v₀² cancels with the steady-state equation, leaving only the perturbation drag ρC_dAv₀·δv, which is linear in δv."

**C2. "What does 'small perturbation' mean quantitatively?"**

The Taylor expansion error is proportional to (δv/v₀)². At δv = 2.5 m/s (10% of v₀), the error is (2.5/25)² = 1%. At δv = 5 m/s (20%), the error is 4%. At δv = 12.5 m/s (50%), the error is 25%. The simulation comparison (Scenario E1) provides the definitive answer: linearization diverges visibly at ±5 m/s from v₀ and becomes unreliable beyond ±10 m/s.

**C3. "Why can't you take the Laplace transform of v² directly?"**

The Laplace transform is a linear operator — it satisfies L{αf + βg} = αL{f} + βL{g}. The product of two time-domain functions (v·v = v²) does not have a simple Laplace-domain equivalent. L{v²} ≠ V(s)². The transform of a product is a convolution in the s-domain: L{v²} = V(s) * V(s)/(2πj), which cannot be solved algebraically. This is precisely *why* linearization is necessary — it converts the nonlinear term into a linear one that the Laplace transform can handle.

### Category D: Transfer function (high probability)

**D1. "What is your transfer function and what does each parameter mean physically?"**

G(s) = K/(τs + 1) = 0.0401/(64.1s + 1). The time constant τ = m/(ρC_dAv₀) = 64.1 s is the ratio of vehicle inertia to aerodynamic damping — physically, it is the time for the vehicle to reach 63.2% of a new equilibrium speed after a throttle step. The gain K = 1/(ρC_dAv₀) = 0.0401 (m/s)/N is the steady-state speed change per Newton of additional force — physically, how much faster the car eventually goes for each Newton of extra push. Both parameters depend on the operating speed v₀ — a crucial feature that makes the linearized model operating-point-dependent.

**D2. "Is the plant stable? How do you know?"**

The pole is at s = −1/τ = −0.0156 s⁻¹, in the left half of the s-plane. A single real negative pole corresponds to a stable, first-order system. Physically, stability arises because aerodynamic drag increases with speed — any speed increase is self-correcting through increased drag. The plant is stable for all realistic parameter values (ρ > 0, C_d > 0, A > 0, v₀ > 0, m > 0).

**D3. "What is the disturbance transfer function and why does it share the same pole?"**

G_d(s) = −c/(s + a) = −628/(64.1s + 1). It shares the same pole (−1/τ) because both the input force and the grade disturbance act on the same physical system — the vehicle mass m being damped by the same aerodynamic force. The denominator of both transfer functions comes from the plant's characteristic equation, which is determined by the system's dynamics, not by the input channel. The sharing of poles is a fundamental property of linear systems and means the vehicle responds to hills on the same timescale as to throttle changes.

### Category E: Controller selection (high probability)

**E1. "Why PI and not PID?"**

Three reasons. First, the plant is first-order with no zeros — there is no phase lag from additional poles that requires lead compensation from a derivative term. Second, the derivative term amplifies measurement noise from wheel-speed sensors, producing throttle chatter without meaningful performance improvement. Third, quantitative simulation shows that PID improves peak disturbance deviation by only X% compared to PI, while adding a tuning parameter, a derivative filter, and noise sensitivity. The cost-benefit tradeoff decisively favors PI for this plant.

**E2. "Why not use PID anyway? It won't make things worse, will it?"**

It can, in two ways. First, derivative action amplifies high-frequency noise, causing rapid oscillations in the throttle command that create uncomfortable ride quality — a performance degradation in the noise sensitivity dimension even if set-point tracking improves marginally. Second, derivative kick on set-point changes (unless mitigated with derivative-on-PV) causes throttle impulses that produce harsh jerk forces. The claim that "PID cannot be worse than PI" holds only for noiseless, continuous-time, unconstrained implementations — none of which apply in practice.

**E3. "Why IMC tuning and not Ziegler–Nichols?"**

IMC is designed for plants with known models; ZN is designed for plants where only input-output data is available. We have a derived model — using ZN would discard this information. Furthermore, ZN targets a quarter-decay-ratio response (≈25% overshoot), which is unacceptable for cruise control comfort. IMC produces a single tuning parameter λ that maps directly to the desired closed-loop time constant, providing transparent, predictable tuning. The ZN-tuned PI's 25% overshoot was confirmed in simulation, validating this assessment.

### Category F: Simulation (moderate probability)

**F1. "Why did you use both a linear and nonlinear simulation?"**

The linear model verifies controller math — it confirms that the closed-loop transfer function produces the predicted response. The nonlinear model verifies physical validity — it checks whether the linearized controller works on the actual (nonlinear) plant. The comparison between them quantifies the linearization accuracy, which is a core PDC concept. Without both, either the controller math or the physical validity would be untested.

**F2. "What happens in your simulation when the throttle saturates?"**

Without anti-windup: the integrator continues accumulating error during saturation, building up a large integral value. When the error drops and the desired control effort falls below the saturation limit, the accumulated integral keeps the throttle high, causing overshoot. With back-calculation anti-windup: the difference between the desired and saturated control signals is fed back to reduce the integrator, preventing windup and eliminating the excess overshoot.

---

## 5. Deep follow-up and stress-test questions

### "What if" questions (testing conceptual depth)

**Q1. "What if the road grade is not constant but varies sinusoidally? Would your controller still work?"**

Yes, PI would still reject sinusoidal grade disturbances, but with finite attenuation rather than zero error. The disturbance rejection depends on the frequency: at low frequencies (slow grade changes like rolling hills), the loop gain is high and attenuation is excellent. At high frequencies (rapid terrain changes), the loop gain drops and the disturbance passes through to velocity. The Bode plot of T_d(jω) shows this frequency-dependent attenuation quantitatively. Steady-state error remains zero for any constant or step disturbance component, but sinusoidal disturbances at finite frequency produce sinusoidal velocity error with amplitude |T_d(jω)|.

**Q2. "What if the vehicle mass suddenly doubles mid-simulation (e.g., a trailer is attached)?"**

The plant gain K is unaffected (K = 1/(ρC_dAv₀), independent of mass), but the time constant doubles (τ = m/(ρC_dAv₀)). The PI controller tuned for τ = 64 s would now control a plant with τ = 128 s. The pole-zero cancellation in the IMC design becomes imperfect (controller zero at −1/64.1 does not cancel the new plant pole at −1/128), creating a slow residual mode. The response would become sluggish but remain stable — integral action still guarantees zero steady-state error. The closed-loop would behave as a second-order system with a fast mode (from the controller) and a slow mode (from the uncancelled plant pole).

**Q3. "What if there is a 2-second time delay in the throttle actuator? How does that change your analysis?"**

A 2-second dead time converts the plant from G(s) = K/(τs + 1) to G(s) = Ke^{−2s}/(τs + 1). This fundamentally changes the control problem: the dead time limits the achievable closed-loop bandwidth (the IMC parameter λ must satisfy λ > 2 s for stability), introduces a gain margin limitation, and makes the system potentially unstable for high gains. ZN would become more appropriate in this case because it was designed for FOPDT plants. The IMC tuning would still work but with a modified formula: K_p = τ/(K(λ + L)) where L is the dead time. PID would also become more valuable — the derivative term can partially compensate for dead time by providing phase lead.

**Q4. "Why does the linearized model overestimate or underestimate the response for large perturbations?"**

For a speed increase (v > v₀), the actual drag is ½ρC_dAv², which is larger than the linearized drag ½ρC_dA(v₀² + 2v₀·δv) because the neglected (δv)² term is positive. Higher actual drag means more damping than the linear model predicts, so the nonlinear response settles *faster* and to a *lower* steady-state velocity than the linear prediction. For a speed decrease (v < v₀), the opposite holds. This asymmetry is visible in the linear vs. nonlinear comparison plot and follows directly from the concavity of the v² function.

**Q5. "Could you use a gain-scheduled controller instead of a fixed-gain PI?"**

Yes. Since K and τ both depend on v₀, a gain-scheduled controller would recompute K_p = τ(v)/(K(v)·λ) and τ_I = τ(v) at each moment based on the current speed. This would maintain optimal performance across all speeds, unlike the fixed-gain design that degrades at speeds far from 25 m/s. Gain scheduling is standard in production cruise control ECUs. It was not implemented in this project because (1) fixed-gain PI demonstrates the core PDC concepts more clearly, (2) gain scheduling requires continuous parameter identification or lookup tables, adding significant complexity, and (3) the robustness analysis shows that fixed-gain PI provides acceptable performance across 15–35 m/s.

### Conceptual depth questions

**Q6. "What is the physical interpretation of the integral action in your PI controller?"**

The integrator accumulates the velocity error over time. Physically, it represents the controller "remembering" how long and how much the car has been below (or above) the target speed, and continuously adjusting throttle until the error is eliminated. The integral state after settling represents the difference between the actual steady-state force required (which depends on road grade, wind, mass) and the nominal force u₀ assumed by the controller. In equilibrium, the integrator holds exactly the force deficit that the proportional term cannot supply — this is how PI achieves zero error without knowing the disturbance precisely.

**Q7. "Explain the connection between your cruise control model and a CSTR."**

Both are first-order nonlinear systems that are linearized around an operating point. In a CSTR, the nonlinearity comes from the Arrhenius rate law k(T) = Ae^{−E_a/RT}; in cruise control, it comes from quadratic drag ½ρC_dAv². Both systems have a single state variable (concentration / velocity), a manipulated input (coolant flow / throttle force), and a disturbance (feed concentration / road grade). The linearization procedure is identical: Taylor-expand the nonlinear terms, subtract the steady state, take Laplace transforms. The resulting transfer functions are first-order with operating-point-dependent gain and time constant. A PI controller designed for one system at one operating point degrades at other operating points for the same mathematical reason — the linearization coefficients change.

---

## 6. Strong answer frameworks

### Framework for answering "Why did you choose X?" questions

**Structure: Reason → Evidence → Alternative → Why alternative is worse**

"We chose [X] because [specific technical reason, not generic claim]. This is supported by [simulation result, analytical derivation, or textbook reference]. The alternative [Y] was considered but rejected because [specific deficiency for this plant]. For example, [concrete comparison showing X outperforms Y]."

**Example application:**

"We chose IMC tuning because it directly targets the closed-loop time constant through the single parameter λ, which is physically interpretable as how fast the car responds to speed changes. This is supported by the λ-sweep simulation (Figure B3), which shows a monotonic tradeoff between speed and robustness. Ziegler–Nichols was considered but rejected because it targets a quarter-decay-ratio response, producing 25% overshoot as confirmed in simulation (Figure B1). For a cruise control application where speed-limit overshoot is unacceptable, IMC's guaranteed zero overshoot (under pole-zero cancellation) is a decisive advantage."

**Pitfalls to avoid:**
- "We chose X because the textbook said so" (appeal to authority)
- "We chose X because it was easier" (admits laziness, not engineering judgment)
- "We didn't have time to try Y" (undermines the recommendation's credibility)

### Framework for answering "What are the limitations?" questions

**Structure: Limitation → Impact assessment → Whether it affects the conclusion**

"The model assumes [limitation]. This matters because [specific physical consequence]. However, [it does / does not] affect our controller recommendation because [specific reason with quantitative evidence if available]."

**Example application:**

"The model assumes constant vehicle mass. In practice, mass varies from approximately 1200 kg (driver only) to 2000 kg (fully loaded). This affects the time constant (τ ∝ m) and therefore the transient response. However, it does not affect our controller recommendation because (1) the integral action ensures zero steady-state error at all masses, and (2) the robustness simulation shows all specifications except rise time are met across this range. The rise time degradation at m = 2000 kg (estimated 40% increase) is acceptable because safety is not compromised by slightly slower speed changes."

### Framework for answering "What would you do differently?" questions

This is often the final viva question and tests whether the student has reflected critically on the work.

**Strong answer structure:** Acknowledge one specific improvement + explain exactly how it would strengthen the project.

"If repeating this project, I would implement gain scheduling — recomputing K_p and τ_I as functions of the current velocity using the known relationship τ(v) = m/(ρC_dAv₀) and K(v) = 1/(ρC_dAv₀). This would maintain optimal tuning across all speeds instead of relying on robustness at the design point. The fixed-gain approach was appropriate for demonstrating PDC fundamentals, but gain scheduling would bridge the gap between our academic design and production cruise control systems."

---

## 7. Final quality checklist

### Technical completeness

- [ ] Nonlinear ODE derived from Newton's second law with all four forces
- [ ] Steady-state operating point calculated numerically (u₀ = 468.8 N at v₀ = 25 m/s)
- [ ] Taylor expansion of v² shown explicitly (the single most important derivation step)
- [ ] Deviation variables defined; steady-state subtracted
- [ ] Transfer function derived in standard form G(s) = K/(τs + 1) with numerical values
- [ ] Disturbance transfer function derived
- [ ] Dimensional analysis of all terms
- [ ] Physical interpretation of K and τ
- [ ] P, PI, and PID controllers designed with closed-loop transfer functions
- [ ] IMC tuning parameters computed (K_p = 320, τ_I = 64.1 for λ = 5)
- [ ] Controller comparison table with all metrics

### Mathematical correctness

- [ ] All equations numbered and referenced
- [ ] Units verified for every term in every equation
- [ ] Linearization coefficients match v₀ = 25 m/s: b_eff = ρC_dAv₀ = 24.96 N·s/m
- [ ] τ = m/b_eff = 1600/24.96 = 64.1 s ✓
- [ ] K = 1/b_eff = 1/24.96 = 0.0401 (m/s)/N ✓
- [ ] Closed-loop time constant for P: τ_CL = τ/(1 + K_pK) ✓
- [ ] IMC parameters satisfy K_p = τ/(Kλ) and τ_I = τ ✓
- [ ] ZN parameters computed correctly if included

### Simulation completeness

- [ ] Open-loop step response verifies τ and K
- [ ] Closed-loop step response for all controllers (Scenario S1)
- [ ] Disturbance rejection for all controllers (Scenario D1)
- [ ] Linear vs. nonlinear comparison at multiple step sizes (Scenario E1)
- [ ] IMC λ-sweep (Scenario E3)
- [ ] Mass variation robustness (Scenario R1)
- [ ] Anti-windup demonstration (Scenario E2)
- [ ] Metrics computed and tabulated for all configurations
- [ ] At least 6 well-formatted, publication-quality figures

### Presentation quality

- [ ] Abstract written (150–250 words, contains quantitative results)
- [ ] All figures have numbered captions, axis labels with units, legends
- [ ] All tables have numbered captions and units in headers
- [ ] Equations numbered sequentially
- [ ] Notation consistent throughout
- [ ] References formatted consistently (IEEE or Harvard)
- [ ] No spelling or grammar errors in captions and headings
- [ ] Report reads as a coherent narrative, not a disconnected sequence of calculations

### Academic rigor

- [ ] Every assumption is stated, justified, and its impact assessed
- [ ] Controller recommendation is stated explicitly with supporting evidence
- [ ] Tuning methodology is justified (not just "we used IMC")
- [ ] Linearization validity is quantified through simulation comparison
- [ ] Limitations are discussed with impact assessment, not just listed
- [ ] Results are interpreted, not just presented
- [ ] Competing controllers are fairly compared on the same scenarios
- [ ] The discussion section contains engineering judgment, not just restated data

### Viva readiness

- [ ] Can derive the v² linearization on a whiteboard from memory
- [ ] Can state τ, K, u₀ numerical values and explain their physical meaning
- [ ] Can explain why rolling resistance vanishes from the linearized model
- [ ] Can justify PI over P (integral action eliminates steady-state error)
- [ ] Can justify PI over PID (first-order plant, noise sensitivity, marginal improvement)
- [ ] Can justify IMC over ZN (model-based, no overshoot, single parameter)
- [ ] Can explain what the integrator physically "does" in PI control
- [ ] Can describe what happens when the throttle saturates (windup)
- [ ] Can explain the CSTR analogy for the cruise control plant
- [ ] Can answer "What would you do differently?" with a specific, thoughtful improvement
