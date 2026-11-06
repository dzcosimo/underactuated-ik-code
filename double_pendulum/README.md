# Tendon-driven double pendulum: Algorithms 1 and 2 (Figs. 7, 10-13, 15-16)

Code for the double-pendulum examples of

> C. Della Santina, *Pushing the boundaries of actuators-to-task kinematic inversion: from fully
> actuated to underactuated (soft) robots*, IJRR.

Everything is plain MATLAB (no Simulink, no toolboxes). The original simulations were Simulink
models. They were ported to MATLAB and checked against those models (see [Validation](#validation)).

## The system (Sec. 2.3)

A planar double pendulum with point masses at the link tips, relative joint angles
q = (q1, q2) measured from the downward vertical (q = 0 is the hanging configuration), joint
damping D = diag(d1, d2) and parallel springs diag(k1, k2). A single tendon wraps two pulleys of
radii r1 = cos θ and r2 = sin θ, so the actuation matrix is A = [r1; r2] (Eq. (5)) and the tendon
tension τ is the only input (n = 2, m = 1). The dynamics are Eq. (4) with the potential of
Eq. (6). The task is the horizontal end-effector position
x = h(q) = l1 sin q1 + l2 sin(q1 + q2) (Eq. (7); the code writes it as
l1 cos(q1 − π/2) + l2 cos(q1 + q2 − π/2)).

Fixed parameters (all cases): l1 = l2 = 1, m1 = m2 = 1, d1 = d2 = 10, α = π/2.
Actuation coordinates (Sec. 2.5.1, Eqs. (16), (19)): qa = A^T q, and qu is the orthogonal
direction (α = π/2). The unactuated potential force Gu = [0 1] T^T G (Eq. (17)) is zero exactly
on the attainable-equilibria manifold E (Eq. (18)).

## Cases and figures

`run_all_double_pendulum` runs every case and writes `figures/<name>.png` and `.eps`.

| Paper figure | Case (all with α = π/2) | Output files (`figures/`) | Original file in the paper sources |
|---|---|---|---|
| Fig. 7 (a,b) | Alg. 1 (Sec. 4.5), base actuation: θ = 0, k1 = k2 = 0, g = 9.81, q̄u = −qa (Eq. (28)), Γ = 1, x̄ = −1, q0 = (0, π/2) | `fig07a_alg1_base_actuation_robot`, `fig07b_alg1_base_actuation_q`, `fig07b_alg1_base_actuation_x` | `doublePendulum_alg1_k0_beta0_qopi2_2`, `_q_1`, `_x_1` |
| Fig. 7 (c,d) | Alg. 1, general actuation without gravity: θ = π/8, k1 = k2 = 10, g = 0, q̄u from Eq. (31), Γ = 1, x̄ = −1, q0 = (0, 0) | `fig07c_alg1_no_gravity_robot`, `fig07d_alg1_no_gravity_q`, `fig07d_alg1_no_gravity_x` | `doublePendulum_alg1_k1010_betapi8_qopi2_2`, `_q_2`, `_x_2` |
| Fig. 10 | Alg. 2 (Sec. 5.3): θ = π/8, k1 = k2 = 0, g = 9.81, initial guess q = (π/2, π/2), x̄ = 0.5 on [0,10), −0.5 on [10,20]; (ΓE, ΓZ) = (0,1), (1,0), (1,1) | `fig10_alg2_configuration_comparison` | `doublePendulum_alg2_k0_betapi8_qopi2pi2_part2_configurtion_comparison_1` |
| Fig. 11 (a,b,c) | the same run with ΓE = ΓZ = 1 | `fig11a_alg2_q`, `fig11b_alg2_gu`, `fig11c_alg2_x` | `doublePendulum_alg2_k0_betapi8_qopi2pi2_q_2`, `_gu_2`, `_x_2` |
| Fig. 12 (a,b) | the same run, t ∈ [0,10] and [10,20] | `fig12a_alg2_robot_t0_10`, `fig12b_alg2_robot_t10_20` | `doublePendulum_alg2_k0_betapi8_qopi2pi2_1`, `_part2_1` |
| Fig. 13 (a-i) | Alg. 2 vector field (ΓE = ΓZ = 1, g = 9.81) for (θ, k1, k2, x̄) = (0,10,0,0.5), (0,0,0,0.5), (0,0,10,0.5), (π/8,10,0,0.5), (π/8,0,0,0.5), (π/8,0,10,0.5), (π/8,10,0,−1), (π/8,0,0,−1), (π/8,0,10,−1) | `fig13a_...` to `fig13i_...` (parameters in the file name) | `doublePendulum_alg2_k<k1><k2>_beta<θ>[_rm1]_field_1` |
| Fig. 15 | Alg. 2 as a feedback loop (Sec. 5.4), nominal case: end-effector paths with FF and FB all | `fig15_fb_end_effector_paths` | `task_fb_1` |
| Fig. 16 (a,b,c) | Sec. 5.4, nominal: θ = π/8, k1 = 0, k2 = 10, g = 9.81, robot at rest in q0 = (0, π/2), CLIK state q̄(0) = 0, ΓZ = 0.5, ΓE = 1, τ = Ga(q) + 10 (q̄a − qa), x̄ = −1 / +1 / −1 on [0,30) / [30,60) / [60,90]; variants FF, FB task, FB all | `fig16a_fb_nominal_q`, `fig16b_fb_nominal_qbar`, `fig16c_fb_nominal_x` | `comparison_fb_nominal_q_1`, `_q_bar_1`, `_x_1` |
| Fig. 16 (d,e,f) | the same with imperfect cancellation τ = 0.5 Ga(q) + 10 (q̄a − qa) | `fig16d_fb_perturbed_q`, `fig16e_fb_perturbed_qbar`, `fig16f_fb_perturbed_x` | `comparison_fb_robust_q_1`, `_q_bar_1`, `_x_1` |

The feedback variants of Fig. 16 (`src/clik_alg2_fb.m`): **FF** is Eq. (42) with the task
error x̄ − h(q̄). **FB task** uses the measured x instead (blue path of Fig. 14). **FB all**
also evaluates Ja at the measured q (blue and orange paths of Fig. 14).

Figs. 7, 10-12 integrate the CLIK dynamics only. They are not simulations of the robot. Figs.
15-16 are full dynamic simulations: CLIK, then the low-level controller, then Eq. (4).

The parameters and gains in the table were taken from the paper captions. They were checked
against the data stored in the original `.fig` files, by re-running the original Simulink models
and matching the stored trajectories to machine precision. Three values are not stated in the
paper: the Alg. 2 springs of Figs. 10-12 (k1 = k2 = 0), q0 = 0 in Fig. 7(c,d), and ΓZ = 0.5,
Kp = 10 in Figs. 15-16.

## How to run

From MATLAB, in any folder:

```matlab
run('<path>/double_pendulum/run_all_double_pendulum.m')
```

or, inside `double_pendulum/`, simply `run_all_double_pendulum`. It writes `figures/` (28 figures, each as .png and .eps)
and `results/double_pendulum_results.mat` (struct `R` with every trajectory). Single cases can
be run with the functions in `src/`, e.g.

```matlab
setup_paths
p = dp_params('theta', pi/8, 'q0', [pi/2; pi/2]);
R = sim_alg2(p, 1, 1, @(t) 0.5, [0 10], (0:0.01:10)');   % Alg. 2, xbar = 0.5
plot_task_time(R.t, R.x, R.xbar);
```

**Run time:** about 2 minutes in total with MATLAB R2025a on an Apple-silicon laptop. Each
simulation takes about 1 s. Most of the time goes to the 9 vector fields of Fig. 13 and to
exporting the figures.

## Requirements

* MATLAB, developed and tested with **R2025a**. Older releases should work. The function
  `theme` is used only if it exists, to force light figures.
* No toolbox for the simulations and figures.
* **Symbolic Math Toolbox** only to regenerate `model/generated/` with
  `model/support_calculate_symbolic.m`. You do not need to do this: the generated functions are
  included.

## Folder layout

```
double_pendulum/
  README.md
  run_all_double_pendulum.m    ENTRY POINT: all cases -> figures/, results/
  setup_paths.m                adds src/, model/, model/generated/ to the path
  model/
    dp_params.m                parameters (Sec. 2.3); theta is "beta" of the original code
    dp_dynamics.m              M qdd + C qd + D qd + G = A tau  (Eq. (4))
    dp_gravity.m               G(q) = dU/dq, gravity + springs (Eq. (6))
    dp_actuation.m             A = [r1; r2] (Eq. (5))
    dp_point_position.m        point along the robot (drawing, end effector)
    dp_task.m                  task h(q) (Eq. (7))
    dp_act_terms.m             x, Ja, Ju, Ga, Gu, Hua, Huu at (qa, qu)
    dp_Gu.m                    Gu at (q1, q2) (manifold E, Eq. (18))
    support_calculate_symbolic.m  symbolic derivation -> generated/
    generated/                 q_to_q_act, q_act_to_q, Jacobian_q_act_to_q, Gu/Ga/Hua/Huu/Ja/Ju*_supp
  src/
    clik_alg1.m, alg1_parametrization.m     Algorithm 1, Eq. (33), with q̄u(qa) of Eqs. (28), (31)
    clik_alg2.m, projector_PE.m, projector_PZ.m, regularized_inverse.m   Algorithm 2, Eqs. (42)-(43)
    clik_alg2_fb.m, low_level_controller.m  Algorithm 2 as feedback loop (Sec. 5.4, Eq. (51))
    sim_alg1.m, sim_alg2.m, sim_alg2_fb.m   the three ported Simulink schemes
    ode_piecewise.m, default_ode_options.m  ode45 with restarts at reference switches
    plot_*.m, new_figure.m, new_time_figure.m, style_*.m, padded_ylim.m, save_figure.m   figures
  figures/                     created by run_all_double_pendulum
  results/                     created by run_all_double_pendulum
```

Correspondence with the original code: `IK_a1` → `clik_alg1`, `IK_a2` → `clik_alg2`,
`IK_a2_fb` → `clik_alg2_fb`, `calculateP_parallelG_u` / `calculateP_perpG_u` →
`projector_PE` / `projector_PZ`, `controller` → `low_level_controller`, `f_fcn` (+ `invM_fcn`,
`C_fcn`, `D_fcn`, `G_fcn`, `A_fcn`) → `dp_dynamics`, `qBarU` + `J_qBarU` → `alg1_parametrization`,
`*Function` / `*Function_act` wrappers → `dp_act_terms`, `plotDoublePendulum` /
`plotMultipleDoublePendula` → `plot_pendulum` / `plot_pendulum_sequence`, `plot_manifolds` →
`plot_vector_field`, `plot_results_IK_a*_scheme` → `plot_task_time`, `plot_q_time`,
`plot_gu_time`. The global variables of the original code are replaced by a parameter struct
`p`. The plotting code of Figs. 10, 15 and 16 did not exist as scripts (those figures were
assembled by hand). It is written here to reproduce the stored `.fig` files: same data, styles,
legends and axis limits.

## Differences from the original implementation

1. **Simulink → MATLAB.** `IK_a1_scheme.slx`, `IK_a2_scheme.slx` and `IK_a2_fb_scheme.slx` are
   replaced by `sim_alg1`, `sim_alg2` and `sim_alg2_fb`. They use the same blocks and the same
   initial conditions. The solver is MATLAB's `ode45` instead of Simulink's variable-step
   `ode45`, with tighter tolerances (RelTol 1e-8, AbsTol 1e-10 instead of 1e-4 / 1e-3). The
   integration is restarted at the reference switches (t = 10, 30, 60), which Simulink also hits
   exactly. The results are given on a uniform grid (dt = 0.01 s). With the paper settings,
   Simulink returned few points (55 for the 20 s of Alg. 2); the robot drawings interpolate
   linearly between the samples, so the end-effector paths of Figs. 12 and 15 are now smoother.
   The differences are invisible at figure scale.
2. **Angle wrapping.** The `.slx` of Sec. 5.4 wrapped the robot angles into [−π, π]. The paper
   trajectories never leave that interval, so `sim_alg2_fb` does not wrap. It warns if |q| > π.
3. **Unused integral action.** The low-level controller had an integral term with gain 0. It is
   omitted.
4. **Hessian blocks Hua, Huu** (`support_calculate_symbolic.m`). The original substituted
   q = T^{-T}(qa, qu) instead of q = T (qa, qu). The two are identical for α = π/2 and ‖A‖ = 1,
   which covers every paper case, and the new form is also correct for other α. The generated
   functions were regenerated with R2025a. `Ja`, `Ju`, `Gu`, `Ga` and the coordinate maps are
   textually identical to the originals.

### Paper vs. code (left as in the code that produced the figures)

* **Fig. 16 (d,e,f), perturbed case.** The caption says the cancellation is reduced to Ga/3.
  The stored trajectories are reproduced exactly with **0.5 Ga** (the steady state of the FF run
  gives s = 0.500001), and not with Ga/3. The release uses 0.5. With 1/3 the curves change
  noticeably: max |Δq| ≈ 0.07 for FF, and several rad for FB task.
* **Eq. (51), FB all.** The equation evaluates P_E at the measured q. The code (and therefore
  Fig. 16) evaluates P_E, P_Z, Huu and Gu at the CLIK state q̄, and only Ja and x at the measured
  q.
* **Eq. (7).** It is written with cosines. With angles measured from the gravity direction, as
  in Sec. 2.3 and in the code, the horizontal coordinate is l1 sin q1 + l2 sin(q1 + q2). The
  same applies to Eqs. (37)-(39).

## Validation

The scratch copies of the original Simulink models were run in MATLAB R2025a, and their
trajectories were compared with the port at the Simulink output times. The originals were never
modified; only their gains were set through a global variable. Maximum absolute differences:

| Case | q: port vs. Simulink as configured for the paper | q: both at RelTol 1e-10, AbsTol 1e-12 | q̄ (both tight) |
|---|---|---|---|
| Fig. 7(a,b) Alg. 1 base actuation | 3.0e-09 | 3.6e-11 | – |
| Fig. 7(c,d) Alg. 1 no gravity | 5.1e-10 | 5.1e-12 | – |
| Figs. 10-12 Alg. 2, ΓZ = ΓE = 1 | 8.2e-06 | 4.4e-10 | – |
| Fig. 10 Alg. 2, ΓZ = 1, ΓE = 0 | 8.4e-06 | 1.4e-10 | – |
| Fig. 10 Alg. 2, ΓZ = 0, ΓE = 1 | 1.6e-05 | 2.8e-10 | – |
| Fig. 16 nominal FF | 3.1e-05 | 5.2e-11 | 3.3e-10 |
| Fig. 16 nominal FB task | 2.9e-05 | 1.4e-10 | 1.9e-10 |
| Fig. 16 nominal FB all | 1.1e-03 | 2.4e-09 | 1.9e-09 |
| Fig. 16 perturbed FF | 1.9e-05 | 9.5e-11 | 3.3e-10 |
| Fig. 16 perturbed FB task | 7.6e-05 | 8.6e-11 | 7.9e-10 |
| Fig. 16 perturbed FB all | 2.5e-05 | 1.6e-10 | 7.1e-10 |

With both solvers tight, the port and the original models agree to ≤ 2.4e-9 rad. They implement
the same equations. The differences in the second column are the integration error of the paper
settings: RelTol 1e-4, or 1e-3 for Sec. 5.4. The largest one is the "FB all" run, which Simulink
integrated with only 193 steps over 90 s. The Simulink runs with the paper settings reproduce the
data in the original `.fig` files to ≤ 3e-8. Visual checks against the original figures matched
for Figs. 7, 10, 12, 13, 15 and 16.
