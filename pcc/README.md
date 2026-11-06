# Planar PCC soft robot: Algorithms 1 and 2 (Figs. 18-19)

Code for the soft-robot simulations of

> C. Della Santina, *Pushing the boundaries of actuators-to-task kinematic inversion: from fully
> actuated to underactuated (soft) robots*, IJRR.

A planar soft robot made of **5 constant-curvature (CC) segments** (piecewise constant
curvature, PCC; total length 1 m, mass 2 kg) is actuated by **2 inputs**, one per section
(segments 1-3 and 4-5): the robot is underactuated (n = 5, m = 2). The task is the 2D tip
position. Two closed-loop inverse kinematics (CLIK) laws of the paper compute a reference
configuration q̄ that a low-level controller tracks:

* **Algorithm 1** integrates the actuated coordinates q̄ₐ with a Jacobian-transpose update
  through an explicit static branch q̄ᵤ = φ(q̄ₐ) (end-to-end Jacobian J = [Jₐ Jᵤ][I; dφ/dqₐ]).
  In Figs. 18-19, φ neglects gravity (elastic statics only).
* **Algorithm 2** integrates the whole configuration [q̄ₐ; q̄ᵤ] along a vector field that is the
  sum of a task component, tangent to the static manifold Gᵤ = 0 (projector P_E, Eq. (42)), and a
  statics-relaxation component that drives Gᵤ to 0 at fixed task (projector P_Z). It needs no
  explicit solution of the statics.

Each law runs in feedforward (**ff**: task error computed on the CLIK state, x = h(q̄)) or
feedback (**fb**: x = h(q), measured robot) form. Everything is plain MATLAB (no Simulink).

## Requirements

* MATLAB (developed and tested with **R2025a**; uses `theme`, so R2025a or newer for the
  figure and video scripts; the simulation itself has no special requirements).
* No toolbox for the simulations, figures and videos.
* **Symbolic Math Toolbox** only to regenerate the model functions (`model/generate_pcc_model.m`),
  which is not needed: the generated functions are shipped in `model/generated/`.
* Optional: Parallel Computing Toolbox to run the cases in parallel with `parfor`.

## Folder layout

```
pcc/
  README.md
  setup_paths.m          adds src/, model/, model/generated/ to the path (called by the entry points)
  run_pcc_case.m         ENTRY POINT: run one case, save results/<case>.mat
  run_all_pcc.m          run every case (sequentially, or in parallel, see below)
  make_figures_pcc.m     Figs. 18 and 19 from results/  -> figures/
  make_videos_pcc.m      the videos from results/       -> videos/
  src/
    pcc_cases.m          the list of cases (names, options)
    pcc_sim.m            closed-loop simulator (reference, CLIK, saturation, controller, robot; Euler)
    clik_alg1.m          Algorithm 1 (static branch: zero / 1st / 2nd order in gravity)
    clik_alg2.m          Algorithm 2 (with the numerical escapes of the original code)
    projector_PE.m       projector P_E of Eq. (42)
    projector_PZ.m       projector P_Z
    low_level_controller.m   tau = G_a(q̄) + Kp (q̄ₐ - qₐ)
    pcc_dynamics.m       M q'' + C q' + G + D q' = A tau
    pcc_params.m         physical parameters
    pcc_model.m          model struct, stiffness scaling (kscale)
    G_total.m, Ga_total.m, Gu_total.m, Hua_total.m, Huu_total.m   potential forces / Hessians with kscale
    floor_curvature.m    curvature floor |q_i| < 1e-2 -> 1e-2 (generated functions are 0/0 at q_i = 0)
    forward_kinematics_planar_PCC.m, forward_kinematics_act.m, CC_rt_planar.m   kinematics
    robot_backbone.m     backbone points for drawing
  model/
    generate_pcc_model.m symbolic derivation of the model (Symbolic Math Toolbox)
    generated/           the generated functions used by the simulation
  results/               simulation results (created by run_pcc_case; empty in the release)
  figures/               Figs. 18 and 19 (PDF, PNG) as produced by make_figures_pcc
  videos/                the videos (MPEG-4) of all cases, see the table below
```

## How to run

From MATLAB, in the folder `pcc/`:

```matlab
run_pcc_case('stiff_a1ff')              % one case (here: Fig. 18, Alg. 1 ff), -> results/stiff_a1ff.mat
run_pcc_case('soft_a2fb', 'T', 0.5)     % Fig. 19, Alg. 2 fb, first 0.5 s only (quick test)
run_all_pcc                             % every case (edit SMOKE / USE_PARFOR at its top)
make_figures_pcc                        % Figs. 18 and 19, needs stiff_a1ff ... stiff_a2fb, soft_a1ff ... soft_a2fb
make_videos_pcc('sets', {'fig18_stiff'})            % one video set (overwrites videos/fig18_stiff)
make_videos_pcc('sets', {'fig18_stiff'}, 'outdir', 'test_videos', 'maxframes', 30)   % quick test
```

`pcc_cases()` returns the list of cases (after `setup_paths`); each result `R` (in `results/<case>.mat`) contains the
time `t`, the robot curvatures `Q`, the CLIK output `Qbar`, the task reference `xbar`, the tip
position `tip`, the number of Alg. 2 escapes `escapes` and the options `o`.

### Running the cases in parallel

The cases are independent. Either set `USE_PARFOR = true` in `run_all_pcc.m`, or start one
MATLAB per case from a shell in the folder `pcc/`, e.g.

```bash
for c in soft_a2ff soft_a2fb soft_tri_a2ff soft_tri_a2fb; do
  matlab -singleCompThread -batch "run_pcc_case('$c')" > $c.log 2>&1 &
done
```

(one process per physical core; each process needs ~0.6 GB of memory).

## Cases

| Case(s) | Robot | Targets | dt | T | Paper / video folder |
|---|---|---|---|---|---|
| `stiff_a1ff`, `stiff_a1fb`, `stiff_a2ff`, `stiff_a2fb` | stiff (kscale 1) | (0.6, 0.6) | 1e-5 s | 5 s | **Fig. 18**, `videos/fig18_stiff/` |
| `soft_a1ff`, `soft_a1fb`, `soft_a2ff`, `soft_a2fb` | soft (kscale 0.03) | (0.6, 0.6) | 1e-5 s | 20 s | **Fig. 19**, `videos/fig19_soft/` |
| `soft_a1ff_first`, `soft_a1fb_first` | soft | (0.6, 0.6) | 1e-5 s | 20 s | `videos/fig19_soft_first/` |
| `soft_a1ff_second`, `soft_a1fb_second` | soft | (0.6, 0.6) | 1e-5 s | 20 s | `videos/fig19_soft_second/` |
| `stiff_tri_a1ff`, `stiff_tri_a1fb`, `stiff_tri_a2ff`, `stiff_tri_a2fb` | stiff | triangle, hold 5 s | 1e-5 s | 15 s | `videos/fig18_stiff_triangle/` |
| `soft_tri_a1ff`, `soft_tri_a1fb`, `soft_tri_a2ff`, `soft_tri_a2fb`, `soft_tri_a1ff_first`, `soft_tri_a1fb_first` | soft | triangle, hold 20 s | 3e-5 s | 60 s | `videos/fig19_soft_triangle/` |
| `soft_tri_a1ff_second`, `soft_tri_a1fb_second` | soft | triangle, hold 20 s | 3e-5 s | 60 s | `videos/fig19_soft_triangle_second/` |

Law suffixes: `a1`/`a2` = Algorithm 1/2, `ff`/`fb` = feedforward/feedback, `_first`/`_second` =
Algorithm 1 with the static branch to first/second order in gravity (see below). In every video
folder, `<law>.mp4` shows the robot and `x1_t.mp4`, `x2_t.mp4` the tip coordinates over time
(the time panels of `fig19_soft_first` also show the four laws of Fig. 19, those of the
`_second` folders all eight laws). 1 s of video = 1 s of simulated time.

### Common settings

* Robot (`src/pcc_params.m`): 5 CC segments, lengths 1/6 m (section 1, 3 segments) and 1/4 m
  (section 2, 2 segments); masses 1/3 and 1/2 kg; damping 20/3 and 10 N m s/rad; base bending
  stiffness 200/3 and 50 N m/rad ("k200100"); gravity 9.81 m/s² along -x₂ (downward in the plots;
  the straight robot points up); start at rest with
  each section bent by -π/6. The **stiff** robot is the base robot (kscale = 1), the **soft**
  robot has all stiffnesses multiplied by kscale = 0.03 ("k63"). Because
  G(q) = G_grav(q) + K q is linear in K, the stiffness is scaled at run time
  (`src/pcc_model.m`, `*_total.m`) without regenerating the model.
* CLIK gains: Alg. 1 Γ = 4.5 I (Jacobian transpose); Alg. 2 Γ_∥ = 3, Γ_⊥ = 10. The CLIK
  velocity is saturated to [-1, 1] componentwise. Low-level controller: τ = G_a(q̄) + 10 (q̄ₐ - qₐ).
* Integration: fixed-step explicit Euler for the robot and the CLIK state together (the
  original Simulink model used ode1 with the same step). For Alg. 1 the static branch output
  q̄ᵤ enters the controller with a one-step delay (memory block of the original model).
* Alg. 2 numerical escapes (unchanged from the original code): if Jₐ is near singular or
  ‖P_E‖ > 1e3, the step is replaced by 10 sin(sin(q̄))³ (bounded, avoids a stall); if H_uu is near
  singular or ‖P_Z‖ > 1e3, the statics component is replaced by 1e-3·(1,…,1). They are counted
  (`R.escapes`) and reported once, not printed at every step. They are
  triggered only by Alg. 2 (fb) on the soft robot (`soft_a2fb`: 60 490 task
  and 325 646 statics escapes out of 2e6 steps; `soft_tri_a2fb`: 20 073 and 119 693 out of 2e6 steps
  at the 3x larger step), never in the other cases. The original Simulink run of Fig. 19 shows the
  same behaviour and the result matches it (see Reproducibility).

### Algorithm 1 with gravity to first and second order

Write the unactuated statics as Gᵤ(qₐ, qᵤ) = E (qᵤ - qᵤ₀) + g N(qₐ, qᵤ), where
qᵤ₀ = φ₀(qₐ) is the zero-gravity (elastic) branch used in Figs. 18-19 (`qBarU`, linear in qₐ),
E = K_uu is the elastic stiffness of the unactuated coordinates (the elastic force vanishes on
qᵤ₀) and g N is the gravity force. Expanding qᵤ = qᵤ₀ + g q₁ + g² q₂ + … in Gᵤ = 0:

* first order: **φ₁(qₐ) = qᵤ₀ − E⁻¹ Gᵤ(qₐ, qᵤ₀)**
* second order (q₂ = −E⁻¹ ∂N/∂qᵤ(qᵤ₀) q₁, ∂N/∂qᵤ = H_uu − E):
  **φ₂(qₐ) = qᵤ₀ + (2 I − E⁻¹ H_uu(qₐ, qᵤ₀)) d₁,  d₁ = −E⁻¹ Gᵤ(qₐ, qᵤ₀)**

Both are closed form (no iterative solver); dφ/dqₐ is computed by central differences (step
1e-6); Gᵤ and H_uu are evaluated at the floored curvature (`src/clik_alg1.m`).

### Triangle with a sliding reference

Targets (0.6, 0.6) → (0.6, 0.4) → (0.75, 0.5), each active for one hold time (5 s stiff,
20 s soft). From the second target on, the reference does not jump: it slides linearly from
the previous target to the new one during the first half of the hold (2.5 s / 10 s), then stays
on the target (`tramp` in `src/pcc_sim.m`). The soft triangle uses dt = 3e-5 s to halve the run
time (on the soft single-target case, dt = 3e-5 s and 1e-5 s give tip positions within 0.05 mm
after 3 s for both algorithms; dt = 5e-5 s is numerically unstable). The single-target soft
cases use dt = 1e-5 s as in the paper.

### Known limitation: Alg. 2 (fb) on the soft robot

On the soft robot, Alg. 2 in feedback form diverges when the targets push q̄ across a **fold of
the statics** (a point of the static manifold where H_uu becomes singular, H_uu → 0: there the
projectors P_E and P_Z blow up and the numerical escapes take over). When testing candidate
triangles (same hold times and sliding reference), Alg. 2 (fb) reached the first target
(0.6, 0.6) and then collapsed (tip near (-0.27, -0.17)) on legs towards, e.g., (0.4, 0.5),
(0.55, 0.55), (0.5, 0.45) or (0.5, 0.55); a larger triangle (0.6, 0.6) → (0, 0.85) →
(0.35, 0.25) kept the statics-channel escape active for most of the run. The triangle above was
chosen in the region where all eight laws stay regular. On the same candidate triangles,
Algorithm 1 (all variants) and Alg. 2 (ff) did not diverge (Alg. 1 shows the steady-state
errors expected from its approximate statics).

## Run times

Measured with one MATLAB process per case (`-singleCompThread`, 10 cases in parallel on an
Apple-silicon laptop); a single process alone is somewhat faster.

| Cases | Steps | Time per case | ms/step |
|---|---|---|---|
| `stiff_a1*` (Fig. 18) | 5e5 | ~3 min | 0.4 |
| `stiff_a2*` (Fig. 18) | 5e5 | ~11 min | 1.4 |
| `stiff_tri_a1*` | 1.5e6 | ~10 min | 0.4 |
| `stiff_tri_a2*` | 1.5e6 | ~66 min | 2.6 |
| `soft_a1*`, `soft_tri_a1*` (gravity neglected) | 2e6 | ~14 min | 0.4 |
| `soft_a1*_first`, `soft_tri_a1*_first` | 2e6 | ~16-19 min | 0.5 |
| `soft_a1*_second`, `soft_tri_a1*_second` | 2e6 | ~75 min | 2.3 |
| `soft_a2*` (Fig. 19), `soft_tri_a2*` | 2e6 | ~83 min | 2.5 |

The Alg. 2 and second-order cases ran while ~12 MATLAB processes shared the machine; alone,
Alg. 2 runs at ~1-1.4 ms/step.

In total about 15.6 h of CPU time for the 24 cases, ~1 h 50 min wall-clock time with 10 processes
in parallel.

## Reproducibility

`pcc_sim` is a plain-MATLAB re-implementation of the original Simulink model of the paper
(`simulator.slx`, not shipped): the same functions, saturation, memory block and fixed-step Euler
(ode1). Running every case of this release (full horizons) and comparing with the saved runs:

* Figs. 18-19 against the original Simulink runs (tip position over the whole run): ≤ 0.005 mm
  for the six cases `stiff_*`, `soft_a1ff`, `soft_a1fb`; max 3.0 mm (final 0.04 mm) for
  `soft_a2ff` and max 1.6 mm (final 0.00 mm) for `soft_a2fb`.
* All other cases (gravity 1st/2nd order, triangles) and the plain-MATLAB re-runs of
  Figs. 18-19 against the results of the development code from which this release was
  extracted: bit-identical (maximum difference exactly 0 on q, q̄ and the time vector).

The shipped videos and the figures in `figures/` were rendered from the results of this
release.

## Model regeneration

`model/generate_pcc_model.m` derives M, C, G, D, A, the coordinate maps, the actuation-space
blocks of G and of its Hessian, the task Jacobians and the zero-gravity static branch
symbolically and writes them to `model/generated/` (or to a given folder). It is needed only if
the physical parameters in `src/pcc_params.m` (other than a stiffness scaling) are changed.
