# Elastic-pointe ballerina: CLIK for a redundant underactuated robot

Code for the "ballerina" example (Fig. 1b) of the IJRR paper on closed-loop inverse kinematics
(CLIK) for underactuated robots (C. Della Santina et al.). A planar dancer stands on a passive
elastic pointe and moves her centre of mass (CoM) with three actuated joints. The task has
dimension 2 and there are 3 actuators, so the task is redundant. Algorithms 1 and 2 of the paper
are used with the Moore-Penrose pseudoinverse, in feedforward (ff) and feedback (fb) form.

The code is plain MATLAB (no Simulink, no toolbox needed to run). The model was generated
symbolically. The simulations use fixed-step RK4. Tested with MATLAB R2025a.

## Contents

```
ballerina/
  README.md                this file
  run_all_ballerina.m      entry point: runs the 7 simulations -> results/, prints the tables
  summarize_results.m      results tables (Markdown) from results/*.mat
  make_videos_ballerina.m  renders the videos from results/*.mat
  src/
    bal_params.m           physical parameters and low-level gains
    bal_scenario.m         definition of the 7 runs (start, targets, gains, stiffnesses)
    bal_run_case.m         runs one scenario, saves results/<name>.mat
    bal_sim.m              simulation: plant (RK4) + low-level controller + CLIK (Euler)
    bal_clik.m             CLIK update, Algorithm 1 and Algorithm 2 (non-square)
    bal_phi.m              statics map qu = phi(qa) of Algorithm 1 (Newton)
    bal_sat.m              velocity saturation of the CLIK update
    bal_margin.m           stability margins H_uu and the closed-loop Schur complement
  model/
    gen_symbolic.m         symbolic derivation (Symbolic Math Toolbox) -> the files below
    bal_M, bal_c, bal_G, bal_H, bal_com, bal_Jcom, bal_points   generated model functions
  videos/                  rendered videos of the released runs (main/, mismatch/, soft/)
  results/                 created by run_all_ballerina (not shipped)
```

## System

The model is planar, with z up and g = 9.81 m/s^2. The bodies are uniform rods plus a point-mass head:

| body | from | length [m] | mass [kg] |
|---|---|---|---|
| stance leg | pointe (origin) to hip | 0.9 | 3 |
| torso | hip to shoulder | 0.6 | 5 |
| working leg | hip to foot | 0.9 | 3 |
| arm | shoulder to hand | 0.55 | 1.2 |
| head | point mass at the centre of a circle of radius 0.1 on top of the torso | | 1 |

The total mass is M = 13.2 kg.

The configuration is q = (qa1, qa2, qa3, qu). All angles are positive clockwise, and a segment
at absolute angle th points along e(th) = [sin th; cos th].
- qa1 is the torso angle relative to the stance leg (hip, actuated).
- qa2 is the working leg relative to the stance leg (hip, actuated). The legs are together at qa2 = 0.
- qa3 is the arm relative to the torso (shoulder, actuated). The arm lies along the torso at qa3 = 0.
- qu is the absolute angle of the stance leg at the pointe, measured from vertical. It is
  **passive**, with a torsional spring k and a damper b.

The actuation matrix is A = [I3; 0], so the qa are already actuation coordinates. The dynamics is

    M(q) q'' + c(q, q') + G(q) = [tau; -b qu'],    U = g sum_i m_i z_i + k qu^2 / 2,    G = dU/dq,   H = d2U/dq2.

The task is the CoM, x = h(q) = (x_c, z_c), with Jacobian [Ja Ju] (2x3 and 2x1). Two scalar
identities drive everything below:
- **G_u = k qu - M g x_c.** The statics G_u = 0 gives qu = M g x_c / k, so the horizontal CoM
  target fixes the lean of the pointe.
- **H_uu = k - M g z_c.** The stability of the pointe statics depends only on the CoM height.

Parameters (`src/bal_params.m`):
- k = 200 Nm/rad. k_soft = 120 Nm/rad is used only in the soft scenario.
- b = 50 Nms/rad, so the damping ratio of the sway is about 0.8.
- The low-level controller (the paper's) is `tau = G_a(q_bar) + P (qa_bar - qa) - D qa'`, with
  P = 300 I3 Nm/rad and D = 30 I3 Nms/rad.

## Why the closed loop is a physical system, and the stability condition

With q_bar held fixed, the low-level law adds three terms: a constant torque G_a(q_bar), a joint
spring P toward qa_bar, and a joint damper D. The closed loop is therefore again a mechanical
system. Its potential is

    U(q) + (qa_bar - qa)' P (qa_bar - qa) / 2 - G_a(q_bar)' qa

and its dissipation comes from D and b. If q_bar is an equilibrium of the model (G_u(q_bar) = 0),
then q = q_bar is an equilibrium of this closed loop. It is stable when the Hessian
H + blkdiag(P, 0) is positive definite, i.e. (H_aa + P > 0) when the Schur complement is positive:

    H_uu - H_ua (H_aa + P)^-1 H_au > 0.

As P grows this tends to the open-loop condition on the passive joint:

    H_uu = k - M g z_c > 0     <=>     k / (M g z_c) > 1.

Both margins are computed along every run (`src/bal_margin.m`, Table 1):
- Along all main runs, H_uu >= 70.5 Nm/rad, i.e. k/(M g z_c) >= 1.54. The Schur margin is >= 67.2.
- The highest point is the centred target T2: z_c = 1.00, M g z_c = 129.5.
- With k_soft = 120, T1 is stable (H_uu = 8.6) and T2 is not (H_uu = -9.5). This is the soft scenario.

## Algorithms (`src/bal_clik.m`)

All quantities are evaluated at the CLIK state q_bar with the model stiffness. Define:
- X = H_uu^-1 H_ua (1x3). Along the equilibrium manifold, dqu/dqa = -X.
- J = Ja - Ju X (2x3), the task Jacobian along the manifold.
- J^+ = J'(J J')^-1, the plain Moore-Penrose pseudoinverse (no damping).

**Algorithm 1.** The state is qa_bar. The statics is solved exactly, qu_bar = phi(qa_bar), by
scalar Newton on G_u with a warm start (`bal_phi.m`). The update is

    qa_bar' = J^+ Gamma e.

**Algorithm 2 (non-square generalization).** The state is q_bar in R^4, free to leave the
equilibrium manifold. Define T = [I3; -X] and Y = Ja^+ Ju. The update is

    q_bar' = T J^+ Gamma e  -  [-Y; 1] (H_uu - H_ua Y)^-1 Gamma_perp G_u(q_bar).

The two terms do separate jobs:
- **Task term.** [Ja Ju] T = J, so the task moves as Gamma e. Also [H_ua H_uu] T = 0, so the
  term moves along the tangent of the manifold and does not change G_u to first order.
- **Statics term.** [Ja Ju] [-Y; 1] = 0, so it does not move the task to first order (Ja has
  full row rank). Also [H_ua H_uu] [-Y; 1] = H_uu - H_ua Y, so G_u' = -Gamma_perp G_u and q_bar
  converges to the manifold.

With a square, invertible Ja (m = number of actuators), J^+ = J^-1 and Ja^+ = Ja^-1. The formula
then reduces to the paper's form, with the P_E Ja^-1 and P_Z H_uu^-1 terms. During development
this was checked numerically on 200 random square cases, with a relative difference of 3e-14
(the check script is not part of this release). In this release Algorithm 2 starts at
qu_bar = phi(qa0) + 0.15 rad, off the manifold.

**ff / fb.**
- ff: e = x_ref - h(q_bar).
- fb: e = x_ref - h(q), with q the measured plant configuration.

In both variants J^+, T and the statics term are evaluated at q_bar. Only the error uses the
measurement. The variant of the paper's feedback scheme that evaluates the mapping at the
measured q is not used.

**Gains.**
- Gamma = 3.0 (ff) and 1.0 (fb), Gamma_perp = 3.
- fb is gain-limited by the slow pointe sway. Gamma = 1.2-1.5 leaves 8-15 mm of oscillation at
  some hold ends, and Gamma = 0.7 is too slow. ff is not limited in this way.

**Saturation.** The CLIK update is clipped component-wise to +-1 rad/s (`bal_sat.m`).
`'satmode', 'scale'` gives a direction-preserving version. It changes the hold-end errors by
less than 1.5 mm.

**Rank of J.** J stays full rank along all released runs:
- sigma_min(J) >= 0.053 on the main runs, 0.037 on the mismatch runs and 0.033 on the soft run.
  It is logged per sample in R.svmin.
- Where sigma_min is small, J^+ has a gain of up to about 20, and the saturation bounds the
  update. The saturation is active for 2.7-5.9 % of each main run.
- MATLAB's nearlySingularMatrix warning is silenced only inside `bal_run_case`, and the state is
  restored afterwards.

## Simulation (`src/bal_sim.m`)

- **Plant.** RK4 with dt = 1 ms. q_bar is held over each step, and the torque law is evaluated
  continuously in the plant state.
- **CLIK.** Explicit Euler at the same dt. The log interval is 1 ms.
- **Step size.** Halving dt changed the CoM by at most 0.13 mm over a whole run, and by at most
  0.012 mm at the end of each hold (checked during development).
- **Start.** The plant starts at rest at the equilibrium phi(qa0) of the plant stiffness. The
  CLIK state starts at the model equilibrium.

## Scenarios (`src/bal_scenario.m`)

**Start.** qa0 = (0.60, 1.15, -2.65), qu = 0.09 rad, CoM at (0.139, 0.941):
- the torso is 40 deg forward;
- the working leg is back (foot at z = 0.60);
- the arm is raised forward-up.

**Targets (CoM, m).** The poses described are those reached by main_a2ff:
- **T1 (0.35, 0.86): arabesque penchée.** The torso is about horizontal, the working leg is
  raised back (foot at z = 1.50), the arm reaches forward and the pointe leans +13 deg.
- **T2 (0.00, 1.00): centred and raised.** A low arabesque with the arm up and no lean.
- **T3 (-0.35, 0.97): cambré to the other side.** The torso is 20 deg back, the arm is
  overhead, the working leg stays back and the pointe leans -13 deg.

**Timing.** Each target is held for 6 s. Within each hold, the reference slides linearly to the
new target over the first 3 s. The first target is a step. The total is 18 s.

**Main runs.** Along all of them the working foot stays above z = 0.468 m, and every other point
above 0.56 m.

| run | plant k | model / low-level k | targets | duration |
|---|---|---|---|---|
| main_a1ff, main_a1fb, main_a2ff, main_a2fb | 200 | 200 | T1, T2, T3 | 18 s |
| mismatch_a2ff, mismatch_a2fb | 200 | 160 (0.8 k) | T1, T2, T3 | 18 s |
| soft_a2ff | 120 | 120 | T1 (start), T2 | 15 s |

**Soft run.** At k_soft = 120 the main start is unstable, because M g z_c = 121.9 > 120. No
upright pose with a low enough CoM keeps the foot off the floor. The run therefore starts at rest
at the stable T1 equilibrium of k_soft:
- qa0 = (0.120, 0.434, -1.848) and qu = 0.378 rad (21.6 deg), with H_uu = 8.6;
- this pose comes from a continuation of the Algorithm 1 flow at T1, from k = 200 down to 120;
- the reference then slides to T2, where the model has H_uu = -9.5.

A run would stop if |qu| > 1.2 rad.

Each `results/<name>.mat` contains:
- t, q, q_bar, x (plant CoM), x_ref, targets, params and cpu_time;
- R, the full log of `bal_sim`: dq, tau, x_bar = h(q_bar), Huu (plant), Huu_bar, Gu_bar, sat,
  svmin, err = |x_ref - x|, err_end_hold, opts.

## Results

The tables below are the output of `summarize_results`.
- "Below 5 mm from" is the time after which the error stays below 5 mm until the end of that
  hold. Holds end at 6, 12 and 18 s.
- "--" means the error is not below 5 mm at the end of the hold.
- "0.00" (soft, hold 1) means the error is below 5 mm for the whole hold.

Table 1:

| run | Gamma | CoM err at end of hold 1 / 2 / 3 [mm] | final err [mm] | below 5 mm from [s], per hold | min H_uu | min Schur | min k/(M g z_c) | max abs(qu) [rad] | min sigma(J) | sat. frac. |
|---|---|---|---|---|---|---|---|---|---|---|
| main_a1ff | 3.0 | 0.03 / 1.03 / 1.34 | 1.34 | 2.77 / 10.23 / 16.27 | 70.5 | 67.4 | 1.54 | 0.244 | 0.060 | 0.037 |
| main_a1fb | 1.0 | 0.53 / 2.03 / 0.94 | 0.94 | 4.34 / 11.66 / 17.70 | 70.6 | 67.3 | 1.55 | 0.262 | 0.058 | 0.027 |
| main_a2ff | 3.0 | 0.03 / 1.03 / 1.35 | 1.35 | 2.78 / 10.23 / 16.27 | 70.5 | 67.4 | 1.54 | 0.245 | 0.062 | 0.037 |
| main_a2fb | 1.0 | 0.95 / 2.19 / 0.94 | 0.94 | 5.41 / 11.65 / 17.70 | 70.6 | 67.2 | 1.55 | 0.290 | 0.053 | 0.059 |
| mismatch_a2ff | 3.0 | 124.31 / 0.58 / 176.95 | 176.95 | -- / 10.23 / -- | 68.5 | 64.7 | 1.52 | 0.162 | 0.044 | 0.000 |
| mismatch_a2fb | 1.0 | 1.22 / 12.59 / 27.12 | 27.12 | 4.40 / -- / -- | 67.2 | 63.4 | 1.51 | 0.273 | 0.037 | 0.050 |
| soft_a2ff | 3.0 | 2.14 / 536.61 | 649.42 | 0.00 / -- | 1.1 | -0.5 | 1.01 | 0.652 | 0.033 | 0.000 |

Table 2:

| run | abs(x_ref - h(q_bar)) at hold ends [mm] | abs(h(q_bar) - x) at hold ends [mm] | min z foot [m] | min z other [m] | arm elevation at hold ends [deg] |
|---|---|---|---|---|---|
| main_a1ff | 0.00 / 0.01 / 0.00 | 0.03 / 1.03 / 1.35 | 0.479 | 0.779 | -13 / 47 / 89 |
| main_a1fb | 1.23 / 3.03 / 6.51 | 1.76 / 4.56 / 5.66 | 0.484 | 0.719 | -15 / 43 / 86 |
| main_a2ff | 0.00 / 0.01 / 0.00 | 0.03 / 1.03 / 1.35 | 0.477 | 0.786 | -12 / 48 / 88 |
| main_a2fb | 1.51 / 2.76 / 6.55 | 0.56 / 4.46 / 5.71 | 0.468 | 0.560 | -6 / 57 / 79 |
| mismatch_a2ff | 0.00 / 0.01 / 0.00 | 124.31 / 0.59 / 176.94 | 0.438 | 0.888 | 5 / 39 / 62 |
| mismatch_a2fb | 159.48 / 14.22 / 282.62 | 158.26 / 1.75 / 257.25 | 0.428 | 0.685 | -10 / 46 / 85 |
| soft_a2ff | 0.00 / 0.01 | 2.14 / 536.61 | 0.189 | 0.715 | -13 / -13 |

**Main runs.**
- All four laws are below 2.2 mm at the end of every hold.
- The ff laws get below 5 mm 1.7-3.2 s before each hold ends. The fb laws do so only 0.3-0.6 s
  before the end, because they are limited by the slow pointe sway.
- The two algorithms reach almost the same poses.

**Mismatch, ff.** Feedforward converges to the equilibrium of the wrong model
(|x_ref - h(q_bar)| = 0.01 mm). A bias of 12-18 cm remains in the plant: the real, stiffer pointe
leans less (max 0.16 rad instead of 0.23). At T2 (x_c = 0, so qu = 0) the spring plays no role
and the bias vanishes.

**Mismatch, fb.** Feedback removes the bias, but more slowly than in the nominal case, because
the model lean is about twice the real one.
- The error is 1.2 mm at the end of hold 1.
- At the ends of holds 2 and 3 the run is still converging (12.6 and 27.1 mm, with a slight
  overshoot).
- During development, holding T3 longer gave 3.0 / 1.5 / 0.35 mm at t = 20 / 24 / 26 s, while
  ff stays at 178 mm.

**Soft (no fall, see caveats).** The CLIK reference does what it should: h(q_bar) reaches T2
(0.01 mm), so q_bar converges to the attainable but unstable model equilibrium (H_uu = -9.5).
There is no saturation and no singularity. The plant cannot hold that equilibrium. It stays
leaning to the old side and drifts along the buckled stable branch of the spring-loaded
pendulum: qu grows from 0.38 to 0.65 rad, and the error is 537 mm at 12 s and 649 mm at 15 s.
The whole error is physical, |h(q_bar) - x|.

## Honest caveats

- **The elastic pointe alone keeps the dancer upright.** With k = 200 > M g z_c the passive
  joint is stable by itself: with all joints locked, the dancer would not fall. The example
  therefore shows the CLIK of an underactuated, redundant, elastic system. It does not show
  balance recovery.
- **The soft scenario shows drift on the buckled branch, not a fall.** A fall past |qu| = 1.2 rad
  would need k/(M g z_c) < sin(1.2)/1.2 = 0.78, and then the start pose itself would already be
  unstable. The run is reported as it is, without forcing a fall.
- **ff and fb use different gains (Gamma = 3 vs 1).** fb is gain-limited by the slow sway, so the
  ff/fb error differences in the tables include the gain difference. With the fb gain, ff ends
  holds 2 and 3 at 9.25 and 8.25 mm instead of 1.03 and 1.35 mm, i.e. it misses 5 mm. At equal
  gain, fb is the more accurate of the two. To reproduce:
  `bal_run_case('main_a2ff', 'Gam', 1.0, 'outname', 'main_a2ff_gam1')`.
- **T3 is a cambré, not a mirrored penchée.** The redundant pseudoinverse flow cannot cross
  directly from one side to the mirrored side. In kinematic screening, 0 of 240 mirrored
  start/target triples converged. The flow is drawn into the singular "collinear" pose (torso,
  working leg and arm aligned) and stalls there. Passing through the raised centred target T2
  avoids this. For the same reason the third pose is a cambré on the other side, not the mirror
  image of the penchée T1.
- **The start and targets were tuned.** They come from a kinematic screening with these criteria:
  convergence of both algorithms, foot off the floor, arm elevation, sigma_min(J) and the H_uu
  margin. They are not generic.

## How to run

In MATLAB, from any folder:

```matlab
run('ballerina/run_all_ballerina.m')    % 7 simulations -> ballerina/results/, prints Tables 1-2
```

Other useful calls (after `addpath ballerina ballerina/src ballerina/model`):

```matlab
summarize_results                                   % tables only, from results/*.mat
R = bal_run_case('main_a2fb');                      % one run
R = bal_run_case('main_a2fb', 'satmode', 'scale', 'outname', 'test');   % override options
make_videos_ballerina                               % all videos -> videos/ (overwrites the shipped ones)
make_videos_ballerina('sets', {'soft'}, 'nmax', 60, 'outdir', tempdir)  % quick check
```

`model/gen_symbolic.m` rebuilds the generated model (Symbolic Math Toolbox). It is needed only if
`src/bal_params.m` changes.

**Run times** (Apple M4 Pro, R2025a):
- Each simulation loop takes about 0.5 s; the CPU column is printed by `summarize_results`.
- `run_all_ballerina` takes about 5 s in total (about 10 s including the MATLAB start-up with `matlab -batch`).
- The videos take about 0.15 s per frame: about 15 min for all 10 videos (5300 frames).

**Reproducibility.** The runs of this release are bit-identical to the original research code:
the maximum difference of the CoM trajectory is 0 m on all 7 runs.

## Videos (`videos/`)

Real time, 30 fps, MPEG-4. The robot panels are 720x720 and the time panels 960x480.

| file | content |
|---|---|
| main/a1ff, a1fb, a2ff, a2fb.mp4 | the dancer, CoM trace, targets (active one bold), sliding reference, ghost poses |
| main/xc_t.mp4, zc_t.mp4 | x_c(t) and z_c(t) of the four laws and the reference |
| mismatch/a2ff, a2fb.mp4 | as above, plus the model CoM h(q_bar) (hollow marker) |
| mismatch/err_t.mp4 | abs(x_ref - x)(t), log scale, with the 5 mm line |
| soft/a2ff.mp4 | soft pointe: the reference reaches T2, the plant drifts on the buckled branch |
