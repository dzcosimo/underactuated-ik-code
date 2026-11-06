# Code release: kinematic inversion for underactuated (soft) robots

MATLAB code for the simulations of

> C. Della Santina, *Pushing the boundaries of actuators-to-task kinematic inversion: from fully
> actuated to underactuated (soft) robots*, The International Journal of Robotics Research, in press.

Project page, with explainer videos, interactive demos and the simulation videos:
https://tud-phi.github.io/underactuated-ik/

> **About this release.** This code release is a Claude-generated version (Anthropic's Claude,
> used as a coding assistant) of a code base originally written entirely by the author,
> Cosimo Della Santina. Claude ported, reorganized, documented and packaged the original code.

The paper proposes two closed-loop inverse kinematics (CLIK) algorithms for underactuated robots.
Both compute a reference configuration that a low-level controller tracks, so that a task variable
reaches a target:

* **Algorithm 1** inverts the end-to-end map from actuated coordinates to task through an
  explicit parametrization of the robot's statics, qu = φ(qa).
* **Algorithm 2** needs no explicit statics. It moves along the manifold of attainable equilibria
  (Gu = 0) with a task component (projector P_E) and a statics-relaxation component (projector P_Z).

Each algorithm can run in **feedforward** (task error computed on the CLIK state) or
**feedback** form (task error from the measured robot).

## Contents

| Folder | System | What it reproduces |
|---|---|---|
| [`double_pendulum/`](double_pendulum/README.md) | tendon-driven double pendulum (n = 2, one input) | Figs. 7, 10-13, 15-16: Alg. 1 and Alg. 2 examples, vector fields, Alg. 2 as a feedback loop |
| [`pcc/`](pcc/README.md) | planar soft robot, 5 constant-curvature segments, 2 inputs | Figs. 18-19 (stiff and soft robot, four CLIK variants), plus: Alg. 1 with gravity approximated to first and second order, and three targets in sequence (triangle) |
| [`ballerina/`](ballerina/README.md) | planar "ballerina" on an elastic pointe, 3 actuated joints, CoM task | the balancing robot of Fig. 1(b) in simplified form; redundant case (more actuators than task variables) handled with the pseudoinverse |

Each folder is **self-contained**: its own README, entry-point scripts, source code, model
functions. The folders do not depend on each other. To keep the repository small, it leaves out
the rendered videos, figures and result files that the READMEs mention: the scripts recreate them,
and the videos are on the project page.

## Requirements

* MATLAB, tested with **R2025a**. The figure and video scripts use `theme`, so they need R2025a or newer.
* No toolbox is needed to run the simulations, figures and videos. The symbolic model-generation
  scripts (optional: the generated functions are included) need the Symbolic Math Toolbox.
* No Simulink: all simulations are plain MATLAB. The original Simulink implementations of the
  double-pendulum and PCC examples were ported and checked against them (see each README).

## Quick start

Work in one folder at a time: start MATLAB in that folder, or `cd` into it. Do not put two folders
on the path together, because they share function names (e.g. `clik_alg1`, `setup_paths`).

```matlab
cd double_pendulum;  run_all_double_pendulum   % about 2 min; writes figures/
cd ../ballerina;     run_all_ballerina         % about 10 s; writes results/ and prints the tables
cd ../pcc;           run_all_pcc               % hours (see pcc/README.md); or run_pcc_case('<case>')
```

Videos are re-rendered with `make_videos_pcc` and `make_videos_ballerina` (see the READMEs).
The rendered videos are on the project page. All videos run in
real time: 1 s of video is 1 s of simulated time.

## Citation and license

If you use this code, please cite the paper:

```bibtex
@article{dellasantina2026underactuated,
  author  = {Della Santina, Cosimo},
  title   = {Pushing the boundaries of actuators-to-task kinematic inversion: from fully actuated to underactuated (soft) robots},
  journal = {The International Journal of Robotics Research},
  year    = {2026},
  note    = {In press}
}
```

License: MIT, see [LICENSE](LICENSE).
