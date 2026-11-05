# Underactuated inverse kinematics: MATLAB code

Code for the simulations of the paper

> C. Della Santina, *Pushing the boundaries of actuators-to-task kinematic inversion: from fully actuated to underactuated (soft) robots*, The International Journal of Robotics Research, in press.

Project page with explainer videos, interactive demos and the simulation videos: https://dzcosimo.github.io/underactuated-ik/

> **About this release.** This repository is a Claude-generated version (Anthropic's Claude, used as a coding assistant) of a code base originally written entirely by the author, Cosimo Della Santina. Claude reorganized, documented and repackaged the original code. The methods and the original implementation are the author's.

## Contents

| Folder | System | What it shows |
|---|---|---|
| [`pcc/`](pcc) | Planar soft arm, piecewise constant curvature: 5 curvature segments, 2 actuated sections, total length 1 m | Algorithms 1 and 2, feedforward and feedback, on a stiff and a soft arm (paper, Figs. 18 and 19), plus a three-target sequence and Algorithm 1 with gravity in its statics |
| [`ballerina/`](ballerina) | Planar dancer on an elastic pointe: 3 actuated joints, 1 passive spring joint, task = centre of mass | Both algorithms on a redundant task (3 actuators, 2D task), with model mismatch and with an unstable target equilibrium |

Both folders follow the same pattern: `src/` holds the algorithms and the simulation, `model/` the dynamic model (generated functions are included, so no Symbolic Math Toolbox is needed to run).

Naming of the variants, used everywhere:

| Code | Meaning |
|---|---|
| `a1ff`, `a1fb` | Algorithm 1 (statics solved beforehand), feedforward / feedback |
| `a2ff`, `a2fb` | Algorithm 2 (statics solved online), feedforward / feedback |

Feedforward: the task error is computed on the CLIK state, `e = x_bar - h(q_bar)`. Feedback: on the measured robot, `e = x_bar - h(q)`. In both cases a low-level controller `tau = G_a(q_bar) + P (qa_bar - qa) (- D qa')` tracks the CLIK output.

## Quick start

```matlab
% soft arm, one case (a few minutes for a stiff case, 0.5-2 h for a soft one)
cd pcc
R = run_pcc_case('stiff_a2fb');      % list all cases with {pcc_cases().name}

% ballerina, all seven scenarios (about 0.5 s each) and the results tables
cd ../ballerina
run_all_ballerina
make_videos_ballerina                % videos, needs MATLAB R2025a or later
```

`pcc/run_all_pcc.m` runs every soft-arm case (15-20 h sequentially; see the file for running them in parallel, and `SMOKE = true` for a 1-3 min installation check).

## Requirements

- MATLAB. Video rendering of the ballerina uses `theme()` (R2025a or later).
- Optional: Symbolic Math Toolbox, only to regenerate `ballerina/model/` after changing `bal_params.m`; Parallel Computing Toolbox, only for `USE_PARFOR` in `run_all_pcc.m`.

## Notes

- `pcc/model/generated/` was generated with the parameters in `pcc/src/pcc_params.m`. The generator script (`model/generate_pcc_model.m`, mentioned in the comments) is not part of this release. Stiffness can still be changed at run time through `kscale` (see `pcc/src/pcc_model.m`).
- Results (`results/*.mat`) and videos are not stored in the repository; the scripts recreate them.

## Citation

```bibtex
@article{dellasantina2026underactuated,
  author  = {Della Santina, Cosimo},
  title   = {Pushing the boundaries of actuators-to-task kinematic inversion: from fully actuated to underactuated (soft) robots},
  journal = {The International Journal of Robotics Research},
  year    = {2026},
  note    = {In press}
}
```

## License

MIT, see [LICENSE](LICENSE).
