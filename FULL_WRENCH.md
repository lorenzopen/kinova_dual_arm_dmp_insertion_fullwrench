# This copy: the full-wrench SOCP (2026-10-19)

2026-10-20 update: signed friction distance margins from `friction_cone.pdf`,
with the full nominal Coulomb opening. The former 70% cone and normal-axis
offset are replaced. Current equations, defaults and the 3D task-adaptive
extension are in [docs/FRICTION_MARGIN.md](docs/FRICTION_MARGIN.md).
The earlier offset experiment remains archived in
[docs/experiments/cone_shift_20261020/README.md](docs/experiments/cone_shift_20261020/README.md).
Vitus' original thesis uses uncontracted Coulomb cones plus a separate 1 N
normal floor per hand (pp. 80-81, eq. 5.23); this project's six point
point cones and full-wrench cost are an adaptation, not an identical program.

**Comandi per provarla sui Kinova:** [GUIDA.md](GUIDA.md)
(controller, ATI, prove a vuoto, `insert` scalare e `insert --twist`).

A copy of `kinova_dual_arm_dmp_insertion` (Desktop, commit `1d225f1`) with ONE change of
substance: the convex program. The original folder is untouched. Same python
package name: **do not put both folders in the same colcon `src/`**.

## What changed

| file | change |
|---|---|
| `kinova_dual_arm_dmp_insertion/socp.py` | rewritten: cost = human precision on the two FLANGE wrenches (Vitus' Sigma^-1), cones per point, no floor, no cap, no slack. Built once per set of active pads (DPP) |
| `controller.py` | builds the new SOCP from `cell.yaml`; `full_wrench` regulates the internal flange wrench on every route |
| `main.py` | passes `socp`, `full_wrench`, `full_wrench_channels` from the config; `insert` refuses `full_wrench: true` unless `allow_hand_wrench: true` |
| `config/cell.yaml` | `socp.margin_mode: task_adaptive`, scale 0.25, task floor 0.5 N; `w_force: 0`, `no_bending: false`, `full_wrench: false` |

The program, in short (`w = H f` = the two flange wrenches, 12):

    min_f  1/2 w' D' Sigma^-1 D w  (+ w_force/(m g)^2 |f|^2, off by default)
    s.t.   sum f_i + m g = 0,  sum (p_i - c) x f_i = 0
           |(I - n n') f_i| <= mu (-n' f_i) - rho_i sqrt(1+mu^2)   per pad
           -n' f_i >= 0

The 18 pad forces are only the exact friction model of the tri-pad (a point has
no moment); what is commanded is the flange wrench.

`full_wrench_channels: twist` = squeeze (the existing `ds` loop) + the OPPOSITE
TWIST about the pad axis, nothing else - in simulation the channels a
tilt-compliant tool can carry. `internal` = the old hwd law (internal
tangential + all internal moments).

## Historical checks before the signed-distance update (no robot)

- SOCP time per call: **1.4 ms** median (1.9 max) after the first call (~20 ms,
  compilation). The old Kinova SOCP: 1.7 ms. Codex's non-DPP version: 22 ms - on
  the robot PC that was the 100 Hz -> 7 Hz defect of 2026-10-16.
- Dry runs, `check`, on the two routes inserted on the robot on 2026-10-16:

| route | arm | SOCP ok | squeeze median asked | seat found |
|---|---|---|---|---|
| T F75_Tt1_s9007, c100_five_cut800 | original | 104/104 | 4.7 N | yes |
| | new, scalar squeeze | 104/104 | **2.9 N** | yes |
| | new, `full_wrench: true` | 105/105 | 2.9 N | yes |
| Z F75_Zt1_s9012, c100_five | original | 111/111 | 5.2 N | yes |
| | new, scalar squeeze | 111/111 | **2.0 N** | yes |
| | new, `full_wrench: true` | 112/112 | 2.0 N | yes |

  With `full_wrench: true` the left offset moved only in rotation (opposite
  twist, max 1.67 deg) and 0.00 mm in translation. The dry-run backend returns
  zero moments, so this checks the wiring, not the tracking.
  (The 43 mm "route phase vs PyBullet" is the same in the original: not this change.)

## Before the robot - read this

1. **The squeeze asked drops to 2-3 N** on the 0.145 kg cubes (was 4.7-5.2). The
   controller still floors its target at 1 N, and the pickup squeeze is still
   6 N. The jam sentry's limit scales with the squeeze (`2 mu F - m g`): expect
   it to be tighter.
2. **The wrench source.** Until 2026-10-20 it was ESTIMATED from the joint torques,
   off by up to 8-9 N per arm: a few newtons of internal wrench sat inside that error.
   **2026-10-21: the wrist ATIs are mounted** (`wrench_source: ft`, ros_backend.py,
   checked with `ft_check`); `insert --twist` turns the full wrench on and refuses
   to run unless both arms read their ATI. The joint-torque estimate is still logged.
3. The routes were trained with the old squeeze law.
