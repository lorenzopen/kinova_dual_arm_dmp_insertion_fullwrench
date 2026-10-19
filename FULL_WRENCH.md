# This copy: the full-wrench SOCP (2026-10-19)

**Comandi per provarla sui Kinova:** [docs/AVVIO_FULL_WRENCH.md](docs/AVVIO_FULL_WRENCH.md)
(controller, prove a vuoto, baseline per pezzo, nuovo SOCP e profilo twist).

A copy of `kinova_dual_arm_dmp_insertion` (Desktop, commit `1d225f1`) with ONE change of
substance: the convex program. The original folder is untouched. Same python
package name: **do not put both folders in the same colcon `src/`**.

## What changed

| file | change |
|---|---|
| `kinova_dual_arm_dmp_insertion/socp.py` | rewritten: cost = human precision on the two FLANGE wrenches (Vitus' Sigma^-1), cones per point, no floor, no cap, no slack. Built once per set of active pads (DPP) |
| `controller.py` | builds the new SOCP from `cell.yaml`; `full_wrench` regulates the internal flange wrench on every route |
| `main.py` | passes `socp`, `full_wrench`, `full_wrench_channels` from the config; `insert` refuses `full_wrench: true` unless `allow_hand_wrench: true` |
| `config/cell.yaml` | new block `socp:` (`r_s` 0.30, `w_force` 0, `no_bending` false), `full_wrench: false`, `full_wrench_channels: twist` |

The program, in short (`w = H f` = the two flange wrenches, 12):

    min_f  1/2 w' D' Sigma^-1 D w  (+ w_force/(m g)^2 |f|^2, off by default)
    s.t.   sum f_i + m g = 0,  sum (p_i - c) x f_i = 0
           |(I - n n') f_i| <= (1 - r_s) mu (-n' f_i)   per pad

The 18 pad forces are only the exact friction model of the tri-pad (a point has
no moment); what is commanded is the flange wrench.

`full_wrench_channels: twist` = squeeze (the existing `ds` loop) + the OPPOSITE
TWIST about the pad axis, nothing else - in simulation the channels a
tilt-compliant tool can carry. `internal` = the old hwd law (internal
tangential + all internal moments).

## Checked on this PC (no robot)

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
2. **The wrench is ESTIMATED from the joint torques** (`wrench_source: estimated`),
   off by up to 8-9 N per arm (cell.yaml). Regulating a few newtons of internal
   wrench sits inside that error. Turn `full_wrench` on only with an F/T, or after
   checking the moment estimate.
3. The routes were trained with the old squeeze law.
