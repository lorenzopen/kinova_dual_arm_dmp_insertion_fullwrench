# Per-pad friction and passive-tool transfer: frozen design

The two source PDFs are identified by SHA-256 in papers/sources.json. This is
an adaptation to six discrete sphere contacts, not a reproduction of either
paper's apparatus or published success rates.

## Questions and source correspondence

1. Friction-cone paper eqs. 3-11, 23-27, Figs. 5/9/10: compare planned and
   realized contact forces, signed distance, friction utilization, unloading;
   compare no margin, fixed margin and task-adaptive margin. Fixed margin is
   calibrated using optimizer profiles only to match adaptive mean combined
   hand force, before any physical result is inspected.
2. Wong eqs. 41-48, Fig. 11: compare tangential-force/torsion coupling and
   planned versus realized wrist wrenches under centered, planar-offset and
   spatial-offset CoM. Its aggregate ellipsoid is a named comparison with
   Reff=0.0328 m and rs=0.10, not a new local moment on a sphere. The physical
   model always retains six point contacts with zero spinning/rolling friction.
3. Compare the same rigid plate welded to its flange with the original
   floating-plate springs. Geometry, mass, material and wrist coupling stay
   identical. Record discrepancies instead of replacing measured forces with
   the optimizer witness.

## Information regime and boundaries

This is a known-load mechanics bench, explicitly an oracle-load condition:
prescribed mass/CoM and nominal geometry are supplied to the planner. It does
not validate online inertial estimation, blind geometry acquisition, placement,
the paper's environmental pivot, full Gen3 dynamics, or the Kinova insertion
controller. Actual body pose is used only for retrospective scoring and plant
construction. The command uses the prescribed reference and planned wrenches.
Per-pad simulation contact measurements are used for diagnostics, never future
forces or direct independent pad commands.

## Conditions fixed before outcomes

- Box 120 mm, mass 1.9 kg; nominal pad geometry d=20 mm, r=30 mm, sphere R=10 mm.
- CoM cases: [0,0,0], [15,0,0] mm, [15,15,0] mm, relative to box geometric centre.
- Three prescribed initial pose perturbations: zero, +1 degree about y,
  -1 degree about y. These are deterministic paired cases, not independent
  experimental repeats and not a population estimate.
- Main mechanisms: welded tripod and floating plate. Floating defaults:
  500 N/m peripheral axes; central 20000 N/m translation, 40/0/0 Nm/rad rotation.
  An explicitly separate adaptive-only sensitivity uses central axial 2700 N/m.
- Effective physical friction 0.75 = object 1.0 * sphere 0.75; read back values.
  Plate collision is off; only the six sphere links may carry the box.
- Wrist proxy: translation 1500 N/m and 15 Ns/m, rotation 40 Nm/rad and
  1.15 Nms/rad, same for every mechanism and planner. Pose bias K^-1*w is
  the common paper-style realization. No per-pad force actuation, no outer
  force tuning, no new friction-reserve supervisor.
- Current precision-only TP-GMM/SOCP is unchanged for the three point-cone
  arms: session C, no human-mean target, no effort/identity addition, hard
  equilibrium, no prescribed compression cap/floor, no bending restriction.
  Adaptive task margin = 0.25*||Ftask|| + 1 N (application default; paper used
  0.5 N). Fixed comparison has one calibrated per-pad distance. The Wong
  comparison uses its distinct minimum weighted-wrench cost and zero bending
  at the aggregate contact pole; hence it compares a package of choices.
- Core: 2 mechanisms * 4 planners * 3 CoM cases * 3 poses = 72 cases.
- Adaptive-only stress: light known 1.7 kg, heavy known 2.2 kg, mass estimate
  -10%, unmodelled CoM, friction estimate +10%; 2 mechanisms * 5 * 3 = 30 cases.
- Adaptive-only central-2700 sensitivity: 3 CoM * 3 poses = 9 cases. Total 111.

## Execution and diagnostics

dt=0.5 ms, maximum 4.5 s per trial. Initial fixture represents presentation
only and is released at 1.0 s; all scored hold/wielding is airborne. Hold until
1.5 s, smooth tilt 0 to 15 degrees over 2.5 s, then hold to 4.5 s. There is no
floor, shelf, hidden object support or success from a held fixture. A falling
or displaced object is retained as failure; no extrapolation after stop.

Record native Bullet contact order to reconstruct force signs; preserve
normal, two friction directions, actual contact points, resultant per pad,
normal spread, relative tangential velocity, object and tool poses, desired
wrist wrench and auxiliary witness. Independent sign and force-balance checks
precede interpretation. Numerical contact presence is N>0.05 N; also report
the source paper's N<=0.5 N weak-contact convention separately. Evaluate eta,
signed distance, required-margin deficit, time outside nominal cones, sliding,
drop/pose error, force and moment tracking separately, unexpected plate contacts,
and inertia/gravity ratio. Witness mismatch is not itself a tracking failure:
the per-pad forces are not commanded uniquely.

The positive reserve guarantee is evaluated at fixed nominal normal and mu.
Check the force-space Lipschitz inequality independently and separately report
the gap caused by changed normals. If a pad has multiple manifold entries,
evaluate their cones individually and label the resultant/averaged-normal
reduction. Never infer actual slip solely from eta or from Bullet's own
friction constraint; measure relative motion and object drift as well.

Only one bounded worker runs; BLAS uses one thread. Every trial has its own
directory and checkpoint, source/parameter hashes and failure record. Original
data, old analyses, ROS packages and production controllers remain unchanged.
