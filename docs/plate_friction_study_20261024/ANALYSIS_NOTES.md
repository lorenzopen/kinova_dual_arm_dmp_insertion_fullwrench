# Interpretation and analysis choices

The physical campaign and optimizer settings are frozen by PROTOCOL.md and
runs_manifest.json. The analysis was developed while the campaign was running.
The endpoint thresholds of 20 mm geometric-centre error and 5 degrees attitude
error are **post hoc descriptive thresholds**, not preregistered acceptance
criteria. All continuous errors and physical stop reasons are also retained.
Counts are over three CoM cases and three deterministic initial orientations;
they are not empirical probabilities from independent random trials.

Physical stop criteria were frozen in study.py: object COM below 0.20 m or
more than 0.12 m from the reference, or excessive tool translation. Cone
utilization and margin deficits never stop a run. All failures remain in the
111-case denominator. Their tracking metrics use only the recorded airborne
prefix and must not be interpreted as equal-horizon comparisons.

The detailed time/force plots use the spatial CoM / zero initial angle pair,
to show tangential/torsional demand. This is an illustrative declared case,
not a best-case result or a representative population sample. Full metrics
are provided for every trial. Group medians can hide multimodality; the report
also states the failed Wong-inspired subgroups explicitly.

The score begins at 1.3 s (or at the recorded airborne prefix for earlier
losses). Numerical contact presence uses N>0.05 N; paper-style weak contact
uses N<=0.5 N. Both strict eta>1 and tolerant eta>1.001 are saved; a negative
cone distance below -0.001 N marks numerical excess. Sliding is a separate
diagnostic, speed>1 mm/s and eta>0.95 while loaded, not a universal failure
criterion. Contact friction in Bullet is itself a constraint of the simulator;
low observed cone violation does not independently validate Coulomb friction.

The Wong feasibility relaxation is a diagnostic-only baseline. Slack is never
introduced into the active SOCP or executed plan. Capability plots characterize
fixed-geometry contact cones only, not the complete attainable wrench set of
the passive spring mechanism. Normals/forces from the simulator are diagnostics
and do not enter the unknown-object controller; this entire campaign is an
explicit known-load oracle mechanics benchmark.

The four half-step runs reuse the original pilot IDs and are numerical QA,
specified after the core outcomes were visible. Spring damping coefficients
are kept fixed despite the constructors' dt-based caps. Classifications match;
the study does not claim convergence of all 111 cases or numerical robustness
to every Bullet contact parameter.
