# Analysis and thesis handoff - 24 October 2026

This branch contains the work produced during the force-controller analysis and
the subsequent PyBullet / passive-tool studies. The production controller is
unchanged by this handoff.

## Main entry points

- `plate_friction_study_20261024/RESULTS.md`: 111-case known-load per-pad
  friction study, with all physical failures retained. The complete illustrated
  report is `output/pdf/studio_coni_floating_plate.pdf` in that directory.
  `study.py` is the frozen physical worker; `metrics.csv`, `runs/`, `profiles/`
  and `figures/` contain metadata and outputs. Dense NPZ traces stay local;
  four half-step checks are separate.
  See `PROTOCOL.md`, `ANALYSIS_NOTES.md`, `DATA_DICTIONARY.md` and `ENVIRONMENT.json`.
- `analysis_20261023/controller_night/READING_GUIDE.md` and `RESULTS.md`: overnight
  modeling, damping, controller comparisons, limitations and held-out results.
- `analysis_20261023/controller_night/pybullet_nominal_20261024/README.md`:
  nominal insertion simulation, central-2700 sensitivity, Gen3 assets/dynamics
  work, and the later D80 comparison. See each subdirectory's results/protocol.
- `analysis_20261023/archive_review/REPORT.md`: experimental run and bag audit.
- `analysis_20261023/moving_average_sim/REPORT.md`: moving-average study.
- `thesis_plate_update_20261024/project/`: the latest complete staged thesis
  source from this chat (plate drawings and passive-contact explanation).
  `thesis_haption_update_20261024/` and `thesis_update_20261024/` preserve the
  earlier Haption/virtual-coupling and Kinova/SOCP editing passes, including
  diffs, sources and installation verification. The friction study is a
  separate report; it has not been inserted into these staged thesis sources.

## Data and runtime boundaries

`NVIDIA_RUNS/` is the original experimental archive and stays read-only and
local. No file in it has been moved, deleted, renamed or overwritten. Original
ROS bags are not uploaded in this commit; they are needed separately for a
fresh audit from raw hardware data. Existing derived metrics and plots are
included. The frozen baseline copies in the campaign directory are preserved.

Downloaded Python/Conda packages and DLLs under `pybullet_nominal_20261024/deps/`,
temporary renders/caches, and redundant checkpoint/delivery zip bundles stay
local. Their exclusion does not delete them. Runtime bootstrap scripts,
provenance and dependency references remain available. The scripts currently
describe the Windows workspace/runtime paths used for these results: on a new
machine install the dependencies and adapt those paths before rerunning. Do
not launch a second campaign worker against the same output directory.

The reports, plots and summary metrics can be inspected without running the
simulator. At the user's request, 396 dense NPZ data files remain local and
are excluded from this commit (about 424 MiB). Their paths, sizes and SHA-256
hashes are listed in `NPZ_LOCAL_ONLY_20261024.json`. The small archived TP-GMM
prior in the friction study's inputs is kept: it is a model input, not a trace.
Rerunning analyses that consume dense traces requires those files separately
or regenerating them. No local trace file has been deleted or changed.

The friction study's original `SHA256.json` seals the complete local delivery,
including NPZ traces. On the lightweight checkout, a complete-manifest check
will therefore report these deliberately missing files; the manifest has not
been rewritten to conceal their omission. Source/data/report files that are
included retain their original bytes. In addition,
absolute original-paper/source references in its audit describe the originating
machine and may be unavailable on a different checkout. The physical benchmark
uses a known load and wrist proxies; it does not certify hardware contact cones
or reproduce the full Wong system or Kinova insertion experiment.

Git attributes preserve exact bytes in the sealed campaign/thesis directories,
including line endings, so checkout does not invalidate their SHA-256 manifests.
