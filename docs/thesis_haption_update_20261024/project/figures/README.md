# Thesis figures

One folder per chapter. The LaTeX includes `figures/<chapter>/<name>.pdf`
(the macro `\thesisfigure` in `preamble.tex` adds the `figures/` prefix); every
figure also has a `.png` beside it for quick viewing. **Every figure is made by
a script in `scripts/`**, run from that folder:

```
cd thesis_english_2026/figures/scripts
python fig_ucm.py
```

| figure | chapter | script | data |
|---|---|---|---|
| `ch05_prior/prior_query` | 5 prior | `../evidence/make_figures.py` | `lib/prior.py`, session-C prior |
| `ch05_prior/ucm_session_c` | 5 prior | `scripts/fig_ucm.py` | raw exp13 recordings, session C (same index as `07_torsional_ucm/B_ucm/24_ucm_atlas.py`) |
| `ch06_estimation/observability` | 6 estimation | `../evidence/make_figures.py` | analytic |
| `ch08_evidence/torsional_index` | 8 evidence | `../evidence/make_figures.py` | `07_torsional_ucm/docs/CONFRONTO_RIGIDEZZA_UCM.md` |
| `ch09_geometry/geometry_ladder` | 9 geometry | `../evidence/make_figures.py` | illustrative, seed 26 |
| `ch09_geometry/polytope_zoo` | 9 geometry | `scripts/fig_zoo.py` | `10_tripad_passivo/campaign_geometry/populations` |
| `ch10_tetris/tetris_shoulder_case` | 10 Tetris | copied by `../evidence/make_figures.py` | saved diagnostic picture |
| `ch10_tetris/pipeline_snapshots` | 10 Tetris | `scripts/fig_snapshots.py` | `data/s7025_on/snaps` |
| `ch10_tetris/nest_poses` | 10 Tetris | `scripts/fig_snapshots.py` | `data/s7025_on/snaps` |
| `ch10_tetris/planned_vs_pybullet` | 10 Tetris | `scripts/fig_boards.py` | `data/s7025_on/boards.json`, `game.csv` |
| `ch10_tetris/hang_estimate` | 10 Tetris | `scripts/fig_hang_estimate.py` | `data/s7025_on/game.csv` |
| `ch10_tetris/insertion_forces` | 10 Tetris | `scripts/fig_insertion_forces.py` | `data/s7025_on`, `data/s7025_off` traces |
| `ch10_tetris/learnt_routes` | 10 Tetris | `scripts/fig_learnt_routes.py` | `kinova_dual_arm_dmp_insertion/cubes100/routes/narrow75_100` |
| `ch10_tetris/lean_to_pybullet` | 10 Tetris | `scripts/fig_lean_to_pybullet.py` | GIFs in `C:\Users\Lorenzo\Desktop\TESI_GIF` (outside the repo) |
| `ch11_kinova/cell_top_view` | 11 Kinova | `scripts/fig_kinova.py` | `config/cell.yaml` of the robot package (outside the repo, `KINOVA_REPO`) |
| `ch11_kinova/planned_vs_executed` | 11 Kinova | `scripts/fig_kinova.py` | route files + `docs/runs/2026102*_twist_ati` logs of the robot package |
| `ch11_kinova/wrench_one_run` | 11 Kinova | `scripts/fig_kinova.py` | L run 22/10 15:00 (`docs/runs/20261022_F75_Lt1_s9014_twist_ati/1_1500_riuscita`) |
| `ch11_kinova/ati_vs_estimate` | 11 Kinova | `scripts/fig_kinova.py` | same run |
| `ch11_kinova/runs_table.tex` | 11 Kinova | `scripts/fig_kinova.py` | the ten ATI runs; summary in `data/kinova_runs_summary.json` |

## The game data (`data/`)

`data/s7025_on` and `data/s7025_off` are ONE game each (seed 7025, 7-bag,
today's defaults), the second with `--no-learnt` (straight line on every move).
They are recorded by `scripts/trace_game.py`, which runs the package's
`game.py` unchanged and only WRITES DOWN, every 10 ms, the forces, the target
squeeze, the slot push, the estimate and the truth (report only), plus camera
snapshots at fixed steps. To redo them (about 45 min each):

```
cd thesis_english_2026/figures/data
python ../scripts/trace_game.py --out s7025_on  -- --seed 7025
python ../scripts/trace_game.py --out s7025_off -- --seed 7025 --no-learnt
```

`trace.npz` is git-ignored (as every `.npz`); the snapshots, `boards.json` and
`game.csv` are small and kept.

`scripts/style.py` holds the common look (serif, piece colours of the
simulation, the two hand colours) and `save()`, which writes the pdf and png.

## Articulated Gen3 comparison (24 October 2026)

`scripts/fig_kinova_comparison.py` regenerates `ch11_kinova/real_sim_normal`,
`real_sim_zoom`, `real_sim_moment` and `original_source_stop` as PDF/PNG.
It uses the common `style.py` hand colours and typography. Compact plotted
signals and original source SHA-256 identities are in
`data/kinova_comparison_20261024/manifest.json`; `metrics.json` records the
descriptive route/seat metrics. The full insertion curves use approximate
inner impedance; the original-source trial is shown separately to its early
stop. No new physics or tuning is performed by the plotting script. The three
block diagrams are editable TikZ in Chapters 7, 10 and 11.

## Haption and virtual coupling diagrams (24 October 2026)

Chapter 3 contains two editable TikZ illustrations: the recording architecture
with schematic Haption silhouettes, and one translational spring-damper
coupling. They use the existing blue `#1f6fb2` and orange `#d9622b` hand
colours with light tints and neutral grey objects. The silhouettes are
explanatory drawings, not device CAD. The diagram distinguishes the elastic
wrench log from the filtered/scaled/clipped feedback and from the complete
proxy command. No experimental values or controller source are changed.
