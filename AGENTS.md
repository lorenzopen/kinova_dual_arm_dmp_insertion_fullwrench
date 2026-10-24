# Preservation of experimental data

The user explicitly requires that `NVIDIA_RUNS/` never be deleted. Treat the
entire directory, including CSV logs, metadata, and bags, as read-only source
data: do not delete, move, rename, overwrite, clean, or generate files inside it.
Write derived analysis, manifests, plots, and reports outside that directory,
normally under `docs/analysis_20261023/` or a separate dated analysis directory.
