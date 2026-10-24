# Numerical refinement (quality assurance, not tuning)

The four original pilot cases are reused once at half the physics time step:
welded/adaptive/centered/0, floating/adaptive/centered/0,
welded/adaptive/spatial/2 and floating/Wong/spatial/2. Selection follows the
existing pilot manifest. This check is specified after the core outcomes have
been seen; it is not a preregistered performance experiment or a new tuning set.

Do not start until the 111-case worker has finished. One worker, BLAS one thread.
Copy the frozen simulator to a separate refinement source and change only
DT=0.0005 to 0.00025 and the recording divider to retain 100 Hz. The spring
constructors retain their original dt=0.0005 argument: this model uses dt to
cap damping coefficients, so passing the smaller dt there would inadvertently
change the physical damping as well. The actual integration/stability dt is
0.00025. Read the same profiles. Keep all physical coefficients and solver
parameters identical. Preserve
the source diff/hash, both traces and all failures. Compare endpoint outcome,
final pose error, wrist-force RMSE, per-pad minimum reserve and margin deficit.
Do not tune gains in response. Classification changes or large quantitative
changes limit the conclusion and must be reported explicitly.
