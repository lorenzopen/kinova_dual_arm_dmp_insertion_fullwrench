# Prova richiesta: damping traslazionale80

Cambio scelto da Lorenzo: D traslazionale60→80Ns/m, D rotazionale7 invariato.
Nessuna ricerca di gain. Centrale2700 e tre pad500N/m; massa, contatti,
geometria, FFI, SOCP, corsa/velocità e passo rimangono quelli della baseline.

Confronti già fissati prima della lettura dei nuovi risultati:

- Z, Gen3 articolati/servo implicito approssimato, dt0.25ms: baseline
  Gen3_Z_central2700_implicit_dt025 contro nuova D80.
- L, stesso backend, dt0.5ms: baseline Gen3_L_central2700_implicit_dt05
  contro nuova D80. Il passo di L non si confronta con quello di Z.
- Legge originale, Z, dt0.25ms, Kalman assestato2s: baseline
  Gen3Lab_Z_central2700_warm_dt025 contro nuova D80. Non si sostituiscono
  gli eventuali abort in aria con curve dell'altro backend.

Un solo worker fisico alla volta. Sorgenti originali dei runner e tutte
le baseline rimangono conservati; nuove copie parametrizzate e provenienza
in gen3/damping_runner_provenance.json. Le classi del controller del
laboratorio restano immutate, cambiano soltanto i parametri D passati loro.

Valutazione: grafici per mano, errore sul target originale, bande di
oscillazione della forza e dell'errore, transitori, minimi per mano,
rate/corsa FFI e posa geometrica finale. Nessun risultato è una nuova
taratura già validata sul robot. NVIDIA_RUNS sempre in sola lettura.
