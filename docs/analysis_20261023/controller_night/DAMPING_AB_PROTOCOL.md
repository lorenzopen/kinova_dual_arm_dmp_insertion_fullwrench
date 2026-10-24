# Confronto futuro sul damping — preparato, NON eseguito

Scopo: distinguere se più damping interno riduce l'onda/picco mantenendo la
capacità di inseguire il target. Non è un ordine di avvio del robot e non cambia
alcun parametro di produzione.

## Configurazioni candidate

- A: baseline delle FFI riuscite, `cart_d = 60 60 60 7 7 7`.
- B: aumento traslazionale isotropo, `80 80 80 7 7 7`.
- C: incremento sul solo asse y di base, `60 80 60 7 7 7`.

C è motivata dalle normali registrate, quasi parallele a y delle rispettive basi.
Prima di applicarla va verificata la semantica nel sorgente effettivo del servo.
Se non confermata, non assumere che quei coefficienti agiscano nelle basi.
Non cambiare contemporaneamente rigidezza, gain I, filtri, attrito/Kalman,
velocità, profilo SOCP, canali wrench o geometria/tool.

## Disegno minimo

1. Conservare la baseline corrente e acquisire il sorgente/versione effettiva
   del controllore interno, inclusi calcolo velocità, filtri e limiti coppia.
2. Usare la stessa route Z già riuscita e il relativo profilo/FFI invariato.
   Stessa posa e forza iniziale, stesso pezzo, tara comparabile, stessa board.
   Annotare eventuali variazioni di presa/contatto senza cancellare tentativi.
3. Per un pilot A/B usare ordine A–B–B–A, con ripristino delle condizioni a
   ogni tentativo. È uno screening di quattro run, non una stima definitiva.
4. Se utile proseguire con almeno tre blocchi A–B–B–A o un ordine bilanciato
   fissato prima di osservare le forze. Eventuale A/C costituisce una variante
   separata. Non scegliere soltanto run riuscite o le finestre più favorevoli.
5. Ripetere poi sulla L per verificare il transitorio route→slow_seat, se il
   primo confronto non mostra peggioramenti rilevanti. Nessuna modifica del
   target per rendere più facile il confronto sul damping.

## Misure necessarie

Registrare i parametri runtime effettivi di entrambi i bracci, versione codice,
ATI per mano, pose target e misurate, q/dq, torque se disponibile, timestamp,
stati/limiti FFI e motivo di ogni interruzione. Conservare sia onda sia minimi,
senza sostituire la forza reale con una curva filtrata nel resoconto.

Confrontare per route/fase omogenea: RMS 0,8–2,5 Hz e 2,5–10 Hz, errore RMS
rispetto al target originale, picchi positivi/negativi, forza minima di ogni
mano, ritardo comando→moto, inversione del moto e saturazione di velocità.
Un grafico più liscio con perdita di forza nel cambio target non è un successo.
Confrontare coppie/blocchi e riportare variabilità, non solo medie aggregate.

Il pilot serve a discriminare l'ipotesi; eventuali soglie di accettazione
vanno stabilite rispetto ai requisiti della tesi prima di una validazione finale.
Restano attivi tutti i controlli esistenti di forza, tracking e freschezza.
Nessun dato nuovo va scritto o sovrascritto dentro NVIDIA_RUNS.
