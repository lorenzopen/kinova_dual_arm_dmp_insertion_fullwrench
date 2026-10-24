# Steering di Lorenzo: «magari bastava alzare il damping»

Questa ipotesi va esaminata **prima di concludere che serva un controllore di
forza più complesso**. Non sostituisce il resto della ricerca autorizzata.

## Riscontri verificati

- Le 12 run riuscite FFI: `cart_k` traslazionale 1500, rotazionale 20;
  `cart_d` traslazionale 60, rotazionale 7 su entrambi i bracci.
- Nei 114 record deduplicati, 36 hanno parametri runtime completi: 34 con
  D=60/7 e 2 con D=80/7. Gli altri hanno damping non confermato dal metadata;
  non assegnare loro automaticamente il default corrente.
- Z online 23/10 12:18:41: D=80/7, K=1500/20, gain storico 0,5, canali none;
  `seat_found`. Banda 0,8–2,5 Hz 0,1583 N RMS media mani, tracking 0,3097 N RMS.
  L'onda quindi esisteva anche con 80, ma su un anello diverso dalla FFI.
- Z online 12:24:47: stessa configurazione dichiarata D=80/7, interrotta;
  non attribuire l'interruzione al damping senza esaminarne fase e motivo.
- I documenti storici riportano passaggio 5/1→60/7 e ritardi differenti, ma
  cambiano condizioni/leggi e non costituiscono una prova causale controllata.
  In particolare le frasi assolute di docs/squeeze_loop_math.md «60 made it
  worse» e «Higher cart_d only adds more lag» sono interpretazioni da verificare,
  non conclusioni da ripetere come dati. Non modificare quel documento storico.

## Confronto da aggiungere

1. Verificare la legge effettiva del controllore interno se il sorgente è
   disponibile localmente. Questo repository lo lancia da `kinova_control`,
   non contiene necessariamente la sua implementazione. Non inventare massa
   apparente, derivazione velocità, ritardi o unità non verificati.
2. Distinguere D cartesiano interno (feedback di velocità del robot) da D esterno
   sulla derivata della forza ATI: segnali, rate e dinamica sono diversi.
3. Mantenere fisso FFI/K/target/limiti; studiare una famiglia fisica massa–molla–
   smorzatore con incertezza d'inerzia/contatto, oltre ai modelli a ritardo.
   Uno sweep ipotetico D=60/80/100/120 può essere un confronto simulato,
   **non una proposta già validata di parametri robot**. Includere valori
   inferiori se utile per separare inerzia, attrito e ritardo.
4. Non convertire un diverso `cart_d` in un diverso ritardo puro o rapporto di
   smorzamento scelto arbitrariamente. La risposta D→impianto deve avere un
   modello dichiarato; se non è identificabile dai log, dirlo.
5. Misurare smorzamento, ritardo, rapidità, overshoot, minimi di forza e
   saturazione. Più damping può attenuare una risonanza e contemporaneamente
   rallentare il recupero: verificare entrambi gli effetti.
6. Se i dati non discriminano, formulare il minimo confronto sperimentale
   ripetuto che cambi soltanto D lasciando uguali FFI e condizioni iniziali.
   Preparare il protocollo, senza eseguire il robot durante questa ricerca.

Eseguire damping_audit.py per il catalogo con provenienza completa.
