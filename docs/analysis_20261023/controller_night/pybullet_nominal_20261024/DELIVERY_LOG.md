# Registro finale della nuova prova PyBullet

- Richiesta iniziale: stesso FFI/SOCP autonomo; poi centrale2700N/m, poi due Gen3 nella geometria del laboratorio.
- Primo backend flottante: Z/L/S/T e raffinamenti; central2700 implica4200N/m totali. Interpretazione precedente totale2700 superata e conservata.
- Gen3 articolati: mesh ufficiali e URDF locale, trasformazioni del lab e FK verificati. Rimossa nella sola copia la massa predefinita Bullet1kg di un frame privo di inerzia.
- Servo approssimato: esplicito instabile; implicito nominale completa Z/L, ma Z cambia77.2→4.4mm col raffinamento. Risultato preliminare, nessuna attribuzione hardware.
- Nuovo steering: trovato controller originale2092 righe nei Download. Snapshot immutato; classi estratte verbatim e Pinocchio isolato. Il codice inviato da Lorenzo conferma compute_torque/estimate_wrench mediante confronto AST.
- Loop simulato: K/D originali, Kalman/RC, clip35/12, slew0.5, watchdog2rad/s20tick, friction scale0, torque_sensor=-tau_applied precedente, pubblicazione250Hz e ATI separati. Homing non simulato; covarianza fresca/assestata dichiarate.
- La legge originale sul plant URDF abortisce già in aria. Verificati FK/J/gravità, assenza collisioni iniziali, sensore in statica e segni. Pinocchio ABA indipendente senza contatti conferma crescita prima del watchdog a due dt, sia con Kalman fresco sia assestato.
- Conclusione: integrazione controller realizzata; plant non ancora validato per giudicare l'inserimento. Nessun damping/inerzia arbitrari introdotti per ottenere una riuscita. Dati minimi mancanti nel report.
- Nessun processo di simulazione lasciato attivo; nessun ROS/robot/deploy/push. NVIDIA, baseline e332 sorgenti runtime invariati con verifica SHA-256 finale.
