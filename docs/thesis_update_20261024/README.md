# Aggiornamento tesi del 24 ottobre 2026

Progetto originale: `C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026`.
La copia preparata e compilata è in `project/`; gli originali dei file modificati
e il PDF precedente sono conservati in `originals/`, con hash in
`original_manifest.json`. `thesis_changes.diff` permette di rivedere il testo.

Sono aggiornati i capitoli 7 (interfaccia e valutazione SOCP), 10 (provenienza
del carico nominale d'inserimento), 11 (impedenza, FFI e confronti) e l'appendice
di riproducibilità. Tre schemi TikZ semplici usano lo stile esistente. Quattro
figure vettoriali conservano font serif e colori delle mani originali: normali,
zoom Z, momenti, arresto della prova con il codice del laboratorio.

Le curve complete del simulatore sono esplicitamente quelle con impedenza
interna approssimata; la prova col sorgente originale si ferma prima del
pickup e rimane separata. Il confronto include il limite di raffinamento del
passo Z e il risultato del damping 60/80. Non viene attribuita una causa certa
alle oscillazioni hardware. Il piccolo bias mediano della vecchia serie non
viene più descritto come precisione istantanea al decimo di newton.

`figures/data/kinova_comparison_20261024/` contiene i segnali compatti, le
identità degli input e le metriche. Lo script delle figure funziona senza
robot, ROS o nuove simulazioni. Nessun dato in NVIDIA_RUNS e nessun controller
di produzione sono stati modificati.

Verifiche: tre passaggi pdfLaTeX e BibTeX; nessun riferimento/citazione
mancante; nessun nuovo overfull (quattro già presenti in capitoli non
modificati); 25 hash delle fonti e cinque array compatti verificati; valori
della tabella confrontati alle metriche calcolate. Ventisei pagine renderizzate
e controllate visivamente. Compilazione: `build_staged.ps1`. La preview
integrata non ha potuto avviare Tectonic; è stato usato MiKTeX già installato,
con shell escape disabilitato.

`install_plan.json` identifica solo i file da applicare; `install_update.py`
verifica che nessun originale o dipendenza sia cambiato prima di scrivere.
`installed.json` viene creato soltanto dopo verifica degli hash di consegna.
