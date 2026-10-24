# Floating plate: immagini e schema per la tesi

Capitolo aggiornato: `04_contact_and_end_effectors.tex`. Schema frontale e
laterale nella figura 4.1 (pagina stampata 15); sezione sulle molle l0=0;
sei immagini da tre prove illustrate di contatto; grafico di inclinazione
e forze effettive dei pad sulla faccia inclinata. La tabella preesistente
di audit delle collisioni è stata adattata alla larghezza della pagina.

I due moduli copiati in `vendor/` coincidono per SHA-256 con il modello del
progetto TESI_antiG. Non sono stati modificati. La prova usa il floating plate
generale (d=20 mm, r=30 mm, sfere R=10 mm, k periferici=500 N/m, centrale
20 kN/m e 40/0/0 Nm/rad), non la molla assiale Kinova da 2700 N/m.
I tre casi geometrici sono dichiarati prima della simulazione: faccia piana,
faccia a 12 gradi e due facce inclinate di un corpo convesso. Flangia vincolata
in orientamento; avanzamento prescritto; gravità nulla; plate non collidente.
Non è una campagna SOCP, un successo d'inserimento o una misura hardware.

Una sola esecuzione bounded del bench: 12000 step da 0.25 ms per caso, durata
3 s. Nessun fitting sui dati reali. Le pose rilevate sono ri-renderizzate da
una camera fissa per rendere visibili le tre sfere; le due tinte dei fronti
sono solo grafiche. Tracce, pose, contatti e sorgenti sono preservati in
`bench/`, `bench_plate.py`, `render_views.py` e `vendor/`.

Esiti: 3/3 pad caricati al termine nei tre esempi, senza pretendere adattamento
a ogni geometria. La faccia a 12 gradi produce circa 11 gradi di inclinazione
del plate e carichi 0.24/6.33/6.64 N: contatti presenti ma non equamente caricati.
La rigidità relativa dei tre centri è verificata direttamente dalle pose.

La generazione delle figure è riproducibile con
`project/figures/scripts/fig_plate_conformity.py`, che legge i soli dati
compatti, senza rilanciare la fisica. I manifest riportano hash e provenienza.
PDF compilato con pdfLaTeX/BibTeX, nove pagine del capitolo renderizzate e
ispezionate, nessun nuovo overfull o riferimento mancante. Backup del capitolo,
documentazione figure e PDF precedente in `originals/`. `install.py` controlla
gli hash prima e dopo l'applicazione. NVIDIA_RUNS e controller di produzione
non sono stati modificati.
