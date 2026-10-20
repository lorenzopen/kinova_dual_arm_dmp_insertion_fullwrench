# Cinque rotte alternative: orientamenti diversi, colonne 2–5

Preparato il **20/10/2026** per la copia **fullwrench**. Tutti e cinque i mondi sono
diversi da quelli di [ESEMPI_5_PEZZI.md](ESEMPI_5_PEZZI.md). Sono rotte già esportate
e provate in PyBullet a cubi da 100 mm, raccolte qui con le rispettive board.
Non sono nuove traiettorie inventate o traslate a mano.

La prima è la **T con la parte piatta in alto e il gambo in basso**, come richiesto.
La seconda è la T capovolta; seguono una J orizzontale, una S orizzontale e una O
con due soli vicini. Per la O cambia il contesto, non l'orientamento del quadrato.
Servono **3 o 4 pezzi in totale**, contando quello da inserire.

**Numerazione come nelle immagini: colonne 0, 1, 2, 3, 4, 5.** L'intera sagoma
finale dello slot è fra le colonne 2 e 5; i vicini possono occupare anche 0 e 1.
Le colonne aumentano verso le basi. La zona ammessa va da x scaffale 205 a circa
612 mm. Le prime quattro sagome stanno nelle colonne 2–4, la O nelle 3–4.
Il vincolo riguarda lo slot finale; il pezzo si muove sopra la board lungo la rotta disegnata.

Tutte usano **`R=c100_narrow75`**, senza `--lift`: con la cella e il tool attuali,
la distanza massima calcolata della flangia dalla spalla resta circa **772 mm**
nella sequenza `check`, contro il limite geometrico configurato di 902 mm.
È una verifica di distanza, non una verifica di IK, singolarità, collisioni o rumore.
Il comportamento sul robot va verificato con i giri a mani vuote sotto.

| # | Inserimento, visto dal sinistro | W | Vicini da montare | PyBullet placed / slip [mm] |
|---|---|---|---|---|
| 1 | T: barra in alto, gambo in basso | `T_s7039m4` | J, I, S | 2.5 / 2.9 |
| 2 | T capovolta: barra in basso, gambo in alto | `T_s7037m3` | L, O | 2.9 / 1.7 |
| 3 | J: barra in alto, piede in basso a destra | `J_s7109m3` | Z, I | 4.0 / 5.6 |
| 4 | S orizzontale | `S_s7073m3` | O, J | 9.0 / 4.8 |
| 5 | O: board con due soli vicini | `O_s7094m3` | Z, I | 1.3 / 0.8 |

**J = L girata sul retro; S = Z girata sul retro.** Ogni board è costruibile con
il kit I/O/T/L/Z, senza richiedere due copie dello stesso pezzo. Monta soltanto i
vicini indicati: aggiungere gli altri pezzi cambierebbe la board della rotta.
Fissali come nelle prove precedenti: alcuni invadono lo slot e non sono sulla griglia.

I numeri PyBullet sono quelli salvati con le rotte, con il controllo simulato
dell'epoca. Non misurano il nuovo SOCP sul robot. La S è vicina alla soglia di
10 mm di errore di piazzamento: lasciala per ultima.

## Figure, quote e presa

Vista dal **braccio sinistro .200**, verso il destro .107. Pezzo pieno in alto =
come presentarlo a mano; sagoma tratteggiata = posizione finale; **X = centro
della presa sulle due facce**, cerchio puntinato = impronta del tri-pad.
Quote in mm nel frame scaffale: x dal bordo sinistro della colonna 0, z dal pavimento.
Le quote dei vicini sono ingombri; orientali come in figura, non soltanto dal rettangolo di ingombro.

### 1. T normale: barra in alto e gambo in basso — `T_s7039m4`

```bash
export W=T_s7039m4
export R=c100_narrow75
```

![T con barra orizzontale in alto e gambo verso il basso](boards/col2_5/T_s7039m4.png)

| Pezzo | x [mm] | z [mm] |
|---|---|---|
| J = L girata | 19–319 | 1–201 |
| I | 497–597 | 1–401 |
| S = Z girata | 21–221 | 103–403 |
| **T, slot** | **209–509** | **0–200** |

**O non usata.** La T parte già dritta come nello slot, senza inclinazione iniziale:
baricentro x 359, z 890; punte a x 359, z 915, sul **cubo centrale della barra alta**.
Baricentro finale nella base destra: x 449, y 298, z 95.

### 2. T capovolta: barra in basso e gambo in alto — `T_s7037m3`

```bash
export W=T_s7037m3
export R=c100_narrow75
```

![T capovolta con barra in basso](boards/col2_5/T_s7037m3.png)

| Pezzo | x [mm] | z [mm] |
|---|---|---|
| L | 296–596 | 1–201 |
| O | 21–221 | 1–201 |
| **T, slot** | **209–509** | **100–300** |

**I e Z non usate.** Baricentro iniziale x 359, z 837; punte x 359, z 912,
sul **cubo singolo in alto**, non sul centro della barra bassa.
Parte già nell'orientamento finale, senza inclinazione iniziale.
Baricentro finale nella base destra: x 449, y 298, z 145.

### 3. J orizzontale: piede in basso a destra — `J_s7109m3`

```bash
export W=J_s7109m3
export R=c100_narrow75
```

![J orizzontale con piede in basso a destra](boards/col2_5/J_s7109m3.png)

| Pezzo | x [mm] | z [mm] |
|---|---|---|
| Z | 21–321 | 1–201 |
| I | 497–597 | 1–401 |
| **J = L girata, slot** | **209–509** | **0–200** |

**O e T non usate.** Inserisci la L girata sul retro. La barra da tre cubi è
orizzontale in alto; il quarto cubo scende a destra. Baricentro iniziale x 384,
z 890; punte x 359, z 915, sul **cubo centrale della barra alta**.
Parte già come nello slot, senza inclinazione iniziale.
Baricentro finale nella base destra: x 423, y 298, z 95.

### 4. S orizzontale — `S_s7073m3`

```bash
export W=S_s7073m3
export R=c100_narrow75
```

![S orizzontale, ottenuta girando la Z sul retro](boards/col2_5/S_s7073m3.png)

| Pezzo | x [mm] | z [mm] |
|---|---|---|
| O | 21–221 | 1–201 |
| J = L girata | 397–597 | 1–301 |
| **S = Z girata, slot** | **209–509** | **0–200** |

**I e T non usate.** Inserisci la Z girata sul retro: coppia bassa verso sinistra,
coppia alta verso destra. Baricentro iniziale x 359, z 865; punte x 359, z 915,
sul **cubo in alto a sinistra**, quello sopra il cubo destro della coppia bassa.
Parte già come nello slot, senza inclinazione iniziale.
Baricentro finale nella base destra: x 449, y 298, z 70.

### 5. O con due vicini — `O_s7094m3`

```bash
export W=O_s7094m3
export R=c100_narrow75
```

![O con due soli vicini Z e I](boards/col2_5/O_s7094m3.png)

| Pezzo | x [mm] | z [mm] |
|---|---|---|
| Z | 22–322 | 1–201 |
| I | 498–598 | 1–401 |
| **O, slot** | **310–510** | **0–200** |

**L e T non usate.** Baricentro iniziale x 410, z 865; punte x 360, z 915,
sul **cubo in alto a sinistra**. Parte dritta come nello slot.
Baricentro finale nella base destra: x 398, y 298, z 70.

## Prima volta: trasferire questa copia

Tutti i comandi successivi sono **Bash sul PC robot Ubuntu/Jazzy**. Si assumono
già presenti `~/lorenzo/env.sh`, il venv e `~/lorenzo/gen3_bare.urdf`, come in
[SETUP_PC_ROBOT.md](../SETUP_PC_ROBOT.md).

Per trasferire anche queste modifiche locali, **PowerShell sul PC Windows**,
sostituendo `<IP_PC_ROBOT>` con l'IP del PC NVIDIA (non l'IP di un braccio):

```powershell
scp -r "C:\Users\Lorenzo\Desktop\kinova_dual_arm_dmp_insertion_fullwrench" nvidia@<IP_PC_ROBOT>:~/lorenzo/
```

Se invece le modifiche sono già state pubblicate nel repository, aggiorna sul
**PC robot, T4**, con gli script di movimento fermi:

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
git pull --ff-only
```

Usa sempre questa cartella fullwrench, fuori da `ws/src`. Non serve `colcon build`.
Il solo `git pull` non trasferisce modifiche ancora locali al PC Windows.

## T1 e T2 — controller, una volta per sessione

Terminali normali, **senza `env.sh`**. Sono gli stessi parametri k1000 della prova
reale precedente. I comandi avviano anche la home: prima libera la zona.

**T1 — destro .107:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.107 --ros-args -r __ns:=/right -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

**T2 — sinistro .200:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.200 --ros-args -r __ns:=/left -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

Aspetta entrambi in home e fermi. Lascia T1 e T2 aperti.

## T3 — registra l'intera sessione

```bash
source ~/lorenzo/env.sh
mkdir -p "$LORENZO/bags"
cd "$LORENZO/bags"
ros2 bag record -e "/(left|right)/.*" -o varieta_col2_5_$(date +%Y%m%d_%H%M%S)
```

Lascia registrare. Ctrl-C in T3 soltanto a fine sessione.

## T4 — scegli la rotta e crea il profilo

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
python -c 'import numpy, scipy, cvxpy, clarabel, yaml, rclpy, pinocchio; import kinova_dual_arm_dmp_insertion.socp as s; print(s.__file__)'
```

Il percorso stampato deve essere quello della cartella **fullwrench**. Seleziona
**un solo mondo**, copiando le due righe dalla sua sezione sopra. Per iniziare:

```bash
export W=T_s7039m4
export R=c100_narrow75
```

Poi, **una volta per ogni nuova prova**:

```bash
export CFG="config/run_varieta_${W}_k1000.yaml"
export BASELINE="logs/baseline_varieta_${W}_k1000_$(date +%Y%m%d_%H%M%S).csv"
mkdir -p logs
python - "$LORENZO/gen3_bare.urdf" <<'PY'
import os
import shlex
import sys
from pathlib import Path
import yaml

urdf = Path(sys.argv[1]).resolve()
assert urdf.is_file(), f"URDF mancante: {urdf}"
cfg = yaml.safe_load(Path("config/cell.yaml").read_text())
for side in ("left", "right"):
    cfg[side]["urdf_path"] = str(urdf)
cfg["stiffness"] = {"trans": 1000.0, "torsion": 20.0, "tilt": 20.0}
cfg["socp"] = {"r_s": 0.30, "w_force": 0.0, "no_bending": False}
cfg["full_wrench"] = False
cfg["allow_hand_wrench"] = False
cfg["full_wrench_channels"] = "twist"
cfg["wrench_baseline"] = None
Path(os.environ["CFG"]).write_text(yaml.safe_dump(cfg, sort_keys=False))
Path("config/session_varieta.env").write_text("\n".join(
    f"export {key}={shlex.quote(os.environ[key])}"
    for key in ("W", "R", "CFG", "BASELINE")
) + "\n")
print("Mondo:", os.environ["W"], "Rotte:", os.environ["R"])
print("Profilo:", os.environ["CFG"])
print("Selezione per T5: config/session_varieta.env")
PY
```

Questo esclude la vecchia baseline della Z e prepara una selezione comune a T4
e T5. Ripetere la creazione del profilo ne azzera la baseline: non rifarla fra
la registrazione della baseline e l'inserimento. Le misure della cella e del tool
sono quelle di `config/cell.yaml`: se cambi l'allestimento, aggiornale prima.

## T4 — controlli senza movimento

```bash
python -m kinova_dual_arm_dmp_insertion.main check "$W" --arm "$R" --config "$CFG" --slow 3
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.20 --dry
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.10 --dry
```

Sono controlli software, senza robot. Se la distanza stampata supera il limite,
fermati e correggi l'allestimento. La distanza qui calcolata non controlla il
tragitto dalla posizione attuale del robot fino alla presa.

Controlla i topic, **un comando alla volta, Ctrl-C dopo qualche secondo**:

```bash
ros2 topic hz /left/joint_states
ros2 topic hz /right/joint_states
ros2 topic hz /left/estimated_wrench
ros2 topic hz /right/estimated_wrench
```

## T4 — giri a mani vuote, senza lift

**Niente pezzo in questi tre giri**, anche se lo script mostra il messaggio
generico di mettere il pezzo fra i pad. Qui premi Enter con le mani vuote.
Esegui un comando alla volta e passa al successivo solo se il precedente va bene.
Non aggiungere `--lift 0.10`: alzerebbe la presa e aumenterebbe l'estensione.

**Primo giro: senza board**, per vedere tutto il percorso in aria:

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.20 --duration 60
```

**Secondo giro: monta e fissa i vicini della figura**, ancora senza pezzo da inserire:

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.20 --duration 60
```

**Terzo giro: stessa board, distanza di presa, mani vuote:**

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.10 --duration 60
```

Questi tre comandi muovono davvero i bracci. Non c'è controllo automatico delle
collisioni fra avambracci. Se senti lo stesso rumore o vedi una postura troppo
estesa, interrompi la prova e non passare all'inserimento. Tieni disponibile l'E-stop.

## T4 — test delle stime dei momenti, prima del full wrench

Questa routine **registra soltanto**, senza pubblicare target o attivare il
controllo dei momenti. Misura il rumore; non abilita automaticamente
`full_wrench: true` e non certifica la stima.

### 1. Preparazione e azzeramento, bracci fermi

**T1/T2:** controller attivi in mantenimento, entrambi i bracci fermi, tool
montati, niente pezzo e niente a contatto. Nessun `insert` o `follow` in
esecuzione in T4/T5. T3 può continuare a registrare. Tieni l'E-stop disponibile;
non torcere i pad a mano durante questa prima misura.

**T4:** recupera il profilo già creato, senza rigenerarlo:

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
source config/session_varieta.env
printf 'Test momenti: mondo=%s profilo=%s\n' "$W" "$CFG"
mkdir -p logs
export MOMLOG="logs/verifica_momenti_${W}_$(date +%Y%m%d_%H%M%S).csv"
```

Con i bracci fermi e nessun contatto, azzera **un braccio alla volta**:

```bash
ros2 service call /left/zero_wrench std_srvs/srv/Trigger '{}'
```

```bash
ros2 service call /right/zero_wrench std_srvs/srv/Trigger '{}'
```

Controlla che **entrambe le risposte riportino `success: true`** (oppure
`success=True`). Se il servizio manca o fallisce, non considerare le letture
azzerate. Le chiamate sono separate per verificare la risposta: il logger
successivo non deve rifare il tare.

### 2. T4 — registra almeno 30 secondi senza contatti

```bash
python -m kinova_dual_arm_dmp_insertion.wrench_log --config "$CFG" --out "$MOMLOG"
```

Quando compare `logging to ...`, scrivi **nel logger in T4**:

```text
fermo
```

Premi Invio, lascia registrare almeno **30 secondi senza toccare nulla**, poi
**Ctrl-C in T4**. Non chiudere T1/T2. Il riepilogo del logger mostra le forze;
i momenti sono salvati nelle colonne `tx`, `ty`, `tz` del CSV, in **Nm**.

### 3. T4 — analizza il CSV, senza muovere i robot

Nello stesso terminale, dove esiste ancora la variabile `$MOMLOG`:

```bash
python - "$MOMLOG" <<'PY'
import csv
import sys
from pathlib import Path
import numpy as np

path = Path(sys.argv[1])
with path.open(newline="") as f:
    rows = list(csv.DictReader(f))
marks = list(dict.fromkeys(
    r["mark"] for r in rows if r["mark"].startswith("fermo")
))
if not marks:
    raise SystemExit("Nessun tratto marcato fermo: scrivi fermo + Invio nel logger.")
for mark in marks:
    for arm in ("L", "R"):
        rr = [r for r in rows if r["arm"] == arm and r["mark"] == mark]
        if len(rr) < 2:
            print(arm, mark, "CAMPIONI INSUFFICIENTI")
            continue
        a = np.array([[float(r[k]) for k in ("tx", "ty", "tz")] for r in rr])
        if not np.isfinite(a).all():
            print(arm, mark, "DATI NON FINITI: misura non utilizzabile")
            continue
        duration = float(rr[-1]["t"]) - float(rr[0]["t"])
        n = max(1, len(a) // 3)
        print(f"\n{arm} {mark}: {len(a)} campioni, {duration:.1f} s; assi tx ty tz [Nm]")
        print("  media:                ", np.round(a.mean(axis=0), 4))
        print("  deviazione standard:   ", np.round(a.std(axis=0), 4))
        print("  intervallo picco-picco:", np.round(np.ptp(a, axis=0), 4))
        print("  media fine - inizio:   ", np.round(a[-n:].mean(axis=0) - a[:n].mean(axis=0), 4))
        if duration < 30:
            print("  Tratto inferiore a 30 s: registra un tratto piu lungo.")
print("\nCSV:", path.resolve())
PY
```

Se riapri T4, prima dell'analisi imposta `MOMLOG` al **file effettivamente
registrato**, per esempio `export MOMLOG="logs/verifica_momenti_...csv"`,
sostituendo i puntini con il suo nome completo.

Media = residuo dopo il tare; deviazione standard e picco-picco = oscillazioni;
differenza fra media finale e iniziale = deriva durante la misura. Con il
`tool_in_ee` attuale, l'asse dei pad coincide con `z` dell'end effector:
**`tz` è quindi il canale principale per il twist**. Sono momenti grezzi sul
braccio, negli assi locali dell'end effector: il backend li trasforma e cambia
il segno per descrivere la mano sul pezzo. Non confrontare direttamente i segni
dei due bracci senza tener conto dei loro assi.

### 4. Verifica della dipendenza dalla postura

Una misura subito dopo il tare non verifica le altre posture. Per una seconda
prova, crea un nuovo file con la riga `export MOMLOG=...` del punto 1 e ripeti
le due chiamate di azzeramento una sola volta nella postura iniziale. Avvia il
logger del punto 2 e marca `fermo_A`; registra almeno 30 secondi. Prima di un
trasferimento scrivi `spostamento`, così l'analisi non include quel tratto.
Usa soltanto posture e movimenti già verificati nelle prove a vuoto. Raggiunta
la seconda postura e con entrambi i bracci nuovamente fermi, scrivi `fermo_B`
e registra altri 30 secondi, **senza rifare il tare**. Poi Ctrl-C e la stessa
analisi del punto 3. Non usare tratti in movimento come rumore statico.

Se cambia anche l'orientamento della flangia, gli assi locali ruotano: confronta
i componenti nello stesso sistema di riferimento. `tz` resta il canale del
twist se il montaggio del tool non cambia. Differenze a vuoto possono indicare
errori dipendenti dalla postura, non contatti reali.

### 5. Segno e ampiezza: serve un riferimento noto

Il test a vuoto misura rumore e deriva, **non la correttezza del guadagno o del
segno**. Per questi serve un sensore F/T di riferimento oppure un'attrezzatura
di laboratorio fissata con leva e dinamometro. Una forza perpendicolare applica
un momento `M = F * r`: `r` va misurato rispetto all'asse e all'origine corretti.
Verifica entrambi i versi, ripetibilità, componenti sugli altri assi e ritorno
allo zero dopo aver tolto il carico. Non fare il tare con il carico applicato.
Non improvvisare una leva sulle parti stampate o sulle molle: servono un
fissaggio adatto e carichi compatibili con il tool e il setup del laboratorio.

Torcere il pad a mano dà soltanto una risposta qualitativa. Un metodo con leva
e forza misurata è descritto nella [guida NPL alla calibrazione dei trasduttori
di momento](https://eprintspublications.npl.co.uk/4092/).

**Non c'è una soglia universale in Nm per passare a `full_wrench: true`.**
Se rumore, deriva o errore sono dello stesso ordine dei momenti che il SOCP
richiede, la stima non è sufficientemente affidabile per quel controllo.
Il problema della presa che si allenta va verificato anche sul canale di forza;
questa routine non lo risolve. L'attuale script non ha un arresto dedicato per
momento eccessivo o errore angolare.

## T4 + T5 — baseline della rotta selezionata

**T4**, bracci fermi e niente a contatto:

```bash
python -m kinova_dual_arm_dmp_insertion.wrench_log --config "$CFG" --tare --out "$BASELINE"
```

Aspetta il messaggio di logging dopo il tare di entrambi i bracci.
**T5**, mentre T4 registra, sempre a mani vuote:

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
source config/session_varieta.env
printf 'Baseline: mondo=%s rotte=%s profilo=%s\n' "$W" "$R" "$CFG"
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.10 --duration 60
```

T5 legge esattamente la scelta salvata in T4. Non cambiare mondo durante la baseline.
Quando il giro è finito, **Ctrl-C al logger in T4**, poi in **T4**:

```bash
python - "$CFG" "$BASELINE" <<'PY'
import sys
from pathlib import Path
import yaml
from kinova_dual_arm_dmp_insertion.ros_backend import _Baseline

p, baseline = Path(sys.argv[1]), Path(sys.argv[2]).resolve()
assert baseline.is_file(), f"Baseline mancante: {baseline}"
for side in ("L", "R"):
    b = _Baseline(str(baseline), side)
    assert len(b.P) > 0, f"Nessun campione per {side}"
    print(side, len(b.P), "campioni")
cfg = yaml.safe_load(p.read_text())
cfg["wrench_baseline"] = str(baseline)
p.write_text(yaml.safe_dump(cfg, sort_keys=False))
print("Baseline impostata:", baseline)
PY
```

Rifai la baseline per ogni rotta e quando cambiano rigidezza, tool o cella.
Corregge le forze entro 30 mm dalle pose registrate, non i momenti.

## T4 — inserimento con il pezzo

Con board fissata e pezzo orientato come nella sua figura:

```bash
python -m kinova_dual_arm_dmp_insertion.main insert "$W" --arm "$R" --config "$CFG" --slow 3
```

Segui le richieste Enter: presenta il pezzo quando richiesto e lascialo soltanto
alla seconda richiesta. Questo usa il nuovo SOCP con la stretta scalare; `--arm`
seleziona la cartella delle rotte. Per aggiungere il twist, dopo la verifica dei
momenti, usa il passo 7 di [AVVIO_FULL_WRENCH.md](AVVIO_FULL_WRENCH.md).

Ctrl-C interrompe lo streaming; i controller tengono l'ultimo target. L'E-stop
è l'arresto del robot. Tieni T3 in registrazione fino a fine prova.

Per cambiare esempio: libera il pezzo, cambia `W` in **T4**, poi riparti dalla
creazione del profilo e ripeti controlli, giri a vuoto e baseline. T1/T2/T3
restano aperti; T5 deve rileggere `source config/session_varieta.env` ogni volta.

## Log e rigenerazione delle figure

| Dato | Dove |
|---|---|
| Giri a vuoto | `logs/follow2_<mondo>_<data_ora>.csv` |
| Test dei momenti | `logs/verifica_momenti_<mondo>_<data_ora>.csv`, indicato da `$MOMLOG` |
| Baseline | il CSV indicato da `$BASELINE` |
| Inserimento | `logs/<mondo>_c100_narrow75_<data_ora>.csv` |
| Sessione completa | `$LORENZO/bags/varieta_col2_5_*` |

Misura a mano l'errore peggiore di un cubo rispetto allo slot: il robot non lo vede.

| Data | Mondo | Gradino | Esito | Errore [mm] | Note / rumore |
|---|---|---|---|---|---|
| | | | | | |

Per rigenerare le figure con le misure aggiornate, **T4**, senza movimento:

```bash
python -m kinova_dual_arm_dmp_insertion.board T_s7039m4 T_s7037m3 J_s7109m3 S_s7073m3 O_s7094m3 --routes c100_narrow75 --out docs/boards/col2_5
```

## Verifica locale del 20/10/2026

Verificati per tutti i cinque mondi: slot interamente nelle colonne 2–5,
assenza di pezzi fisici duplicati nel kit, `main check --slow 3` e
`follow_two_arms --cell --dry` a gap 0.20 e 0.10. Tutti i comandi terminano
correttamente. Il controllo completo raggiunge al massimo circa 772 mm dalla
spalla con la configurazione attuale. Nessuna prova fisica è stata eseguita.

| Mondo | SOCP ok / tick in `check` | Massima distanza in `check`, sinistro / destro [mm] |
|---|---|---|
| `T_s7039m4` | 368 / 368 | 772 / 771 |
| `T_s7037m3` | 533 / 533 | 770 / 769 |
| `J_s7109m3` | 351 / 351 | 772 / 771 |
| `S_s7073m3` | 345 / 345 | 772 / 771 |
| `O_s7094m3` | 360 / 360 | 772 / 770 |

Verifica eseguita su Windows con Python 3.12, NumPy 2.5.4, SciPy 1.18.1,
CVXPY 1.9.3 e Clarabel 0.11.1, in dipendenze temporanee separate. Il venv del
PC robot resta quello esistente: ripeti i comandi `check` sul PC robot prima
dei movimenti; questi risultati locali non verificano le sue dipendenze ROS.
