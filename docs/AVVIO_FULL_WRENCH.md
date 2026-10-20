# Kinova: avvio della copia full wrench (19/10/2026)

Per cinque mondi **diversi dagli esempi originali**, con orientamenti variati e slot
fra le colonne 2 e 5, usa [ESEMPI_VARIETA_COLONNE_2_5.md](ESEMPI_VARIETA_COLONNE_2_5.md):
include figure, quote e la sequenza completa T1–T5, a partire dalla T con barra in alto.

Comandi da incollare sul **PC robot Ubuntu/Jazzy**, salvo il trasferimento dal PC Windows.
Si parte dalla **T**, come nella prova riuscita del 16/10: `cart_k = [1000,1000,1000,20,20,20]`,
`cart_d = [5,5,5,1,1,1]`, `jam_scale = 1.8`, inserimento tre volte più lento.
Questa rigidezza è quella della prova reale; le rotte erano addestrate con
`[1500,1500,1500,8,40,8]`. Il nuovo SOCP e le rotte insieme sono ancora da verificare sui robot.

**Due cose diverse:** il nuovo `socp.py` è usato sempre in questa copia, anche con
`full_wrench: false`. Con `full_wrench: true` si aggiunge il controllo dei momenti:
con `full_wrench_channels: twist`, stretta + twist opposto attorno all'asse dei pad.
Per provare prima il nuovo SOCP usa i passi 0–6; per aggiungere il twist vedi il passo 7.
`follow_two_arms` verifica solo il percorso: non usa il SOCP.

## 0. Scaricare questa copia sul PC robot

La guida assume che `~/lorenzo/env.sh`, il venv e `~/lorenzo/gen3_bare.urdf` esistano
già dalla sessione precedente. Se mancano, vedi [SETUP_PC_ROBOT.md](../SETUP_PC_ROBOT.md).

La prima volta, sul **PC NVIDIA**, clona il repository separato (se privato,
usa il tuo accesso GitHub tramite HTTPS oppure SSH):

```bash
source ~/lorenzo/env.sh
cd "$LORENZO"
git clone https://github.com/lorenzopen/kinova_dual_arm_dmp_insertion_fullwrench.git
```

Le volte successive aggiorna questa copia, con gli script di movimento fermi:

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
git pull --ff-only
```

In alternativa, trasferimento da **PowerShell sul PC Windows**, sostituendo `<IP_PC_ROBOT>`:

```powershell
scp -r "C:\Users\Lorenzo\Desktop\kinova_dual_arm_dmp_insertion_fullwrench" nvidia@<IP_PC_ROBOT>:~/lorenzo/
```

La copia resta in `~/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench`, fuori da `ws/src`:
ha lo stesso nome Python/ROS del pacchetto vecchio. Non compilare le due copie insieme.
Esegui `python -m ...` dalla nuova cartella. Un `git pull` della cartella vecchia
non trasferisce queste modifiche.

## 1. T4 — ambiente, pezzo e configurazione della prova

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
export W=F75_Tt1_s9007
export R=c100_five_cut800
export CFG="config/run_${W}_k1000.yaml"
export BASELINE="logs/baseline_${W}_k1000_$(date +%Y%m%d_%H%M%S).csv"
mkdir -p logs
python -c 'import numpy, scipy, cvxpy, clarabel, yaml, rclpy, pinocchio; print("Dipendenze OK")'
python -c 'import kinova_dual_arm_dmp_insertion.socp as s; print(s.__file__)'
```

L'ultima riga deve indicare **questa cartella fullwrench**. Non serve `colcon build`.
Se manca cvxpy/clarabel, installali solo nel venv di Lorenzo:

```bash
python -m pip install "cvxpy<1.7" clarabel "numpy<2"
```

Crea una configurazione separata per questa prova: stesso URDF dei controller,
baseline inizialmente vuota, nuovo SOCP attivo, controllo dei momenti disabilitato.
**Esegui questa creazione una volta a inizio sessione**: ripeterla azzera la baseline del profilo.

```bash
python - "$LORENZO/gen3_bare.urdf" "$CFG" <<'PY'
import sys
from pathlib import Path
import yaml
urdf = Path(sys.argv[1]).resolve()
assert urdf.is_file(), f"URDF mancante: {urdf}"
cfg = yaml.safe_load(Path("config/cell.yaml").read_text())
for side in ("left", "right"):
    cfg[side]["urdf_path"] = str(urdf)
cfg["stiffness"] = {"trans": 1000.0, "torsion": 20.0, "tilt": 20.0}
cfg["socp"] = {"margin_mode": "task_adaptive", "margin_task_scale": 0.25,
               "margin_floor_N": 0.5, "margin_fixed_N": 0.5, "w_force": 0.0, "no_bending": False}
cfg["full_wrench"] = False
cfg["allow_hand_wrench"] = False
cfg["full_wrench_channels"] = "twist"
cfg["wrench_baseline"] = None
Path(sys.argv[2]).write_text(yaml.safe_dump(cfg, sort_keys=False))
print("Profilo:", sys.argv[2], "\nURDF:", urdf)
PY
```

Le misure dello scaffale, del tool e la massa vengono da `config/cell.yaml`.
Se la cella è cambiata, aggiornale prima di generare il profilo. Le quote e i disegni dei
cinque pezzi restano in [ESEMPI_5_PEZZI.md](ESEMPI_5_PEZZI.md).

## 2. T1 e T2 — controller dei due Kinova

Terminali **normali, senza `env.sh`**. I comandi includono il movimento in home:
lascia libera la zona prima di avviarli. Aspetta che entrambi siano in home e fermi.

**T1 — destro, .107:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.107 --ros-args -r __ns:=/right -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

**T2 — sinistro, .200:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.200 --ros-args -r __ns:=/left -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

## 3. T3 — registra tutta la sessione

```bash
source ~/lorenzo/env.sh
mkdir -p "$LORENZO/bags"
cd "$LORENZO/bags"
ros2 bag record -e "/(left|right)/.*" -o fullwrench_$(date +%Y%m%d_%H%M%S)
```

Ctrl-C in T3 chiude la registrazione a fine sessione.

## 4. T4 — controllo senza movimento, poi percorso a mani vuote

```bash
python -m kinova_dual_arm_dmp_insertion.main check "$W" --arm "$R" --config "$CFG" --slow 3
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.20 --dry
```

`check` esegue la sequenza senza ROS/robot e stampa `SOCP ok`, portata e rigidezza del profilo.
È un controllo software, non una simulazione fisica o una verifica delle collisioni.

Controlla i topic (Ctrl-C dopo qualche secondo per ogni comando):

```bash
ros2 topic hz /left/joint_states
ros2 topic hz /right/joint_states
ros2 topic hz /left/estimated_wrench
ros2 topic hz /right/estimated_wrench
```

Poi, **senza pezzo**, un comando alla volta, passando al successivo se il precedente va bene.
Primo giro senza board, poi board montata con i vicini fissati:

```bash
# In aria, senza board. SOLO T/L/Z: per O e I ometti --lift 0.10.
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.20 --lift 0.10
```

```bash
# Quota vera, board montata, mani vuote.
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.20
```

```bash
# Distanza di presa, ancora SENZA pezzo.
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.10
```

Questi script muovono davvero i bracci e chiedono Enter. Nessuna verifica automatica
delle collisioni fra avambracci. Tieni disponibile l'E-stop durante le prove.

## 5. Baseline nuova per questo pezzo e questa rigidezza

La configurazione originale punta a una vecchia baseline della Z. Il profilo creato
al passo 1 la esclude: registra ora quella del pezzo scelto, a mani vuote.

**T4**, prima del giro, bracci fermi e niente a contatto:

```bash
python -m kinova_dual_arm_dmp_insertion.wrench_log --config "$CFG" --tare --out "$BASELINE"
```

Aspetta il messaggio di logging dopo il tare di entrambi. **T5**, mentre T4 registra:

```bash
source ~/lorenzo/env.sh
cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
W=F75_Tt1_s9007 R=c100_five_cut800
CFG="config/run_${W}_k1000.yaml"
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --config "$CFG" --cell --gap 0.10
```

Se hai scelto un altro pezzo, usa anche in T5 gli stessi `W` e `R` di T4.
Finito il giro, **Ctrl-C solo al logger in T4**, poi in T4 associa il CSV al profilo:

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
    print(side, len(b.P), "campioni")
cfg = yaml.safe_load(p.read_text())
cfg["wrench_baseline"] = str(baseline)
p.write_text(yaml.safe_dump(cfg, sort_keys=False))
print("Baseline impostata:", baseline)
PY
```

Una baseline vale per quella rotta, quella rigidezza e quel tool; se cambiano, rifalla.
La correzione agisce sulle **forze**, entro 30 mm dalle pose registrate, **non sui momenti**.

## 6. T4 — inserimento con il nuovo SOCP

Board e pezzo disposti come nelle figure del mondo scelto, poi:

```bash
python -m kinova_dual_arm_dmp_insertion.main insert "$W" --arm "$R" --config "$CFG" --slow 3
```

Segui le due richieste Enter: presenta il pezzo fra i pad, poi lascialo quando richiesto.
Questo prova il **nuovo SOCP con la stretta scalare**. Non cambiare `--arm` in `hwd`:
`--arm "$R"` seleziona la cartella della rotta.

Il nuovo SOCP chiede circa 2–3 N nelle prove offline T/Z, meno del vecchio: anche la
soglia del jam può essere più stretta. Non è una misura già confermata sul Kinova.
Ctrl-C ferma lo streaming e i controller mantengono l'ultimo target; l'E-stop resta
l'arresto del robot. Log inserimento: `logs/${W}_${R}_*.csv`, oltre al rosbag in T3.

## 7. Aggiungere anche il twist: dopo aver verificato la stima dei momenti

Nel sistema attuale il wrench è stimato dalle coppie articolari. La baseline sopra
non corregge i momenti: prima di abilitarne l'anello verifica segno, assi e risposta
a una torsione nota; in alternativa usa un F/T configurato e verificato.
La sola prova `check` non verifica il tracking dei momenti.

Quando questa verifica è stata fatta, crea un secondo profilo dalla prova scalare:

```bash
export CFG_TWIST="${CFG%.yaml}_twist.yaml"
python - "$CFG" "$CFG_TWIST" <<'PY'
import sys
from pathlib import Path
import yaml
cfg = yaml.safe_load(Path(sys.argv[1]).read_text())
cfg["full_wrench"] = True
cfg["allow_hand_wrench"] = True
cfg["full_wrench_channels"] = "twist"
Path(sys.argv[2]).write_text(yaml.safe_dump(cfg, sort_keys=False))
print("Profilo twist:", sys.argv[2])
PY
python -m kinova_dual_arm_dmp_insertion.main check "$W" --arm "$R" --config "$CFG_TWIST" --slow 3
```

Poi l'inserimento con twist, mantenendo gli stessi controller:

```bash
python -m kinova_dual_arm_dmp_insertion.main insert "$W" --arm "$R" --config "$CFG_TWIST" --slow 3
```

## Gli altri quattro pezzi

Al passo 1 scegli la riga del pezzo. Ripeti la creazione del profilo, i controlli e la
baseline per quel mondo. Al passo 5 riporta gli stessi valori anche in T5.

| Pezzo | W | R | Primo giro con lift |
|---|---|---|---|
| T | `F75_Tt1_s9007` | `c100_five_cut800` | `--lift 0.10` |
| O | `F75_Ot0_s9005` | `c100_five_cut800` | senza `--lift` |
| L | `F75_Lt1_s9014` | `c100_five_cut800` | `--lift 0.10` |
| Z | `F75_Zt1_s9012` | `c100_five_cut800` | `--lift 0.10` |
| I | `F75_It1_s9001` | **`c100_five_cut700`** | senza `--lift` |

Per esempio, per la I sostituisci le assegnazioni iniziali con:

```bash
export W=F75_It1_s9001
export R=c100_five_cut700
```

Poi ricalcola `CFG` e `BASELINE` con le righe del passo 1. Figure, quote dei vicini,
presa e inclinazione iniziale: [ESEMPI_5_PEZZI.md](ESEMPI_5_PEZZI.md).
