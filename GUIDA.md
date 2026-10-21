# Guida unica: due Kinova + ATI, inserimento DMP (21/10/2026)

Unica guida operativa. Le precedenti (AVVIO_FULL_WRENCH, ESEMPI_*, SETUP_PC_ROBOT, HANDOFF)
sono state rimosse: tutto quello che serviva è qui.

**In breve**
- `follow_two_arms` = solo il percorso a mani vuote (nessun SOCP, nessuna forza). **L'inserimento vero è sempre `main insert`.**
- `insert` = SOCP + stretta scalare. `insert --twist` = **full wrench**: stretta + twist opposto attorno all'asse dei pad,
  chiuso sugli ATI. Senza ATI su entrambi i bracci `--twist` si rifiuta di partire.
- Con gli ATI **non serve più la baseline del wrench**: la forza arriva dai sensori, non dalle coppie dei giunti.
- Le rotte DMP sono **2D**: piano x-z dello scaffale (y costante), una sola rotazione, attorno all'asse dei pad (y).
  Il twist del full wrench è proprio attorno a quell'asse.
- Per ogni rotta: prima `insert` (scalare), poi `insert --twist`, stessa board e stesso pezzo: due dati confrontabili.

**Dati salvati a ogni prova** (`logs/`, e il rosbag in `~/lorenzo/bags/`):
- `<W>_<R>_<data>.csv`, 100 Hz: comandi; pose misurate; forze e momenti usati dal controllo;
  **ATI grezzo** (assi del sensore, N/Nm); **stima Kinova grezza**; offset del twist; wrench chiesto dal SOCP;
  `t_wall` per allinearlo al bag.
- `<W>_<R>_<data>_meta.json`: config usata (con `local.yaml`), comando, commit git, unità e seriale degli ATI,
  bias e rumore di ogni tare, esito.
- `follow2_<W>_<data>.csv`: giri a vuoto, con l'ATI grezzo.
- Il rosbag: tutto `/left_arm/*` e `/right_arm/*`, cioè joint states, target, stima Kinova e ATI a 1 kHz.

---

## 0. Una volta sola sul PC robot (se già fatto, salta)

Terminale **normale**. Crea `~/lorenzo/env.sh` (cartella separata: il PC è condiviso), venv e copia del codice:

```bash
mkdir -p ~/lorenzo/home ~/lorenzo/bags && cat > ~/lorenzo/env.sh <<'END'
LORENZO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOME="$LORENZO/home"
export HISTFILE="$LORENZO/bash_history"
export ROS_LOG_DIR="$LORENZO/ros_log"
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash
[ -d "$LORENZO/venv" ] && source "$LORENZO/venv/bin/activate"
PS1="(lorenzo) $PS1"
END
```

```bash
source ~/lorenzo/env.sh && python3 -m venv --system-site-packages "$LORENZO/venv" && source "$LORENZO/venv/bin/activate" && pip install "cvxpy<1.7" clarabel "numpy<2" && cd "$LORENZO" && git clone https://github.com/lorenzopen/kinova_dual_arm_dmp_insertion_fullwrench.git
```

Due controller del laboratorio, con compiti diversi. Il **vecchio**, in `~/ros2_ws` (`kinova_controller_pkgs`), serve
**solo per portare i bracci in home**. L'**impedenza** la fa quello di **`tele_ros2`**, in `/home/nvidia/tele_ros2`,
avviato come in `dual_arm_bringup/connect.launch.py` del collega: senza home (`--skip-home`) e con
`publish_thread:=true`, che lo porta a 1 kHz (il vecchio gira a circa 3 ms). Gli **ATI** usano `ft_raw` di tele_ros2.
I due workspace hanno pacchetti `kinova_*` con lo stesso nome: **mai tutti e due nello stesso terminale**.
Verifica che siano compilati:

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 pkg prefix kinova_control
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/tele_ros2/install/setup.bash && ros2 pkg prefix kinova_control && ros2 pkg prefix ati_ft_tools
```

**Perché la home non la fa il controller di tele_ros2.** Il 21/10, con `kinova_arms.launch.py home:=true`, è
successo due volte su due: il .200 è andato in fault entrando in coppia subito dopo l'azione Home, e ha dato uno
scatto; il .107 si è fermato leggendo l'azione Home (`DATABASE_ERROR`). `connect.launch.py` usa lo stesso
controller sempre senza home.

---

## 1. Ogni sessione

Terminali normali, **senza** `env.sh` (i controller usano `kortex_api` di `/home/nvidia/.local`).
Mano sull'E-stop e niente sotto le punte. **Un braccio alla volta.**

**T0, home con il controller vecchio** (un braccio alla volta). Il braccio va in home e poi resta in impedenza:
quando è fermo in home, **Ctrl-C**. In uscita il controller rimette il braccio in modalità posizione, e lì resta.

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.107 --ros-args -r __ns:=/right_arm -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=/home/nvidia/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/urdf/gen3_ati_tool.urdf -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

Poi lo stesso con `--ip 192.180.0.200` e `__ns:=/left_arm`. Se il .107 dà ancora `DATABASE_ERROR`, fai la home
dalla web app del Kinova (`http://192.180.0.107`); se succede anche lì, riavvia il braccio.

**T1 e T1b, impedenza con il controller di tele_ros2** (un braccio alla volta). Parte dalla posa in cui si trova,
senza muoversi: avvia T1, controlla per una decina di secondi che resti fermo e che nel log **non** compaia
`Servoing-mode mismatch`, poi avvia T1b. Se compare, chiudi subito con Ctrl-C.

T1, destro (.107):

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/tele_ros2/install/setup.bash && N=right_arm && RM=$(for t in joint_states target_pose joint_cmd cartesian_gains joint_gains switch_control_mode estimated_wrench zero_wrench clear_wrench_offset current_pose_viz target_pose_viz joint_target_viz control_diagnostics control_gains commanded_wrench commanded_twist cmd_vel joint_torque_ff gripper_cmd; do echo -n " -r /$t:=/$N/$t"; done) && ros2 run kinova_control kinova_controller.py --ip 192.180.0.107 --skip-home --ros-args -r __node:=real_robot_impedance_node -r __ns:=/$N $RM -p viz_frame_prefix:=$N/ -p initial_mode:=cartesian -p ee_frame:=end_effector_link -p urdf_path:=/home/nvidia/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/urdf/gen3_ati_tool.urdf -p publish_thread:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

T1b, sinistro (.200): lo stesso comando con `N=left_arm` e `--ip 192.180.0.200`.

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/tele_ros2/install/setup.bash && N=left_arm && RM=$(for t in joint_states target_pose joint_cmd cartesian_gains joint_gains switch_control_mode estimated_wrench zero_wrench clear_wrench_offset current_pose_viz target_pose_viz joint_target_viz control_diagnostics control_gains commanded_wrench commanded_twist cmd_vel joint_torque_ff gripper_cmd; do echo -n " -r /$t:=/$N/$t"; done) && ros2 run kinova_control kinova_controller.py --ip 192.180.0.200 --skip-home --ros-args -r __node:=real_robot_impedance_node -r __ns:=/$N $RM -p viz_frame_prefix:=$N/ -p initial_mode:=cartesian -p ee_frame:=end_effector_link -p urdf_path:=/home/nvidia/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/urdf/gen3_ati_tool.urdf -p publish_thread:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

Il controller di tele_ros2 scrive i topic con nomi assoluti, quindi li rimappiamo nel namespace del braccio (`RM`).
L'URDF è **`urdf/gen3_ati_tool.urdf`** di questo repo: `gen3.urdf` più quello che è montato sulla flangia (ATI,
adattatori, molla e tri-pad), **150 g** in tutto. Senza quella massa la compensazione di gravità non vede il tool.
Se cambi tool, aggiorna massa e baricentro in quel file. `end_effector_link` **coincide con la flangia**: l'impedenza
è alla flangia, come nell'addestramento. Attrito e Kalman restano spenti, come in `connect.launch.py`.

**T1c, i due ATI** (terminale normale, **senza** `env.sh`):

```bash
source /opt/ros/jazzy/setup.bash && source /home/nvidia/tele_ros2/install/setup.bash && ros2 launch ati_ft_tools ati_ft_sensor.launch.py ns_1:=left_arm ip_address_1:=192.180.0.132 frame_id_1:=left_arm/tool_frame configure_sensor_1:=false ns_2:=right_arm ip_address_2:=192.180.0.131 frame_id_2:=right_arm/tool_frame configure_sensor_2:=false
```

I sensori sono montati **al contrario** rispetto alla mappa di tele_ros2: sul sinistro (.200) c'è il **.132**, sul destro
(.107) il **.131**. All'avvio ogni `ft_raw` stampa seriale e unità: sinistro **FT54940**, destro **FT49595**, in N / Nm
(se un NetBox fosse in lbf, il nostro codice legge le unità e converte da solo). `configure_sensor:=false` è voluto:
i NetBox sono già configurati e la configurazione via HTTP li blocca.

**Una volta sola: la velocità del .132.** `ft_raw` vuole 1000 pacchetti/s. Il 21/10 il .131 era a 1000 e
il .132 a **7000**, con il segnale in ritardo e a scalini. Controlla e, se serve, correggi: l'impostazione
resta salvata nel NetBox.

```bash
curl -s http://192.180.0.132/netftapi2.xml | grep -o "<comrdtrate>[0-9]*"
curl -s "http://192.180.0.132/comm.cgi?comrdtrate=1000" >/dev/null
curl -s http://192.180.0.132/netftapi2.xml | grep -o "<comrdtrate>[0-9]*"
```

**T2, registrazione** (lasciala andare tutta la sessione, Ctrl-C alla fine):

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/bags" && ros2 bag record -e "/(left_arm|right_arm)/.*|/rosout" -o sessione_$(date +%Y%m%d_%H%M%S)
```

**T3, lavoro** (aggiorna il codice e scegli la rotta, tabella in fondo):

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench" && git pull --ff-only && mkdir -p logs
```

```bash
export W=F75_Tt1_s9007 R=c100_five_cut800
```

```bash
ros2 topic hz /left_arm/ft_data_raw
```

Ripeti per `/right_arm/ft_data_raw` (circa 1000 Hz) e per `/left_arm/joint_states` e `/right_arm/joint_states`
(circa 250 Hz). Ctrl-C dopo qualche secondo. Tutti i terminali devono avere lo stesso `echo $ROS_DOMAIN_ID`.

---

## 2. Solo la prima volta con gli ATI montati (T3, bracci fermi in home)

**a. Misure a mano**, da salvare (in `config/local.yaml`, fuori da git; ogni comando la legge):
- pesa ciò che sta **oltre** ogni sensore: adattatore, molla e tri-pad, in kg. Molla più tri-pad pesano 40 g,
  che è già il valore in `cell.yaml`; manca l'adattatore lato tool;
- dalla flangia del Kinova al fronte delle sfere ci sono **100 mm** su entrambi i bracci (21/10). Quindi
  `pad_x` = 0.100 − 0.015 = **0.085**, già in `cell.yaml`. Rimisuralo solo se cambi il tool.

```bash
python -m kinova_dual_arm_dmp_insertion.ft_check --tool-mass 0.050 0.050 --pad-x 0.085 --save
```

(sostituisci le due masse con quelle pesate: sinistra, destra).

**b. Orientamento e segno dei sensori:** tre prese a mano per braccio, ciascuna mantenuta per circa 2 s.
Nell'ordine: niente contatto; premi la faccia del tri-pad verso la flangia; premi il tri-pad **verso il basso,
sulla parte davanti**.

```bash
python -m kinova_dual_arm_dmp_insertion.ft_check --calibrate --save
```

Salva `ft_in_ee` e `ft_sign` e ti avvisa se i due ATI sono scambiati.

**c. Verifica dal vivo** (stesso codice di `insert`):

```bash
python -m kinova_dual_arm_dmp_insertion.ft_check
```

Su ciascun braccio: premendo la faccia del pad **fn > 0**; premendo verso il basso **F z > 0**;
ruotando il pad **in senso orario visto dalla faccia** **Tn > 0**; a riposo, tutto vicino a 0.
Ogni pressione deve comparire **solo sul suo braccio**. Il tare stampa il rumore (sd a 1 kHz, senza filtro): qualche centesimo di N va bene; sopra 0,2 N controlla cavi e fissaggio.
Rifai a, b, c solo se smonti o ruoti un sensore o un tool.

---

## 3. Per ogni rotta (T3), un comando alla volta

Board e pezzi: figure e quote nell'appendice. Mano sull'E-stop: non c'è controllo collisioni fra gli avambracci.

```bash
python -m kinova_dual_arm_dmp_insertion.main check "$W" --arm "$R" --slow 3
```

Nessun movimento: deve stampare `SOCP ok N/N`, `seat_found True` e il `cart_k` del launch in T1.

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --cell --gap 0.20 --lift 0.10
```

In aria, **senza board**, punte a 20 cm. **Ometti `--lift 0.10`** dove la tabella dice "no lift".

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --cell --gap 0.20
```

Quota vera, **board montata**, mani vuote.

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms "$W" --routes "$R" --cell --gap 0.10
```

Distanza di presa, ancora **senza pezzo**. Stampa l'errore di inseguimento: deve stare entro pochi mm.

```bash
python -m kinova_dual_arm_dmp_insertion.main insert "$W" --arm "$R" --slow 3
```

**Inserimento vero, stretta scalare.** Due Enter: tieni il pezzo fra i pad come in figura, poi lascialo.
Misura a mano l'errore di piazzamento (cella peggiore rispetto allo slot) e annotalo.

```bash
python -m kinova_dual_arm_dmp_insertion.main insert "$W" --arm "$R" --slow 3 --twist
```

**Inserimento full wrench** (stretta + twist): stessa board, pezzo rimesso al punto di partenza.

Ctrl-C ferma lo streaming e i bracci tengono l'ultimo target; l'arresto del robot resta l'E-stop.
Il codice si ferma da solo sopra 60 N per mano o con 4 cm di errore.

**Fine sessione:** Ctrl-C in T2 (bag), poi T1. Dal **PC Windows** (PowerShell) copia i dati:

```powershell
$d = "C:\Users\Lorenzo\Desktop\dati_kinova_$(Get-Date -Format yyyyMMdd)"; New-Item -ItemType Directory -Force $d | Out-Null; scp -r nvidia@<IP_PC_ROBOT>:~/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/logs nvidia@<IP_PC_ROBOT>:~/lorenzo/bags $d
```

---

## 4. Tutte le rotte

| # | Pezzo | `W` | `R` | Primo giro | Vicini da montare |
|---|---|---|---|---|---|
| 1 | T | `F75_Tt1_s9007` | `c100_five_cut800` | `--lift 0.10` | I, O, Z, L |
| 2 | O | `F75_Ot0_s9005` | `c100_five_cut800` | no lift | L, I, T, Z |
| 3 | L | `F75_Lt1_s9014` | `c100_five_cut800` | `--lift 0.10` | O, T, Z, I |
| 4 | Z | `F75_Zt1_s9012` | `c100_five_cut800` | `--lift 0.10` | I, T, O, L |
| 5 | I | `F75_It1_s9001` | **`c100_five_cut700`** | no lift | L, T, S (= Z girata), O |
| 6 | T barra in alto | `T_s7039m4` | `c100_narrow75` | no lift | J (= L girata), I, S |
| 7 | T capovolta | `T_s7037m3` | `c100_narrow75` | no lift | L, O |
| 8 | J orizzontale | `J_s7109m3` | `c100_narrow75` | no lift | Z, I |
| 9 | S orizzontale | `S_s7073m3` | `c100_narrow75` | no lift | O, J |
| 10 | O, due vicini | `O_s7094m3` | `c100_narrow75` | no lift | Z, I |

Inizia dalla 1 (T, già inserita il 16/10). La 4 (Z) è la più delicata delle prime cinque; la 9 (S) è vicina
alla soglia PyBullet: lasciala per ultima. Si cambia rotta solo con la riga `export W=... R=...` in T3.

Altre rotte esportate: `python -m kinova_dual_arm_dmp_insertion.main list`. Non sono provate sui robot.

---

## Appendice: board

**Cella** (già in `config/cell.yaml`): basi parallele, sinistra .200 a y = +600 mm dalla destra .107;
pavimento dello scaffale da x = 175 a 825 mm davanti alle basi, 30 mm sotto di esse, profondo 125 mm.
Origine (colonna 0) a x = 807,5 mm: le colonne crescono **verso le basi**. Se sposti scaffale o bracci,
correggi `shelf_in_base`.

Quote in mm nel frame dello scaffale: x dal bordo sinistro della colonna 0, z dal pavimento.
Le figure sono **viste dal braccio sinistro** (.200). Il pezzo pieno in alto mostra come presentarlo
fra le punte, già inclinato; la **X** è il punto di pressione delle punte; il tratteggio è lo slot.
I vicini sono fuori griglia, entrano di circa 12-18 mm nello slot: **fissali con biadesivo**.
J = L girata sul retro, S = Z girata sul retro. Monta solo i vicini elencati.

**1. T `F75_Tt1_s9007`**: ![](docs/boards/F75_Tt1_s9007.png)
I 5–405 / 0–100 · O 412–612 / 0–200 · Z 293–493 / 103–403 · L 18–218 / 103–403 · **slot T 106–306 / 100–400**.
Inizio: baricentro x 231, z 800, ruotato 8° in senso orario; punte sul cubo centrale della barra.

**2. O `F75_Ot0_s9005`**: ![](docs/boards/F75_Ot0_s9005.png)
L 293–493 / 0–300 · I 18–118 / 0–400 · T 397–599 / 0–300 · Z 395–599 / 205–504 · **slot O 105–305 / 0–200**.
Inizio: x 205, z 800, dritto; punte sul cubo in alto a sinistra. No lift: con +10 cm il braccio supera la portata.

**3. L `F75_Lt1_s9014`**: ![](docs/boards/F75_Lt1_s9014.png)
O 223–423 / 0–200 · T 2–202 / 0–300 · Z 19–319 / 203–403 · I 498–598 / 0–400 · **slot L 311–511 / 0–300**.
Inizio: x 436, z 800, ruotato 3° in senso orario; punte sul cubo d'angolo, in alto.

**4. Z `F75_Zt1_s9012`**: ![](docs/boards/F75_Zt1_s9012.png)
I 5–405 / 0–100 · T 397–597 / 0–300 · O 3–203 / 103–303 · L 123–323 / 104–405 · **slot Z 310–510 / 100–400**.
Inizio: x 410, z 800, ruotato 3° in senso antiorario; punte sul cubo centrale in alto a destra.

**5. I `F75_It1_s9001`**: ![](docs/boards/F75_It1_s9001.png)
L 16–216 / 0–300 · T 311–611 / 0–200 · S 191–491 / 103–303 · O 292–493 / 305–505 · **slot I 104–204 / 100–500**.
Inizio: x 154, z 700, ruotato 38° in senso antiorario; punte sul secondo cubo dall'alto. Usa `cut700`
perché con `cut800` supererebbe la portata.

**6. T barra in alto `T_s7039m4`**: ![](docs/boards/col2_5/T_s7039m4.png)
J 19–319 / 1–201 · I 497–597 / 1–401 · S 21–221 / 103–403 · **slot T 209–509 / 0–200**.
Inizio: dritta, baricentro x 359, z 890; punte sul cubo centrale della barra alta.

**7. T capovolta `T_s7037m3`**: ![](docs/boards/col2_5/T_s7037m3.png)
L 296–596 / 1–201 · O 21–221 / 1–201 · **slot T 209–509 / 100–300**.
Inizio: dritta, x 359, z 837; punte sul cubo singolo in alto.

**8. J orizzontale `J_s7109m3`**: ![](docs/boards/col2_5/J_s7109m3.png)
Z 21–321 / 1–201 · I 497–597 / 1–401 · **slot J 209–509 / 0–200**.
Inizio: dritta, x 384, z 890; barra da tre in alto, piede in basso a destra; punte sul cubo centrale della barra.

**9. S orizzontale `S_s7073m3`**: ![](docs/boards/col2_5/S_s7073m3.png)
O 21–221 / 1–201 · J 397–597 / 1–301 · **slot S 209–509 / 0–200**.
Inizio: dritta, x 359, z 865; coppia bassa a sinistra; punte sul cubo in alto a sinistra.

**10. O con due vicini `O_s7094m3`**: ![](docs/boards/col2_5/O_s7094m3.png)
Z 22–322 / 1–201 · I 498–598 / 1–401 · **slot O 310–510 / 0–200**.
Inizio: dritta, x 410, z 865; punte sul cubo in alto a sinistra.

Le prove reali del 16/10 (T riuscita, Z a metà) con i loro log: [docs/runs/](docs/runs/README.md).
Il metodo (SOCP full wrench, margini di attrito): [FULL_WRENCH.md](FULL_WRENCH.md), [docs/FRICTION_MARGIN.md](docs/FRICTION_MARGIN.md).
