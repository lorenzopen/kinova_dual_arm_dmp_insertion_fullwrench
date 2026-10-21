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
- Il rosbag: tutto `/left/*` e `/right/*`, cioè joint states, target, stima Kinova e ATI a 1 kHz.

---

## 0. Una volta sola sul PC robot (se già fatto, salta)

Terminale **normale**. Crea `~/lorenzo/env.sh`, venv e URDF senza gripper, come nelle sessioni precedenti:

```bash
mkdir -p ~/lorenzo/home ~/lorenzo/bags && cat > ~/lorenzo/env.sh <<'EOF'
LORENZO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOME="$LORENZO/home"
export HISTFILE="$LORENZO/bash_history"
export ROS_LOG_DIR="$LORENZO/ros_log"
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash
[ -d "$LORENZO/venv" ] && source "$LORENZO/venv/bin/activate"
PS1="(lorenzo) $PS1"
EOF
```

```bash
sed 's|<mass value="0.925" />|<mass value="0.0001" />|' ~/ros2_ws/src/kinova_controller_pkgs/kinova_control/urdfs/gen3_p2_F_only.urdf > ~/lorenzo/gen3_bare.urdf && grep -c 'mass value="0.0001"' ~/lorenzo/gen3_bare.urdf
```

```bash
source ~/lorenzo/env.sh && python3 -m venv --system-site-packages "$LORENZO/venv" && source "$LORENZO/venv/bin/activate" && pip install "cvxpy<1.7" clarabel "numpy<2" && cd "$LORENZO" && git clone https://github.com/lorenzopen/kinova_dual_arm_dmp_insertion_fullwrench.git
```

Il driver ATI è `ft_raw.py` di `tele_ros2`, già sul PC NVIDIA in `~/tele_ros2`. È puro Python e non va compilato.
Se è altrove, trovalo con `find ~ -name ft_raw.py -path "*ati_ft_tools*"` e correggi il percorso in T3.

---

## 1. Ogni sessione: 5 terminali

Lascia libera la zona: T1 e T2 portano i bracci in home. In tutti i terminali deve risultare lo stesso
`echo $ROS_DOMAIN_ID`.

**T1, braccio destro .107** (terminale normale, **senza** `env.sh`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.107 --ros-args -r __ns:=/right -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=/home/nvidia/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=/home/nvidia/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

**T2, braccio sinistro .200** (terminale normale, **senza** `env.sh`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.200 --ros-args -r __ns:=/left -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=/home/nvidia/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=/home/nvidia/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[1000.0, 1000.0, 1000.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

**T3, i due ATI** (terminale normale): sinistro NetBox `.131`, destro `.132`, nei namespace dei bracci.

```bash
source /opt/ros/jazzy/setup.bash && FT=/home/nvidia/tele_ros2/src/ati_ft_tools/ati_ft_tools/ft_raw.py && ping -c1 -W1 192.180.0.131 && ping -c1 -W1 192.180.0.132 && (python3 $FT --ros-args -r __ns:=/left -p ip_address:=192.180.0.131 -p frame_id:=left_ati -p configure_sensor:=false -p rdt_sampling_rate:=1000 & python3 $FT --ros-args -r __ns:=/right -p ip_address:=192.180.0.132 -p frame_id:=right_ati -p configure_sensor:=false -p rdt_sampling_rate:=1000 & wait)
```

Nel log di ciascuno devi trovare `Serial #` e `Receiving RDT packets`. Il `.131` è impostato in **lbf**:
il nostro codice legge le unità dal driver e converte da solo in N/Nm. `configure_sensor:=false` è voluto:
la NetBox è già configurata e la configurazione via HTTP la blocca. Ctrl-C in T3 chiude entrambi.

**T4, registrazione** (lasciala andare tutta la sessione, Ctrl-C alla fine):

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/bags" && ros2 bag record -e "/(left|right)/.*|/rosout" -o sessione_$(date +%Y%m%d_%H%M%S)
```

**T5, lavoro** (aggiorna il codice e scegli la rotta, tabella in fondo):

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench" && git pull --ff-only && mkdir -p logs
```

```bash
export W=F75_Tt1_s9007 R=c100_five_cut800
```

```bash
ros2 topic hz /left/ft_data_raw
```

Fai lo stesso per `/right/ft_data_raw`: circa 1000 Hz. Poi `/left/joint_states`: circa 250 Hz. Ctrl-C dopo qualche secondo.

---

## 2. Solo la prima volta con gli ATI montati (T5, bracci fermi in home)

**a. Misure a mano**, da salvare (in `config/local.yaml`, fuori da git; ogni comando la legge):
- pesa ciò che sta **oltre** ogni sensore: adattatore, molla e tri-pad, in kg;
- misura dalla flangia del Kinova al fronte delle sfere; `pad_x` = quella misura − 0.015 m.
  L'ATI e gli adattatori allungano il tool: prima era 0.070 − 0.015 = 0.055.

```bash
python -m kinova_dual_arm_dmp_insertion.ft_check --tool-mass 0.080 0.080 --pad-x 0.070 --save
```

(sostituisci i tre numeri con i tuoi: massa sinistra, massa destra, `pad_x`).

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

## 3. Per ogni rotta (T5), un comando alla volta

Board e pezzi: figure e quote nell'appendice. Mano sull'E-stop: non c'è controllo collisioni fra gli avambracci.

```bash
python -m kinova_dual_arm_dmp_insertion.main check "$W" --arm "$R" --slow 3
```

Nessun movimento: deve stampare `SOCP ok N/N`, `seat_found True` e il `cart_k` dei terminali T1/T2.

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

**Fine sessione:** Ctrl-C in T4 (bag), T3, T1, T2. Dal **PC Windows** (PowerShell) copia i dati:

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
alla soglia PyBullet: lasciala per ultima. Si cambia rotta solo con la riga `export W=... R=...` in T5.

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
