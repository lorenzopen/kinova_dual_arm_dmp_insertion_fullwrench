# The squeeze loop: why it oscillates, and what `--squeeze-gain` is

Analysis of the 22/10/2026 Z runs (`F75_Zt1_s9012`, `c100_five_cut800`, `--twist`): 15:28 (`cart_d` 5/1,
gain 1), 17:22 (60/7, gain 1), 17:34 (60/7, gain 0.5), 17:44 (60/7, gain 0.25), 18:16 (60/7, gain 0.01),
18:27 (60/7, gain 0.1). Logs in `docs/runs/20261022_F75_Zt1_s9012_*`. Numbers from the logs are marked
*measured*; the rest is the model.

## 1. Two nested loops

```
 SOCP (10 Hz) ──F_ref──▶ squeeze integrator (50 Hz, ours) ──ds──▶ Kinova Cartesian impedance (~280-400 Hz)
                              ▲                                            │ hand moves, spring compresses
                              └──────────────── F (both ATIs, 200 Hz) ◀────┘
```

**Inner loop (Kinova controller, `kinova_controller.py`):** for each arm, a Cartesian impedance law

$$\tau = J^\top\big(K\,e - D\,\dot x\big) - B\,\dot q + g(q) + \tau_{fric}(\dot q),\qquad e = x_d - x$$

with $K = \mathrm{diag}(1000,1000,1000,20,20,20)$ and $D = $ `cart_d` (5/1 before, 60/7 now).
It makes the flange behave like a mass on a spring $K$ and damper $D$ pulled toward the target $x_d$.

**Outer loop (ours, `controller.py`, `_control`):** it moves each hand's target along the pad axis by
$d_s$ so that the measured squeeze matches the SOCP's request.

## 2. The plant: from $d_s$ to force

Along the pad axis, one hand pressing on the piece through the tool spring (contact stiffness $k_c$):

$$m\,\ddot x + D\,\dot x + K\,x + k_c\,x = K\,d_s, \qquad f = k_c\,x$$

$$P_1(s) = \frac{f}{d_s} = \frac{K\,k_c}{m s^2 + D s + K + k_c}$$

The squeeze is measured as the sum of both hands, and the two hands move symmetrically. So

$$P(s) = \frac{F}{d_s} = 2\,P_1(s), \qquad P(0) = 2\,k_{eff},\quad k_{eff} = \frac{K\,k_c}{K + k_c}$$

With $K = 1000$ N/m and $k_c \approx 2700$ N/m (the free spring): $k_{eff} \approx 730$ N/m and
$P(0) \approx 1460$ N/m. The static position ratio is $x/d_s = K/(K+k_c) \approx 0.27$.
*Measured* (17:22): measured/commanded separation ≈ 0.26–0.32. ✓

With $m \approx 3$–4 kg (estimated), the second-order term $m s^2 + D s$ adds only a few degrees of
phase below 2 Hz. **This linear model therefore predicts almost no lag between $d_s$ and $F$ at 1–2 Hz.**
Section 5 shows the robot does not behave like that.

## 3. The controller: a pure discrete integrator

Every $T_s = 0.02$ s (`CONTROL_PERIOD`):

$$e_k = F_{ref} - F_k, \qquad d_{s,k+1} = \mathrm{clip}\Big(d_{s,k} + \mathrm{clip}(\kappa\,e_k,\ \pm\Delta_{max}),\ [-0.012,\ 0.060]\ \mathrm{m}\Big)$$

$$\kappa = g\cdot\mathrm{GAIN},\qquad \mathrm{GAIN} = \frac{1}{4\cdot 1500} = 1.667\times10^{-4}\ \mathrm{m/N},\qquad \Delta_{max} = 2\times10^{-4}\ \mathrm{m}$$

$g$ is `--squeeze-gain`. In $z$ and in continuous time:

$$C(z) = \frac{\kappa\,z^{-1}}{1 - z^{-1}} \;\approx\; \frac{K_i}{s}\,e^{-sT_s/2},\qquad K_i = \frac{\kappa}{T_s} = g\cdot 8.33\times10^{-3}\ \frac{\mathrm{m}}{\mathrm{N\,s}}$$

There is no P term and no D term: only the I term. Its two nonlinearities are the slew limit $\Delta_{max}$
(it saturates when $|e| > \Delta_{max}/\kappa = 1.2/g$ N) and the clamp on $d_s$ (anti-windup).

## 4. The open loop and the crossover

$$L(s) = C(s)\,P(s)\,e^{-s\tau}$$

$\tau$ is every pure delay in the loop: ATI mean and topic (≈ 5 ms), the 100 Hz tick, and half the 20 ms
update. Below the arm's resonance $P \approx P(0)$, so

$$|L(j\omega)| \approx \frac{K_i\,P(0)}{\omega} \quad\Rightarrow\quad \omega_c \approx K_i\,P(0) = g\cdot 8.33\times10^{-3}\cdot 1460 \approx 12.2\,g\ \mathrm{rad/s}$$

| $g$ | $K_i$ [mm/(N·s)] | $\omega_c$ [rad/s] | $f_c$ [Hz] |
|---|---|---|---|
| 1 | 8.3 | 12.2 | 1.9 |
| 0.5 | 4.2 | 6.1 | 0.97 |
| 0.25 | 2.1 | 3.0 | 0.49 |
| 0.1 | 0.83 | 1.2 | 0.19 |

## 5. Why a sine: the phase condition

A loop with negative feedback oscillates when, at some frequency $\omega_{180}$,

$$\angle L(j\omega_{180}) = -180^\circ \quad\text{and}\quad |L(j\omega_{180})| \ge 1.$$

Below that, the loop amplifies its own error at $\omega_{180}$ and it grows until a nonlinearity (here the slew
limit $\Delta_{max}$) caps the amplitude. The result is a steady **limit cycle at $\omega_{180}$**, which looks
like a sine because everything above $\omega_{180}$ is filtered out.

The phase budget:

$$\angle L = \underbrace{-90^\circ}_{\text{integrator}} \;-\; \underbrace{\omega T_s/2}_{\text{hold}} \;-\; \underbrace{\omega\,\tau}_{\text{delays}} \;+\; \angle P(j\omega)$$

The integrator alone already uses up 90°. The oscillation needs another 90° from the plant and the delays.

**Measured, not modelled:** the controller logs both the commanded flange target and the measured flange,
so the arm's response along the pad axis can be read off directly. Band-pass 0.8–2.5 Hz, route phase:

| run | measured separation lags the commanded one by | amplitude ratio | force lags the measured separation by |
|---|---|---|---|
| 15:28, D 5 | 150 ms | 0.32 | ≈ 0 ms |
| 17:22, D 60 | 180 ms | 0.26 | ≈ 0 ms |
| 17:34, D 60 | 175 ms | 0.41 | ≈ 0 ms |

In the same runs, during the route motion along x and z (free directions, large motion) the measured
flange lags the commanded one by only ≈ 5 ms. So:

* the force follows the flange instantly, so the contact behaves as a pure spring, as modelled;
* the amplitude ratio is close to the static spring ratio of 0.27;
* **but along the pad axis, in contact, the flange follows its target ≈ 150–180 ms late.** That is far more
  than the linear impedance model gives (a few ms).

An effective lag $\tau_a$ that leaves the amplitude roughly unchanged acts like a pure delay. Adding
everything up:

$$-90^\circ - \omega\,(\tau_a + \tau + T_s/2) = -180^\circ
\quad\Rightarrow\quad
f_{180} = \frac{1}{4\,(\tau_a + \tau + T_s/2)}$$

| | $\tau_a$ (measured) | $\tau + T_s/2$ (≈) | predicted $f_{180}$ | observed peak |
|---|---|---|---|---|
| D 5 (15:28) | 150 ms | 15 ms | **1.52 Hz** | **1.53 Hz** |
| D 60 (17:22) | 180 ms | 15 ms | **1.28 Hz** | **1.23 Hz** |

So higher damping made the arm slower ($\tau_a$ up), and the frequency dropped exactly as predicted.

**The gain at $f_{180}$ (corrected 22/10 20:30).** A first version of this section took $|P| = F/d_s$ from the
closed-loop logs and found $|L(j\omega_{180})| \approx 1.4\,g$. That is circular: in a sustained oscillation the
loop *forces* $|L| \approx 1$ at the oscillation frequency, so any ratio of two closed-loop signals there gives
back ≈ 1, whatever $g$ is. Indeed, with the ratio built from measured quantities only
($|P| = k_c^{band}\cdot 2\,|x/d_s|$, contact stiffness and arm ratio both measured), every run gives
$K_i|P|/\omega \approx 1.2$–1.6, the gain-0.5 runs included:

| run | $g$ | K | $k_c$ in band (F / measured separation) | measured / commanded separation | $K_i\lvert P\rvert/\omega$ |
|---|---|---|---|---|---|
| 15:28 | 1 | 1000 | 2590 N/m | 0.32 | 1.25 |
| 17:22 | 1 | 1000 | 2690 N/m | 0.26 | 1.57 |
| 17:34 | 0.5 | 1000 | 2630 N/m | 0.41 | 1.23 |
| 20:20 | 0.5 | 1500 | 2530 N/m | 0.48 | 1.39 |

What this does tell:

* at 1–2 Hz the contact is stiff, $k_c \approx 2600$ N/m (the tool spring, 2700), while the **static** stiffness
  measured from the mean squeeze and compression is ≈ 950 N/m (section 7, `insert_open`): the pad / sponge /
  polystyrene side creeps slowly, it is not a plain spring;
* the arm's ratio $|x/d_s|$ is **amplitude-dependent** (0.26–0.32 at gain 1, 0.41–0.48 at gain 0.5), so the arm
  is not linear at these sub-millimetre motions (friction). A loop with such an element settles at the amplitude
  where its describing-function gain is 1: lowering $g$ does not move a fixed margin, it **shrinks the amplitude**
  of the limit cycle.

The observed amplitude is the measurable effect of $g$ (peak of the squeeze spectrum, same Z):

| $g$ | peak, sum of both hands (log) | peak per hand (bag) |
|---|---|---|
| 1 | 0.44–0.63 N | 0.24–0.33 N |
| 0.5 | 0.23 N | 0.10 N (K 1500) |
| 0.25 | 0.18 N | – |
| 0.1 | – | 0.06 N |
| 0.01 / feedforward | – | 0.04–0.07 N (noise floor) |

**The explanation:** an integrator (−90°) in series with an arm that reacts ≈ 0.16 s late along the pad axis
(another −90° at ≈ 1.5 Hz) sets the frequency; the arm's friction-dominated, amplitude-dependent response sets
how large the cycle grows, and $g$ scales it down. The sine is not caused by the update
rate. 50 Hz is about 35 updates per period, and the update rate only contributes $T_s/2$ to the lag.

## 6. Where the 150–180 ms come from (hypothesis, not verified)

They appear only along the pad axis in contact, where the commanded motions are sub-millimetre
($d_s$ moves 0.1–1 mm). In the free directions, with large motions, there is no such lag. That pattern fits
**joint friction / stiction in the Gen3 (harmonic drives)**: at sub-millimetre motions the impedance force
$K\,\Delta d_s \approx 1000 \cdot 0.5\,\mathrm{mm} = 0.5$ N is of the order of the breakaway, so the arm creeps
after its target instead of following it. The friction feed-forward does not help there: it is a function of
$\dot q$, and $\dot q \approx 0$ at those motions. Higher `cart_d` only adds more lag.

To verify: command a 1–2 Hz sine of 0.5 mm on $d_s$ with the piece clamped and the outer loop off (gain 0),
and measure gain and phase from $d_s$ to the measured flange and to $F$. That is an open-loop identification.

**A pitfall:** fitting $F/d_s$ from the closed-loop logs does *not* identify the plant. Because $d_s$ is computed
from $F$ by a deterministic controller, the ratio returns $-1/C(j\omega)$ (magnitude ∝ $\omega/g$, phase ≈ −90°)
whatever the arm does. The command-vs-measured-flange ratio above avoids this, because the flange position
is measured, not computed.

## 7. What can be done

1. **Lower $K_i$** (`--squeeze-gain`, default 0.5 since 22/10). This shrinks the limit cycle (table above).
   The cost is a slower recovery when the force drops: with $\omega_c$ the time constant is
   $1/\omega_c$, which is 0.16 s at $g = 0.5$ and 0.33 s at $g = 0.25$ (that is why the 0.25 run lost the squeeze
   at the end).
2. **Feed-forward:** set $d_s \approx F_{ref}/P(0)$ directly and let the integrator only trim the remainder.
   The integrator can then stay slow without losing responsiveness.
3. **Phase lead / P term:** $C = K_p + K_i/s$ has a zero at $K_i/K_p$ and gives back up to +90° above it. The
   cost is ATI noise passed straight to the hands. It is only useful if $K_p\,|P| < 1$ around 1.5 Hz.
4. **Reduce the lag $\tau_a$ itself:** less `cart_d` (60 made it worse than 5), or better friction compensation
   at low speed. The 20 ms → 10 ms update only gains $T_s/2 = 5$ ms out of ≈ 170.
