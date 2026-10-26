# STATUS — sensorless FOC (Nucleo-F303RE)

Last updated: 2026-10-26 (end of the Claude Code session; development continues manually).
Git history starts on 2026-10-26; this file holds the record before that and the verification
status of each change (compile / sim / hardware). Newest first below; older sections are
history (marked superseded where they no longer apply).

## Hand-off: state at 2026-10-26 (all hardware-verified on the bench unless noted)

Testbed: Nucleo-F303RE + X-NUCLEO-IHM08M1 + Turnigy Aerodrive SK3 3542-1250KV (7 pp, no load),
bench supply 11.4 V current-limited.

**Works:**
- Calibration: two-point Rs = 0.02275 ohm (repeatable), Ld ~5.6 / Lq ~7.2 uH, current signs checked.
- Startup Align -> I-f ramp (500 rad/s el, 800 rad/s^2) -> Transition -> Running, sensorless (eSMO).
- Speed control (1 kHz, inside `FOC_Update`, motoring only): holds 1500 / 2500 rad/s el within
  ~1 rad/s of the estimate.
- 20 kHz PWM, current loop 1 kHz bandwidth, 6x harmonic compensation (`FOC_Harmonic_Comp`):
  `i_d` ripple sd 0.18 A @1500, 0.45 A @2500 rad/s el (was 0.73 / 2.18 A without it).
- Protection: coast via gate pins (`Bridge_Outputs`, outputs really off), over-current break latched
  (OISxN = 1, AOE = 0; also in the `.ioc`), bus over-voltage trip 18 V, start refused below 8 V Vbus,
  implausible calibration results rejected.
- ISR: max ~2540 of 3600 cycles at 20 kHz (DWT, printed after each run).

**What the firmware does at every reset (`RUN_FOC_TEST 1` in `Src/freertos.c`):** offsets ->
calibration (up to ~5.3 A DC, ~0.5 s) -> 5 s speed run (1500, step to 2500 rad/s el at 3 s, up to
~3400 rpm) -> coast. **The motor spins whenever the board resets with the supply on.** Set
`RUN_FOC_TEST 0` for calibration only. A 50 Hz integer log (mode, speeds, EMF, currents, voltages,
Vbus, break flag, MOE, ISR cycles) prints on COM5 at 230400 bps.

**Open / not done:**
- Remaining current ripple ~0.3-0.4 A at 2x and 4x electrical frequency (not investigated).
- With a propeller, startup at 500/800 may lose sync (sim: 300/600 suits a prop; `I_STARTUP` 3 A gives
  only ~0.02 Nm). Not bench-tested with a load.
- Speed is the observer's estimate; no independent speed measurement yet.
- Speed loop never brakes (by design for a bench supply); regenerative braking needs a supply or
  brake resistor that can absorb energy first.
- `ESMO_OMEGA_MAX` 4000 rad/s el (~5500 rpm) caps closed-loop speed.
- Unused/legacy: write-only calibration buffers (~6 KB RAM, partly used by the diagnostics),
  `adc_v_c` (third shunt), legacy `SMO_Update`, `ADC_INJ_SMP` override (default 0, experiment showed
  no effect).
- AGENTS.md still says "Arrowdrive" and that the motor constants do not match — out of date.

**Verification tools:** `tools/sim/` (see its README; `speed_sim` added for the speed loop and the
harmonic compensation, `calib_sim` has a dead-time option) and the bench procedure in CLAUDE.md
("Running the bench yourself").

## TL;DR for whoever picks this up (superseded by the hand-off above; kept as history)

- **2026-10-26 (later): the motor RUNS sensorless closed loop on the bench** (Align -> I-f ->
  Transition -> Running, torque mode 3 A, no load). The stall was inverted current sensing plus wrong
  motor constants; see "Bench 2026-10-26" below.
- *(superseded)* The motor still stalled on the bench; the library had never run on hardware;
  `freertos.c` did not implement `outputs_enabled`; the motor constants were wrong. All resolved.

## Environment facts (user-reported 2026-10-26)

- Serial console: USART2 → ST-Link VCP, **230400 bps 8N1**, appears as **COM5** on the
  Windows 11 host. Docs previously said 115200; the docs have been updated.
  **CubeMX regeneration risk:** `Src/usart.c:42` sets `BaudRate = 230400`, but that line is
  outside the USER CODE blocks and `stm32-foc.ioc` still has `USART2.BaudRate=115200`.
  Regenerating would silently revert the firmware to 115200. Fix the `.ioc` (set 230400 in
  CubeMX), not `usart.c`.

- `platformio.ini` (2026-10-26): added `monitor_speed = 230400`, `monitor_port = COM5`,
  `build_unflags = -Os` (so `-O2` is the only optimisation level). **Build-verified
  2026-10-26**: in a clean `pio run -v`, `foc_lib.c` compiles with `-O2` and no `-Os`
  appears anywhere in the build.

## First thing to establish

Which library is in the project? Check `Lib/foc-lib/foc_lib.h`:

| Marker | Meaning |
|---|---|
| `MOTOR_STARTUP_TRANSITION` and `outputs_enabled` exist | Latest library (this STATUS describes it) |
| `V_RAMP` / `RAMP_DELTA_ANGLE` exist, no `MOTOR_STARTUP_TRANSITION` | Older library (voltage-mode startup); everything in "2026-10 changes" is missing |

Then `pio run` and record the `RAM:` / `Flash:` line here.

**2026-10-26:** latest library confirmed (both markers present). Clean `pio run`: SUCCESS,
RAM 84.1 % (55 088 / 65 536 B), Flash 10.2 % (53 704 / 524 288 B).

## Bench 2026-10-26: calibration completes, motor jerks and never spins

User report: Rs/Ld/Lq runs complete; in the 5 s torque run (`is_ref` 3 A) the motor jerked and
never really turned. Pole count unknown.

**Calibration passing does not validate the current-sense sign or phase mapping:** it is open-loop
voltage injection, and Rs/Ld/Lq all go through `fabsf`.

Host simulation (`tools/sim/startup_sim`, modified copies, current header):
- Wrong motor constants alone: the rotor *does* spin open-loop and coasts out at 1.5 s — does not
  match "never spun".
- **All currents inverted** or **phase V/W swapped (ADC vs PWM)**: current loop goes positive-feedback
  in ALIGN, current runs to ~100 A (Vbus/sqrt3 / Rs), rotor jerks. Matches.
- Current-loop gains from an over-estimated Ld (plant 10 uH): stable at 16 and 30 uH, oscillates
  (~70 A) at 50 uH. Only a candidate if the printed Ld is >= ~4x the real inductance.
- Also seen: with constants corrected (7 pp, 0.63 mWb), `OMEGA_FORCE 300` + `LOCK_TIMEOUT 0.5` still
  coasts out; 500/800 runs. Unloaded torque mode at 3 A accelerates past `ESMO_OMEGA_MAX` (4000) and
  coasts at ~1.9 s — the torque test needs a load or a lower `is_ref`.

**Diagnostic build (compile-verified, `pio run` SUCCESS):**
- `freertos.c`: `RUN_FOC_TEST 0` (USER CODE PD) skips the 5 s FOC run; after the Rs run prints
  signed means of `v_d, i_d, i_a, i_b`. Expect `i_a > 0`, `i_b ~ -i_a/2`.
- `foc_lib.c` (`FOC_BLDC_Parameter_Calculation_Inductance`): prints signed `Ld_est, Lq_est` before
  `fabsf`. Verified in `calib_sim`: **(-,-) correct; (+,+) all currents inverted; opposite signs =
  V/W swapped.**

**Result (hardware, 2026-10-26, Vbus 11.4 V): all phase currents were inverted.** Rs run gave
`i_a = -7.89, i_b = +4.05` (ratio right, both signs flipped = mapping OK, polarity wrong); Ld/Lq
signs (+,+); no V/W swap. IHM08M1 low-side shunts read below the offset for current into the motor.

**Fix:** `CURRENT_SENSE_POLARITY (-1.0f)` in `foc_lib.h`, folded into `V_2_CURRENT` (the sims model
the ADC through the same constant, so they stay consistent; `startup_sim`/`calib_sim` reproduce the
README baselines). **Hardware-verified** with the calibration run: `i_a = +7.92, i_b = -4.04`,
Ld/Lq signs (-,-). The FOC run itself has NOT been run since the fix (`RUN_FOC_TEST` still 0).

Measured on this motor (bench): **Rs = 0.0227 ohm** (header assumes 0.07), Ld = 6.7 uH, Lq = 8.1 uH
(calib_sim says the method reads ~1.6x high). Offsets 2089 / 2031 / 2056 counts.
- **Calibration current is ~7.9 A** at `INJECTION_VOLTAGE` 0.18 V because Rs is 3x lower than
  assumed: above `I_RATED` (5 A), 2 A under the 10 A abort. ~0.07 V would give ~3.1 A.
- Lq - Ld (1.4 uH) > `MIN_SALIENCY_H`, so torque mode runs MTPA (small negative id).

Still open before the motor will run closed loop (sim evidence): motor constants
(`POLE_PAIRS`, `FLUX_LINKAGE`) and `OMEGA_FORCE`/`STARTUP_ACCEL` (see Stall analysis), and the
unloaded 3 A torque test running past `ESMO_OMEGA_MAX`. Set `RUN_FOC_TEST 1` to restore the run.

### Motor constants updated to Turnigy Aerodrive SK3 3542-1250KV (2026-10-26, compile + sim only)

Published spec (HobbyKing / retailer listings): 1250 KV, Ri 0.022 ohm, 3-4S, 50-54 A max, ~760 W;
**pole count not published.** Header now: `RS 0.022`, `LS 7 uH` (bench calibration),
`POLE_PAIRS 7` (set by the user; 14 magnets assumed — count to confirm), `FLUX_LINKAGE 0.000630`
(from KV and 7 pp; 0.000735 if 6 pp), `SPEED_RATED 15000` rpm (no-load KV*12 V, unused in code).
Ratings (user choice "motor spec, board-capped"): `V_RATED 14.8` (4S max), `I_RATED 15` (IHM08M1
15 A rms; motor 50-54 A), `I_PEAK 30` (inside the ~+-31 A sense range), `SPEED_RATED 18500`.
**`INJECTION_MAX_CURRENT = I_PEAK` is therefore 30 A** — above the board's ~21 A peak; the
calibration over-current abort no longer protects the bridge. Decouple it or lower `INJECTION_VOLTAGE`.
**Done (2026-10-26):** `INJECTION_MAX_CURRENT I_RATED` (15 A), `INJECTION_VOLTAGE 0.07` V.

Hardware calibration at 0.07 V (same day): Rs-run `i_a = 2.53 A`, `i_b = -1.92 A` (ratio -0.76, was
-0.51 at 0.18 V), **Rs reads 0.0276 ohm** (0.0227 at 0.18 V), Ld 7.2 / Lq 8.3 uH, signs (-,-).
A fixed inverter voltage error (dead time / device drops) weighs more at low voltage. Two-point fit of
the 0.18 V and 0.07 V runs: **true Rs ~= 0.0204 ohm, voltage error ~= 18 mV**. Both single-point readings
are biased high. Effect of +25 % Rs: current-loop Ki and eSMO Rs (eSMO was sim-tested at +-30 %).
Option: a two-voltage Rs measurement (difference of two DC levels) cancels the offset.

**Two-point Rs implemented (2026-10-26, hardware-verified):** the Rs run now holds
`INJECTION_VOLTAGE_RS_LOW` (0.04 V) then `INJECTION_VOLTAGE` (0.07 V), 256 settle + 256 record samples
each (~100 ms), Rs = dV/dI. New `FOC_t` fields `rs_low_vd_sum`, `rs_low_id_sum`; buffers still hold
the high level (so the freertos.c signed-means diagnostic is unchanged).
- Sim (`calib_sim` copy + per-phase dead-time error 13.5 mV): two-point 0.0219-0.0224 ohm for true
  0.0220 (0.0238 with a very soft dead-time knee); single-point 0.0294. Dead time also biases Ld/Lq
  high in that model (9.6 uH read for 7).
- **Bench: `0.04 V -> 1.55 A, 0.07 V -> 2.86 A`, two-point Rs = 0.0229 ohm** (single-point 0.0245;
  spec 0.022). Ld 7.2 / Lq 8.3 uH, signs (-,-).
- **Coast fix + first motor run setup (2026-10-26):**
  - `freertos.c` now implements the `outputs_enabled` HAL contract (integration checklist item 1):
    the ISR sets TIM1 MOE only on the false->true edge (so an over-current break stays latched) and
    clears it whenever outputs are disabled.
  - `TIMER_config()` sets OIS1N/2N/3N = 1 and clears AOE at runtime. The `.ioc` still has
    `OCNIdleState RESET` (= low sides ON when MOE = 0 with the L6398's active-low LIN: brakes) and
    `AutomaticOutput ENABLE` (hardware re-sets MOE every update, undoing a coast or a break — likely
    why the original sign bug showed up as repeated jerks). **`.ioc` fixed (2026-10-26):**
    `TIM1.OCNIdleState_1/2/3 = SET`, `TIM1.AutomaticOutput = DISABLE`, `USART2.BaudRate = 230400`
    (hand-edited; open it in CubeMX once to confirm it parses). `Src/tim.c` keeps the old values until
    the next regeneration; the runtime override in `TIMER_config()` covers that.
  - Startup defaults changed 300/600 -> **`OMEGA_FORCE 500`, `STARTUP_ACCEL 800`** (light-load choice).
  - `RUN_FOC_TEST 1`, with a 50 Hz integer logger in `StartDefaultTask` (mode, forced/estimated speed,
    EMF, lock, i_d/i_q, i_a/i_b, v_d/v_q, Vbus, break flag, MOE) plus a 0.5 s post-coast check.
  - **TEMPORARY** in `platformio.ini`: `-DBEMF_LOCK_COUNT=1000000 -DLOCK_TIMEOUT=10.0f` = bench step 3
    (I-f ramp only, coast at 5 s at 500 rad/s el). Remove for the full run.
- **MOE-based coast stalled the control ISR (bench, fixed):** the injected ADC is triggered by the
  TIM1 OC4 output, which MOE also gates. Clearing MOE at idle stopped the ADC trigger, the ISR never
  ran again and calibration hung at "Measuring Rs" (found by reading RAM over SWD: adc values frozen
  at the offsets, `inj_n` 0, MOE 0). Now `Bridge_Outputs()` coasts by switching the six gate pins
  from AF6 to GPIO at their inactive levels (HIN low, LIN high); MOE stays set. A real over-current
  break still clears MOE (OISxN = 1 -> all off) and, with AOE off, halts the ISR until reset.
- **Bench steps 2, 3, 5 PASSED (hardware, 2026-10-26, ramp-only build, Vbus 11.4 V):**
  - Align: `i_d` 2.0 A, `i_q` ~0, no break.
  - I-f ramp to 500 rad/s el: eSMO `omega_est` 494-504, lock = 1, `|E|` ~298 mV vs 315 mV predicted
    for 7 pp / 0.63 mWb (368 mV if 6 pp) -> **POLE_PAIRS 7 confirmed**, flux within ~5 %. Plausibility
    threshold 158 mV, so the handover check would pass.
  - Coast at 500 rad/s el: `i_a`/`i_b` within +-60 mA for 0.5 s after stop -> outputs really off.
- **Bench step 4 PASSED (hardware, full run, 500/800, torque `is_ref` 3 A, no load):**
  ALIGN 0-0.5 s; RAMP locks from ~167 rad/s el; **TRANSITION at 1.145 s** (forced 500, est 513,
  |E| 308 mV); **RUNNING at 1.283 s**; accelerates 1473 -> 3953 rad/s el by 1.536 s (`i_q` 2.2-4.2 A,
  `i_d` -1.3..+0.4 A, `v_q` up to 2.7 V); at ~1.55 s passes `ESMO_OMEGA_MAX` (4000) -> observer
  unlock -> `FOC_Coast` -> mode IDLE. After the coast at ~5400 rpm, `i_a`/`i_b` stay ~0 (no braking).
  No break flag at any time. Observed |E|/omega rises with speed (0.60 mWb at 500 -> 0.80 at 3950
  rad/s el): not investigated (extended-EMF term with i_d != 0, or LPF/delay compensation).
- **Speed loop HARDWARE-VERIFIED (2026-10-26, no load, Vbus 11.4 V):** handover at 1.25 s (est
  1379 rad/s el, bumpless); ramp to 1500; hold 1.6-3.0 s: `omega_est` mean 1499.6, range 1499-1500;
  ramp 1500 -> 2500 at 2000 rad/s^2, 25 rad/s overshoot; hold 3.7-5.7 s: mean 2499.6, range 2499-2500.
  No break, `vbus_fault` 0, clean coast. Caveats: speed is the observer estimate (no independent
  measurement); **closed-loop current ripple**: measured `i_d` mean ~0 but sd 1.0-1.25 A, `i_q` mean
  0.6 A sd 0.6-0.75 A (50 Hz snapshots), whereas at standstill (ALIGN) the current is clean (+-25 mA).
  Not investigated: candidates are observer-angle noise into the current loop, current-loop gain from
  a high-reading Ld (1 kHz bandwidth with ~1.5 periods of delay + LPF), or the i_d offset above.
  The ISR `printf` at the handover (added by the user) was moved to the logger task (user decision).
- **20 kHz + 6x harmonic compensation: HARDWARE-VERIFIED (2026-10-26, Vbus 11.4 V, no load):**
  - 20 kHz alone (bench): `i_d` sd 0.73 A @1500 (10 kHz: 1.17), 2.18 A @2500 (10 kHz: 1.0; 6x peak 2.5 A).
  - `FOC_Harmonic_Comp()` (foc_lib.c, `HARM_*` in foc_lib.h): after the PI in `FOC_Current_Loop_Update`,
    the current error is integrated in frames rotating at +6 and -6 x theta, phase-compensated by
    conj(G_eff) of the plant at that frequency (PWM delay `HARM_DELAY_TS`, current LPF, RL, closed PI),
    rotated back and added to v_d/v_q. Each vector limited to `HARM_V_MAX` 0.5 V; active in
    MOTOR_RUNNING above `HARM_OMEGA_MIN` 300 rad/s el; reset by FOC_Coast / startup. `HARM_COMP 0`
    compiles it out. Phasors refreshed at 1 kHz in separate slots (not on the speed-loop tick).
  - Sim (speed_sim with -6/+6 EMF harmonics): true ripple 0.7-1.2 A -> 0.01 A; robust to Ld 0.6x/1.6x,
    Rs 0.5x/2x, +1 period unmodelled delay (all combined: 0.13 A).
  - **Bench: `i_d` sd 0.73 -> 0.18 A @1500 (-75 %), 2.18 -> 0.45 A @2500 (-80 %); 6x peak gone**;
    remaining ~0.3-0.4 A at 2x/4x. Speed loop calm (`iq_ref` sd 4-6 mA). No break / OV.
  - ISR (DWT): RUNNING median 1940, max 2595 of 3600 cycles (72 %). (Before splitting the phasor
    refresh across slots: max 3329 = 92 %.)
  - **Rs at 20 kHz: FIXED, hardware-verified and repeatable.** It read 0.029-0.033, then 0.017 ohm.
    Levels raised to 0.08 / 0.14 V (user OK; ~2.7 / 5.3 A, Ld/Lq injection ~2.8 A peak) — which alone
    gave 0.017 ohm, exposing the real cause: the low level comes first and pulls the unloaded rotor
    into alignment, and the 12.8 ms settle (256 samples at 20 kHz; 25.6 ms at 10 kHz) recorded the
    rotor still swinging. New `RS_ALIGN_ROUNDS` = 0.3 s at the low level before recording. Bench:
    `0.08 V -> 2.710 A, 0.14 V -> 5.347 A`, **Rs = 0.02275 ohm, twice (0.022752 / 0.022753)**; high
    level shows no drift (new freertos.c print: first/last 32 samples 5.344 / 5.349 A).
    `tools/sim/calib_sim.c` period cap raised 5000 -> 20000 (the longer Rs run was truncated).
  - Temporary capture buffer removed from freertos.c (RAM 91.5 % -> 84.1 %). Kept: DWT ISR timing.
  - Guards added: `V_BUS_MIN` (8 V) start check in StartDefaultTask; `CAL_PARAM_TOL` (4x) rejects
    implausible calibrated Rs/Ld/Lq.
- **PWM 10 -> 20 kHz (2026-10-26):**
  - ISR timing (DWT, 10 kHz build, logged as `isr_cyc`): idle ~360, align ~980, ramp ~1800, max 1956
    cycles -> ~55 % of the 3600-cycle budget at 20 kHz (RUNNING not captured: supply off).
  - `PWM_FREQ` 20000 (now `#ifndef`-guarded). Period-counted constants made time-based:
    `BEMF_LOCK_COUNT` (50 ms), `ESMO_UNLOCK_COUNT` (5 ms), `ESMO_PLL_ERR_ALPHA` (5 ms),
    `SPEED_CTRL_DIVIDER` (1 kHz). `ESMO_DELAY_COMP`/`IPARK_DELAY_COMP` unchanged (in periods).
  - `CURRENT_CONTROL_F_BANDWIDTH` pinned at 1000 Hz (was `PWM_FREQ/10` -> 2 kHz at 20 kHz).
  - Sim: eSMO angle error halves (0.28 vs 0.55 deg, all esmo_sim cases, no drops); calib Rs exact,
    Ld 8.1 uH read for 7; startup/stall/speed behaviour as at 10 kHz; true-`i_d` offset at 2500 rad/s
    el halves (-0.59 vs -1.22 A).
  - **20 kHz does NOT reduce the 6x ripple** (speed_sim with a 6 % 6x EMF term, true-current sd at
    1500/2500 rad/s el): 10 kHz/1 kHz bw 1.30/0.79 A; 20 kHz with bw 600 Hz 1.01/0.93, 1000 Hz
    1.21/1.14, 1400 Hz 1.22/1.45, **2000 Hz 0.92/2.49** (2.4 kHz harmonic on the sensitivity peak).
    A PI cannot reject 1.4-2.4 kHz; rejecting it needs a 6x resonant term (now 8 samples per ripple
    period at 2500 rad/s el instead of 4).
  - Safety gap seen on the supply-off run: with Vbus = 0, calibration accepted Ld 0.25 mH / Lq 5.3 mH
    (~35x real) and `FOC_Recalc_Params` set the current-loop gains from them. Proposed: refuse to
    calibrate/run below a minimum Vbus and reject implausible Rs/Ld/Lq.
- **Current-ripple investigation (hardware, 2026-10-26, speed holds 1500 / 2500 rad/s el, no load):**
  10 kHz capture in the ISR (`cap[400][6]` in freertos.c USER CODE: i_d, i_q, iq_ref, v_d, v_q, theta;
  +4.8 KB RAM -> 91.4 %, diagnostic, remove when done). Results:
  - Ripple is a **6 x f_el** harmonic in d/q (5th/7th in the phases), plus smaller 2x and 4x.
  - Not the speed loop (`iq_ref` sd 4-7 mA). Not the current loop oscillating (v_d/v_q 6x content
    only 17-59 mV). Not observer angle (6x angle ripple 0.05 deg). Not ADC sampling (1.5 vs 19.5
    cycles, `ADC_INJ_SMP` override added: same ripple). Not dead time (two-point Rs fit gives
    ~13.5 mV/phase, ~15x too small for the 6x voltage needed).
  - `i_d` sd: LPF 0.5: 1.17 A @1500, 1.0 A @2500; LPF off: 1.21 A @1500, **2.06 A @2500**. The LPF was
    masking it; unfiltered it grows with speed -> most likely **back-EMF 5th/7th harmonics** of the
    12N14P outrunner, at 1.4-2.4 kHz, beyond the 1 kHz current loop. LPF left at 0.5.
  - Options not yet chosen: accept (iq ripple ~0.7 A -> ~5 mNm), a 6x resonant term in the current
    loop (needs delay/phase compensation: only ~4 samples per ripple period at 2500 rad/s el), or
    20 kHz PWM for more loop bandwidth (check ISR time on PC8 first; re-tune delay comps).
- **Speed loop rewritten (2026-10-26):**
  - Old: task `StartSpeedControlLoop`, 200 ms, `iq_ref = -PI(...)` (sign inverted vs the +iq preload).
    Removed (task, attributes, creation — all in USER CODE).
  - New: `FOC_Speed_Control()` in `foc_lib.c`, called from `FOC_Update` in `MOTOR_RUNNING` +
    `FOC_SPEED_CONTROL`, every `SPEED_CTRL_DIVIDER` (10) periods = 1 kHz. Rotation-direction frame;
    **motoring only** (output and integrator clamped >= 0: no regenerative braking, user decision —
    a bench supply cannot absorb it). Reference ramp `SPEED_REF_SLEW` 2000 rad/s^2 el, starting at
    `omega_est` at the handover; integrator preloaded with the startup current (bumpless).
  - Gains from the bench plant (3 A -> ~9800 rad/s^2 el => `SPEED_PLANT_GAIN` 3300 (rad/s^2)/A):
    10 Hz crossover, `KP_SPEED` = 0.019 A/(rad/s el), PI zero at bandwidth/4 (`KI_SPEED` 0.30).
  - **Bus over-voltage trip** `V_BUS_MAX` 18 V in `FOC_Update`: coasts, sets `foc.vbus_fault`, ends a
    calibration run.
  - Test run (`RUN_FOC_TEST 1`): speed mode, 1500 rad/s el, step to 2500 at t = 3 s; logger adds `w_ref`.
  - Sim (`startup_sim` copy in speed mode, J = 1.4e-5): bumpless handover, holds 1500 and 2500 to
    +-1 rad/s, light load 1.06 / 2.46 A, prop-like 4.4 / 11.4 A.
- **Found in the speed sim (not fixed): true `i_d` is not ~0 at speed** although the measured `i_d` is
  regulated to 0: -1.2 A at 2500 rad/s el light load, -6 A prop-like. About half comes from the
  current LPF (`ALPHA_LPF_MEAS` 0.5: ~13 deg lag at 2500 rad/s el; with 1.0: -0.6 / -2.6 A), the rest
  from other sampling/PWM delays. Wastes current at speed; the bench log cannot show it (it logs the
  measured `i_d`). Candidate follow-up: `ALPHA_LPF_MEAS` 1.0 and delay compensation of the Park angle.
- Note: what looked like "hum, small jerk, stop" on the bench is the calibration itself (1 kHz
  Ld/Lq injection hum, rotor snapping to the DC field); the FOC run is still skipped
  (`RUN_FOC_TEST 0`).

Startup with the new constants (`startup_sim`, plant = header, `is_ref` 3 A torque mode):

| OMEGA_FORCE / STARTUP_ACCEL | Light load (0.002 Nm, 1e-7) | Prop-like (0.005 Nm, 5e-6) |
|---|---|---|
| 300 / 600 (header) | coasts out at 1.5 s, no handover | runs (handover 0.17 V, 0.7 deg) |
| 500 / 800 | runs (then accelerates to ~2760 rad/s el) | loses sync in the ramp, coasts |

`I_STARTUP` 3 A gives only ~0.02 Nm (Kt = 1.5 * 7 * 0.00063 = 0.0066 Nm/A). Startup tunables NOT
changed: pick per bench load (override via `build_flags`, e.g. `-DOMEGA_FORCE=500.0f`).
`calib_sim` at the new constants: Rs exact, Ld reads 5.9 uH for 7 uH (bias depends on R/L).

## Stall analysis (simulation evidence, not hardware)

### 1. Motor constants do not match the motor (most likely)

`foc_lib.h` has `POLE_PAIRS 6`, `FLUX_LINKAGE 0.003789 Wb`, `SPEED_RATED 3000 rpm`.
For a 1250 KV motor, flux linkage = 60 / (sqrt(3) * 2*pi * KV * pole_pairs):

| Pole pairs | Flux linkage | Header value is |
|---|---|---|
| 7 (typical 12N14P 3542 — confirm by counting magnets / 2) | 0.63 mWb | 6.0x too high |
| 6 | 0.735 mWb | 5.2x too high |

No-load speed at 12 V is about 15 000 rpm, not 3000.

Why this stalls the **new** startup: the handover requires
`|E_observed| >= EMF_PLAUSIBLE_RATIO (0.5) * |forced speed| * FLUX_LINKAGE`. At the
handover speed (300 rad/s el) the real EMF is ~0.19 V but the check expects >= 0.57 V,
so the lock is never accepted and the drive coasts after `LOCK_TIMEOUT`.

Simulated (`tools/sim/startup_sim`, real motor 7 pp / 0.63 mWb, J = 2e-5 kg m^2 assumed):

| Library constants | Handover speed / accel | Result |
|---|---|---|
| As now (6 pp, 3.79 mWb) | 300 rad/s el / 600 rad/s^2 | **Coasts out at 1.5 s = the stall** |
| Corrected (7 pp, 0.63 mWb) | 300 / 600 | Hands over at EMF ~0.19 V (marginal vs 0.1 V floor), runs |
| Corrected | **500 / 800** | **Hands over at ~0.28 V, runs, under light / prop-like / heavier loads** |
| Corrected | 1000 / 2000 | Falls out of step under prop-like load (3 A gives only ~0.02 Nm) |

The real torque constant is ~1.5 * 7 * 0.63e-3 = 0.0066 Nm/A, so `I_STARTUP` = 3 A
gives only ~0.02 Nm. If the bench motor carries a propeller, startup torque is the limit.

Action: measure or confirm pole pairs and flux linkage (spin by hand/drill and scope the
line-line voltage: psi = V_ll_peak / (sqrt(3) * omega_el)), fix `POLE_PAIRS`,
`FLUX_LINKAGE`, `SPEED_RATED`, then start from `OMEGA_FORCE 500`, `STARTUP_ACCEL 800`.
The startup macros are `#ifndef`-guarded, so they can be overridden from
`platformio.ini` `build_flags` (`-DOMEGA_FORCE=500.0f`). `POLE_PAIRS` / `FLUX_LINKAGE`
are not guarded — edit the header.

### 2. Integration not done (freertos.c) — see checklist below

### 3. Current sensing (classic FOC stall causes, unverified on this board)

- **Sign and phase mapping**: ADC channel for "phase a/b" must be the same phases as
  TIM1 CH1/CH2, with the current sign the library expects. Wrong sign = current loop
  runs away; swapped phases = no torque. Bench test in "Bench plan" step 2.
- **Sample instant**: IHM08M1 uses low-side shunts; the reading is only valid while the
  low-side switch is on. The injected ADC is triggered by TIM1 CC4 — confirm the CC4
  compare value puts the sample in the centre of the low-side on-time of the
  center-aligned PWM. At high duty the window shrinks; the library uses only 2 of the 3
  shunts (`adc_v_c` is unused), so it cannot pick the two with the longest window.
- **Offsets**: sampled only ~50 ms after `Start_PWM()` (`freertos.c` ~line 258). If the
  injected conversions were not running yet, offsets latch wrong.
- `Calculate_Phase_Currents` applies a first-order LPF (`ALPHA_LPF_MEAS` 0.5) to the
  currents that feed both the current loop and the observer. It costs ~0.3 deg of
  observer accuracy in simulation; set it to 1.0 if the measurements are clean.

### 4. Timing assumptions

- `PWM_FREQ` in `foc_lib.h` is **10 kHz** (older docs said 20 kHz). `TIMER_config()`
  derives TIM1 from it. `ESMO_DELAY_COMP` (0.23) and `IPARK_DELAY_COMP` (1.5) were tuned
  in simulation for 10 kHz **and** for duties that take effect one PWM period after the
  ISR computes them (preload/shadow registers). If the timer updates differently, or
  `PWM_FREQ` changes, they must be re-tuned.
- On-bench symptom of a wrong delay: at constant speed with id_ref = 0, measured `i_d`
  is non-zero and grows with speed.

### 5. Speed loop

`StartSpeedControlLoop` runs a 200 ms PI that writes `iq_ref`. It must only run in
`MOTOR_RUNNING` (the startup state machine owns `id_ref`/`iq_ref` before that). Check
its speed units against `smo.omega_est` (electrical rad/s).

## Integration checklist for `Src/freertos.c` (NOT done)

- [ ] **`foc.outputs_enabled`**: when false, the HAL must disable the gate driver / PWM
      outputs (e.g. TIM1 `MOE`), not just write 0 duty — 0 duty in complementary mode
      keeps all low-side switches on (= brakes a spinning motor through the bridge).
      `FOC_Update` keeps duties at 0 while it is false.
- [ ] Enum shift: `MOTOR_STARTUP_TRANSITION` was inserted before `MOTOR_RUNNING`. Any
      code comparing mode numbers or printing them must be updated.
- [ ] Removed macros: `V_ALIGN`, `V_RAMP`, `RAMP_INTERVAL`, `RAMP_DELTA_ANGLE`. Replaced
      by `I_ALIGN`, `I_STARTUP`, `I_REF_SLEW`, `STARTUP_ACCEL`, `OMEGA_FORCE`,
      `LOCK_TIMEOUT`, `LOCK_SPEED_TOL`, `EMF_PLAUSIBLE_RATIO`, `ANGLE_BLEND_RATE`.
- [ ] Torque mode: set `foc.is_ref` to about `I_STARTUP` before starting (or ramp it),
      otherwise the handover into `MOTOR_RUNNING` steps the torque.
- [ ] Speed mode: speed task writes `iq_ref` only in `MOTOR_RUNNING`. The startup
      preloads `speed_ctrl.integral` with the startup current at the handover.
- [ ] Remove any remaining `printf` reachable from the ISR. The library no longer prints
      from ISR paths (`ESMO_Update`, startup, current loop, `FOC_Update`).
- [ ] Keep the PC8 ISR-timing pin.

## Bench plan (each step has a pass criterion; stop at the first failure)

Current-limit the bench supply. Serial console: `pio device monitor` (230400 bps 8N1 on
COM5, set in `platformio.ini`). Log from a low-priority task at ~20–50 Hz (never from the ISR). Budget:
230400 bps 8N1 is ~23 kB/s; a ~100-character line at 50 Hz is ~5 kB/s, so keep lines short
and use a non-blocking or low-priority transmit so logging cannot starve other tasks. Fields: `motor_mode, startup_omega, smo.omega_est, smo.emf_magnitude, smo.locked,
bemf_counter, i_d, i_q, id_ref, iq_ref, v_d, v_q`.

0. **Build**: `pio run` SUCCESS; record RAM/Flash. Confirm `PWM_FREQ` and that TIM1
   really runs at that rate (scope a phase or PC8).
1. **Offsets at standstill, outputs disabled**: raw `adc_v_a/b/c` ~= `ADC_OFFSET`
   (2121 counts for 1.71 V), offsets latched within a few counts of the raw values.
2. **Current loop, rotor locked by alignment** (stay in `MOTOR_STARTUP_ALIGN`, e.g.
   `-DALIGN_TIME=5.0f`): `i_d` settles at `I_ALIGN` (2 A), `i_q` ~= 0, rotor snaps to one
   position and holds. If `i_d` diverges or is negative: current sign / phase mapping.
   Scope PC8: ISR time well under the PWM period.
3. **I-f ramp only** (e.g. `-DBEMF_LOCK_COUNT=1000000 -DLOCK_TIMEOUT=10.0f`): the motor
   should spin open-loop and follow `startup_omega`. Watch the log: `smo.omega_est`
   should track `startup_omega`, and `smo.emf_magnitude` should be close to
   `|startup_omega| * FLUX_LINKAGE`. If the EMF is ~5–6x below that, the constants are
   wrong (suspect 1).
4. **Transition + run** with defaults: expect `RAMP -> TRANSITION -> RUNNING`, no current
   above `I_STARTUP` (+ ripple), then measure `i_d` at constant speed (should be ~0).
5. **Coast**: call `FOC_Coast()` at speed; outputs must go off (no braking current).

Only after a step passes on the bench may it be marked hardware-verified below.

## Verification status of the 2026-10 changes

| Change | Compile | Host simulation | Hardware |
|---|---|---|---|
| eSMO (`ESMO_Update`) | yes | yes (angle error 0.5–2 deg, both directions, SPM + IPM) | **no** |
| PI anti-windup gain fix | yes | yes | **no** |
| Trig / Normalize_Angle / atan2 / sqrt rework | yes | yes (+ Cortex-M4 cycle model) | **no** |
| SVPWM 2/3-gain fix | yes | yes (matches textbook SVPWM to 0.5 counts) | **no** |
| Coast / idle really disables outputs | yes | yes | **no** |
| Align -> I-f -> Transition -> Run startup | yes | yes (header constants AND corrected constants) | **no** |
| Calibration path | yes | yes (Rs now exact; Ld/Lq still biased) | **no** |
| `pio run` of any of the above | yes (2026-10-26) | — | — |

Host simulation = `tools/sim/*` (see `tools/sim/README.md`): ADC quantisation and noise,
`FOC_Update` as compiled, duties applied one period later, PMSM with inertia and load.
It is a model: the rotor inertia (2e-5 kg m^2) and loads are assumptions.

## 2026-10 changes (claude.ai session) — what changed and why

### Observer
- **eSMO with PLL** (`ESMO_Update`, installed by `SMO_Init` when `SMO_USE_ESMO` = 1;
  legacy `SMO_Update` kept). Structure follows TI SPRUJ26A §2.3.1, with four deliberate
  deviations because the reference has errata (each verified by simulating the literal
  equation):
  1. Eq. 11/17 write `V - e + z`; correct is `V - e - z` (literal form diverges).
  2. Eq. 15/16 subtract the LPF lag; correct is to add `atan(w/wc)` (literal: 53 deg error).
  3. Eq. 18–20 omit Ts (`exp(-Rs/Ld)`, `2*pi*fc`); correct has Ts (literal: 36 deg error
     on a typical motor).
  4. Eq. 18/19 use Lq for beta; extended-EMF model (Eq. 11) uses Ld on both axes.
  **Do not "fix" the code to match the document.**
- Discrete-stability boundary layer `phi = k*G/lambda`, kappa gain correction, PLL on
  the EMF vector (direction-independent), lock flag with hysteresis, speed-scheduled LPF
  cutoff with constant lag compensation, `ESMO_DELAY_COMP` latency compensation.
- Inverse-Park angle advanced by `IPARK_DELAY_COMP * omega * Ts`.
- Optimised: parameter-only terms precomputed in `SMO_Compute_Coefficients()` (rerun via
  `SMO_BLDC_Specs_Update`); no expf/fmodf/fminf/fmaxf in the ISR path.

### Control / safety
- **PI anti-windup** tracking gain was inverted (`Kp/Ki` instead of `Ki/Kp`): current-loop
  correction was ~1e-8 per step (none). Fixed and `Kt*Ts` capped at 1. Simulated windup
  251 V -> 6.9 V; overshoot 27.6 A -> 15.7 A.
- **SVPWM applied only 2/3 of the commanded voltage** (sector formulas used alpha/beta
  instead of line-voltage projections). Effects: eSMO saw 1.5x the real voltage, max phase
  voltage 4.6 V instead of 6.93 V, calibration Rs read 1.5x high (the old "1.5*Rs for a
  star winding" note described this bug). Replaced by min-max common-mode injection
  (identical duties to SVPWM). **Consequence: the same `INJECTION_VOLTAGE` now drives
  1.5x the real current: 0.2 V -> 2.86 A at Rs = 0.07 ohm (was 1.9 A), abort at 8 A.**
- **Coast / idle did not coast**: `FOC_Update` overwrote the zero duties with 50 % on all
  phases (zero vector, bridge switching = brakes a spinning motor; simulated 38 A).
  New `outputs_enabled` flag; modulator not run while false. HAL must disable outputs.
- **Startup rewritten**: Align (id = `I_ALIGN`) -> I-f ramp (iq = `I_STARTUP`, forced angle
  accelerating to `OMEGA_FORCE`) -> **Transition** (observer angle + offset decaying at
  `ANGLE_BLEND_RATE`) -> Run (speed integrator preloaded). All stages current-controlled.
  Lock accepted only if observer locked AND speed agrees AND EMF plausible
  (`EMF_PLAUSIBLE_RATIO * |w| * FLUX_LINKAGE`; rejects a false lock on a stalled rotor).
  `LOCK_TIMEOUT` fault coasts instead of looping. Simulated handover: current-vector step
  ~1 deg/period (direct switch: 153–176 deg), peak current 3.3 A (direct switch 9.5–16 A).
- Transforms use `foc->theta` / `foc->omega` set by the state machine; the old
  `smo.theta_est` / `backup_theta` juggling is gone (`backup_theta` kept, unused).
- `FOC_Current_Loop_Update` split into a shared static `FOC_Current_Control()`.

### Math (Cortex-M4 cycle counts from an instruction-level model, flash 0 wait states)
- Trig: single 1281-entry sin table (cos = +N/4), frameless fast path with a bit-pattern
  range test, `noinline` tail-called slow path (keeps the fast path frameless at -O3/LTO).
  `Fast_Sin_Cos` 139 -> 39 cycles; Park 179 -> 47. Table RAM 8 KB -> 5 KB.
  `Init_Trigonometric_Tables()` must run first (`FOC_Init` does).
- `Normalize_Angle`: >1 turn out uses k = floor(a/2pi) and two FMAs (exact, corrects the
  float 2pi constant) instead of fmodf: 182 -> 68 cycles, constant time, 18–100x more
  accurate. Portable Cody-Waite fallback without hardware FMA.
- `Fast_Atan2`: degree-11 minimax, 1.9e-6 rad (was 2e-4), C99 edge cases (0,0 / NaN / inf).
- `Fast_Sqrt`: was wrong outside ~[0.5, 50] (Fast_Sqrt(1e4) = 323). Now inline `VSQRT`.
- SVPWM: squared-magnitude test, sqrt only when clamping: 127 cycles (was 129, and wrong).
- Build: use `-O2` (or `-O3`, same speed, +17 % code) for `foc_lib.c`; avoid `-Os` (Park
  77 vs 47 cycles); **never `-ffast-math`** (deletes the NaN/inf guards). Consider
  `-mslow-flash-data` only if flash data reads stall.

### Constants touched
- Startup macros replaced (see checklist). New: `SQRT3_OVER_2`, eSMO tunables
  (`ESMO_*`), `IPARK_DELAY_COMP`, `EMF_PLAUSIBLE_RATIO`.
- `INJECTION_VOLTAGE` 0.2 V (2.86 A real at Rs 0.07, see SVPWM note). **Currently 0.18 V
  in `foc_lib.h`** (2026-10-26) → ~2.57 A at Rs 0.07.

## Known-remaining / open

- **Motor constants** (above). Highest priority.
- **Inductance calibration biased**: simulation reads Ld ~16 uH for 10 uH. The 1 kHz
  injection DFT does not compensate the one-period PWM delay and the current LPF.
- `ESMO_DELAY_COMP` re-tune after constants / PWM timing are confirmed (bench: i_d ~ 0).
- ~6 KB write-only calibration buffers (`adc_v_a_buff`, `adc_v_b_buff`, `i_a_buff`,
  `i_b_buff`, `pwm_*_buff`) — confirm intent before deleting (RAM).
- `adc_v_c` / `adc_offset_c` unused (third shunt).
- `FOC_Coast` latches `FOC_IDLE` (no auto-restart) — intentional for now.
- Unused `v_bus` parameter warning in legacy `SMO_Update`.

---

## History: 2026-09-17 status (superseded where the 2026-10 section says so)

`foc_lib.c` / `foc_lib.h` had a code review; all 13 critical/high bugs found were fixed and
the project built clean (`pio run` -> SUCCESS, RAM 88.6 %, Flash 9.6 %). Nothing was
verified on hardware.

Calibration Ia/Ib == 0 bug (fixed, unverified): the over-current abort used
`INJECTION_MAX_CURRENT = I_RATED` (5 A) with `INJECTION_VOLTAGE` 1.2 V, so it tripped on
the first ISR tick; the abort called `FOC_Coast()` and looked like a successful run.
Fixed: abort keeps `foc_mode` and buffers, `StartDefaultTask` clears `inj_fault` before
each run and prints `!! ABORTED on over-current`. (Note: the current figures quoted at the
time assumed "1.5*Rs"; that was the SVPWM gain bug, see 2026-10.)

Fixes applied 2026-09 (compile-verified only):
1. Dead BEMF filter in `SMO_Update` restored.
2. 0.5x current gain in `Calculate_Phase_Currents` (LPF state) fixed.
3. SVPWM unsigned underflow -> signed duties, both-end clamp, angle-preserving magnitude
   clamp. (2026-10: modulator replaced, clamp kept.)
4. `v_d` double integration fixed.
5. `vq_limit` NaN / unlimited sentinel fixed (`PI_NO_OUTPUT_LIMIT`).
6. MTPA divide-by-zero for Ld == Lq fixed.
7. Speed refs ignored by the current loop fixed.
8. Speed PI sample time `TS_SPEED`.
9. SMO failure now stops the motor (`FOC_Coast`). (2026-10: Coast now really disables.)
10. `FOC_IDLE` de-energize made unconditional. (2026-10: it still ran the modulator — fixed.)
11. Calibration over-current abort added.
12. Startup timings. (2026-10: startup replaced.)
13. Inverted direction fixed.
Plus: NaN guards in Rs/Ld/Lq calculation, `SMO_Compute_Coefficients`, bounded
`Normalize_Angle` (2026-10: reworked), trig index mask (2026-10: table reworked), speed-LPF
coefficient swap, `fabsf`, SMO on current period's Clarke output.
