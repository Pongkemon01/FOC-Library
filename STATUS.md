# STATUS — sensorless FOC (Nucleo-F303RE)

Last updated: 2026-10-26 (end of the Claude Code session; development continues manually).
Git history starts on 2026-10-26; this file holds the record before that and the verification
status of each change (compile / sim / hardware). Newest first below; older sections are
history (marked superseded where they no longer apply).

## 2026-11-02: Blue Robotics T200 thruster (unknown motor); HFI gated on the start probe

- Bench, T200 in air, no changes to motor constants (SK3 defaults), 11.36 V:
  - Calibration: Rs 0.169-0.196 Ohm (top level 5.19 A), Ld 51-64 uH. Lq is not repeatable
    (62 / 70 / 212 uH over three runs) and the saliency sweep gives nonsense (L ~ 10 mH near
    60/240 deg): the light, low-cogging rotor probably moves under q-axis injection. Open.
  - HFI start probe -82..-100 mrad (SK3: ~-370): not salient enough -> Align -> I-f, eSMO lock
    at ~500 rad/s el, closed loop. Flux measured 0.98 mWb.
  - Before the fix: the running HFI tracker still took over below HFI_OMEGA_LOW (~450 rad/s el),
    lost the angle ~0.3 s later and the motor ran away near no-load speed (vq at the limit,
    estimate 0) for ~11 s. No fault caught it.
- Fix: `hfi_salient` (set at each HFI start, cleared when the probe fails): the tracker never
  starts, so never takes over, for the rest of that run. Shadow test unaffected (stays true).
  Sim: salient cases (`speed_sim` PROF=start SAT=0.01, PROF=low; HFI_K 0.14) byte-identical;
  a T200-like plant (PRS 0.19, PLS 60e-6, Lq/Ld 1.05, 0.98 mWb, PROF=bench) took over at 293 and
  faulted before, now stays on the eSMO and goes IDLE at 8.6 s when the eSMO loses lock.
- Bench after the fix (hardware-verified, TEST_PROFILE 4): eSMO in control throughout, holds the
  150 rad/s el step (estimate 95-180), at the 0 step loses lock and coasts to IDLE; the demo does
  not restart, so the 150 / 1500 steps after it are not run. No runaway. ISR max 1746 cycles
  (tracker off).
- Not covered: low-speed / standstill holding on a non-salient motor (needs HFI or another
  low-speed estimator).
- HFI-control sanity checks (added the same day; both latch `hfi_fault` after 20 ms and coast):
  voltage balance (`HFI_VBAL_RATIO`: |v_dq| above Rs|i| + |w|(Ls|i| + flux) by 0.3 x the voltage
  limit: the morning's runaway, vq at the limit with the tracker at ~0) and over-speed
  (`HFI_OVERSPEED`: |hfi_omega| above 2 x HFI_OMEGA_UP, i.e. 780 rad/s el). Sim: salient cases
  byte-identical (PROF=start/low/bench, SEED=180); the sim never reproduces either failure.
  Bench, T200 with the probe gate temporarily bypassed: run 1 (voltage check only) showed a
  different failure, the tracker's PLL running off to ~2000 rad/s el with the rotor stopped
  (eSMO ~0, no voltage saturation), not caught; run 2 with the over-speed check: tripped at
  ~800 rad/s el (t 3.41 s), hfi_fault 1, IDLE. Hardware-verified: over-speed branch only. The
  voltage-balance branch is compile/sim-verified (no false trips) but not yet seen to trip on the bench.
- Voltage-balance trip, sim fault injection (`speed_sim` env `HFIFAKE=<deg>`, from `HFIFAKE_T`,
  default 3 s: the tracker in control reads omega 0 and an angle `deg` behind the rotor, the T200
  runaway). `PROF=low HALF=0.5 ... 0.002 1e-7 1.33 -1 9 1 <psi> 7`, takeover at 2.74 s, ref 150:
  0/45/70 deg trip as "voltage balance" at 3.19/3.24/3.44 s, rotor already 3830/3570/3320 rad/s el
  (psi 0.63 mWb); psi 0.98 mWb (T200), 45 deg: 3.14 s at 2630 rad/s el. 90 deg gives no torque,
  no runaway, no trip. Without HFIFAKE unchanged (no fault). Sim-verified: the check works but is
  late, since the 0.3 x v_bus/sqrt3 margin (2.1 V at 12 V) hides back-EMF up to ~2.1 V / psi.
  Tightening `HFI_VBAL_RATIO` would trip earlier; not done (false-trip risk unmeasured).
- Lq run holds the rotor (2026-11-02): it applied v_d = 0, so nothing held the light rotor at
  inj_angle while the 1 kHz q-axis voltage pushed it. Now v_d = `CAL_LQ_HOLD` (0.5) x
  `CAL_CURRENT` x Rs (~2.6 A DC d-axis) under the q injection; frequency kept at 1 kHz (at 5 kHz
  the one-period PWM delay is 90 deg). calib_sim (blocked rotor) unchanged: Lq 10.97 vs 10.99 uH.
  T200 bench, 3 calibration-only runs: Lq 58.4 / 56.8 / 80.1 uH (before 62-212), Ld 47.5 / 52.2 /
  45.7 uH; the saliency-sweep fit agrees (Lq ~58.3 / 56.5 / 66.1 uH). Hardware-verified: spread
  much smaller, one run still high. Rs in the same runs 0.157 / 0.226 / 0.167 Ohm (separate).
- Rs spread (0.155-0.226 Ohm) was a bad phase connection, not the algorithm: in the Rs run
  (voltage along phase A) i_b/i_a should be -0.5 but read -0.51 to -0.96 on the T200 (phase C
  carrying 4-49 % of i_a; SK3 steady -0.53). After the user reseated the connectors, 3
  calibration-only runs: Rs 0.1389 / 0.1397 / 0.1386 Ohm, Ld 43.7 / 43.8 / 44.0 uH, Lq 48.4 /
  49.1 / 49.7 uH, i_b/i_a -0.56 / -0.52 / -0.42 (ramp 0.73 V, was 0.81-1.12 V). Hardware-verified.
  The T200 figures above this line were taken with the bad connection.
  Probe rechecked with the good connection (normal build): -97 mrad (needs < -200), sweep A2 2.3 uH
  on L0 45 uH, so the T200 really is too weakly salient for HFI; the gate (7e6abc3) stays. Run:
  Align -> I-f -> eSMO, 1500 then 150 rad/s el held, no fault. Rs that run 0.126 Ohm.
- Bench oddity: the saliency sweep was sane in the bypass runs (L0 57.6 uH, A2 8.9 uH, axis
  163 deg) but nonsense in the first T200 run; the Lq spread suggests the rotor moves.

## 2026-10-30: self-commissioning (unknown motor)

- Calibration no longer needs the motor constants. Rs run: DC voltage ramps (ceiling 0.25 x v_bus
  in 1 s) until |i_d| reaches `CAL_CURRENT` (0.35 x I_RATED = 5.25 A); at the ceiling it accepts
  >= 0.2 x `CAL_CURRENT` (high-R motors), else `inj_fault` (no bus, open winding). Then the
  existing two-point Rs at half / full of that voltage. Ld/Lq amplitude `CAL_CURRENT * Rs`
  (|Z| >= Rs). `INJECTION_VOLTAGE`, `INJECTION_VOLTAGE_RS_LOW` removed.
- Acceptance: broad physical limits (Rs 1 mOhm..50 Ohm, L 0.5 uH..0.1 H, Lq/Ld within 4x)
  instead of +-4x around RS/LS (`CAL_PARAM_TOL` removed).
- HFI amplitude `foc->hfi_v = HFI_STEP_CURRENT * Ls / Ts` (0.54 A step; 0.0756 V at 7 uH, cap
  5 % of V_RATED), recomputed by `FOC_Recalc_Params`. `HFI_VOLTAGE` removed. The HF current step
  stays the same on any motor, so `HFI_ZP_MIN` stays absolute. `HFI_POL_CURRENT` = 0.4 x I_RATED.
- Flux estimate accepted within FLUX_LINKAGE /10 .. x10 (was 0.5..2x).
- Sim-verified: calib_sim with wrong library defaults (-DRS=0.2 -DLS=70e-6, and 0.0044 / 1.4e-6)
  against an SK3 plant (new env PRS/PLS) reads Rs 0.0220 and the same L as before; other plants
  0.004..2 Ohm, 1.4 uH..2 mH all measured within the pre-existing +16 % L bias. speed_sim
  (PROF=start/bench, HFI start) with 5x wrong RS/LS/FLUX_LINKAGE in both directions: identical
  start and holds, flux converges (-3.5..+1.3 %; +22 % from 3.0 mWb in the 5 s start profile, still
  settling). A 0.2 Ohm / 70 uH / 3 mWb plant: HFI start, holds 1500, flux -0.1 %. run_sims.sh:
  small changes only (hfi_v 0.0756 vs 0.075 V; k_nosal Align transient 4.2 A vs 3.2 A, the
  non-salient tracker angle being arbitrary). Compile-verified (`pio run`).
- Not covered: the first I-f start (FOC_HFI 0 or HFI fallback) still checks the back-EMF against
  `FLUX_LINKAGE` (`EMF_PLAUSIBLE_RATIO`), and `OMEGA_FORCE`, `I_STARTUP`, the speed-loop gain
  (`SPEED_PLANT_GAIN`, inertia), `HFI_K` and `HFI_OMEGA_LOW` stay fixed. A low-flux motor's first
  I-f start can coast out.
- Bench 2026-11-02 (hardware-verified, Vbus 11.36 V, TEST_PROFILE 4, no load), two builds:
  - Normal: Rs 0.02311 Ohm (points 0.0795 V -> 2.70 A, 0.159 V -> 6.14 A), Ld 5.63 uH, Lq 7.45 uH,
    hfi_v 0.0706 V, HFI start (probe -367 mrad, polarity kept), holds 150 and 1500 rad/s el,
    flux 0.664 mWb.
  - Wrong defaults (`-DRS=0.2f -DLS=70e-6f -DFLUX_LINKAGE=0.003f`): Rs 0.02212 Ohm, Ld 5.49 uH,
    Lq 7.42 uH, hfi_v 0.0697 V, same start and speed holds, flux 0.6656 mWb (from 3.0 mWb).
  - The ramp's top level reached 6.14 A, ~17 % above `CAL_CURRENT` (5.25 A): the swinging rotor's
    back-EMF stops the ramp early. Fixed 2026-11-02: the alignment hold runs at the full level and
    then scales it back to `CAL_CURRENT` (down only); the low level gets its own
    INJECTION_ROUNDS settle block. Sim (calib_sim 1.2): Rs 0.0220 (true 0.0220), VDT case 0.0224,
    1 Ohm plant 0.9925. Bench (hardware-verified, normal build): 0.0685 V -> 2.33 A,
    0.137 V -> 5.31 A, Rs 0.02303 Ohm, Ld/Lq 5.64/7.44 uH, HFI start and 150/1500 holds as before.
  - ISR max ~2230 cycles (IRQ ~2290) of 3600: not a regression. The ~1550 in CLAUDE.md predates
    HFI (always on since 63a08c7); bench maxima with HFI were 2008-2213 (sections below). CLAUDE.md updated.

## 2026-10-30: no pole-pair parameter; flux linkage measured while running

- The pole-pair count is not used anywhere in the control (all electrical units); `POLE_PAIRS`
  and `bldc.pole_pairs` removed from foc-lib. An application that wants rpm converts itself.
  (Detecting it from the motor alone would need mechanical asymmetry; dropped.)
- `bldc.flux`: starts at `FLUX_LINKAGE`, then `Flux_Estimate` updates it while the eSMO is in
  control and locked above 2 x OMEGA_FORCE: |E| / (g * |omega|), g = the eSMO's back-EMF magnitude
  gain, fitted in sim vs omega*Ts (0.894 + 0.357 wTs + 0.73 wTs^2; the analytic LPF gain alone
  left +3..+6 % speed-dependent bias), 0.5 s low-pass, values outside 0.5..2x FLUX_LINKAGE
  ignored. MTPA and the startup back-EMF plausibility check use bldc.flux.
- Sim (`speed_sim` FLUX=1): plant 0.50 / 0.63 / 0.80 mWb read within +-1 % (1500 and 2500
  rad/s el), -2.9 % at iq ~12 A, +0.3 % with 20 mV dead time, -1.5 % with Rs 30 % off. All 25
  regression outputs byte-identical with FOC_HFI=1 (the estimate does not change those runs).
- The demo prints the measured flux after the speed test. Compile: RAM 78.5 %, Flash 9.9 %.
- Bench 2026-10-30 (prop on, TEST_PROFILE 4, FOC_HFI 1 restored after an unintended local 0):
  measured flux 0.665 mWb (FLUX_LINKAGE 0.630 from KV 1250 at 7 pole pairs: +5.6 %, within a
  hobby motor's KV tolerance; the eSMO's raw |E|/omega at 1500 rad/s el, 0.607, x 1/0.925 =
  0.656). HFI start, holds and the HFI -> eSMO switch unchanged; ISR max 2207 cycles.

## 2026-10-30: demo application (Src/freertos.c) cleaned up

- Restructured as the reference for using foc-lib: a usage overview at the top, options grouped,
  a port layer (console, TIMER_config, Start_PWM, Bridge_Outputs), the demo steps as functions
  (Measure_Current_Offsets, Calibrate, Saliency_Sweep, Speed_Test with Test_Speed_Ref per
  TEST_PROFILE and Log_Row), the control ISR documented as the library contract. Comments and
  recommendations (bench current cap, supply power, task stack, torque mode). Removed the unused
  Stop_PWM / Set_Debug / Clear_Debug; Indonesian comments translated. CubeMX-generated code
  outside the USER CODE blocks unchanged (checked).
- Behaviour change: the zero-current offsets are a 64-sample average (was one sample); Ld/Lq are
  not recalculated when a run aborted. Log format unchanged (tools/hfi_threshold.py still reads it).
- Verification: compile (RAM 78.5 %, Flash 9.9 %); bench 2026-10-30 (prop on, TEST_PROFILE 4):
  offsets A/B/C 2092/2034/2066, Rs 0.02266, Ld/Lq 5.57/7.19 uH, sweep A2 0.91 uH, HFI start kept
  polarity (11 %), 150 hold 149.5 rad/s el, HFI -> eSMO at ~456, ISR max 2213, no faults.

## 2026-10-30: foc-lib refactored for portability (no behaviour change)

- `foc_lib.h`: porting guide at the top, then parameters in three parts: (1) board/sensing/motor
  values a port must provide (reference defaults kept), (2) optional tuning (port timing, HFI,
  startup, loops, calibration levels; `IPARK_DELAY_COMP`, `CURRENT_CONTROL_F_BANDWIDTH`,
  `SPEED_PLANT_GAIN`, `SPEED_CTRL_BW_HZ` (was SPEED_CTRL_BANDWIDTH in rad/s), `SPEED_REF_SLEW`,
  `ESMO_OMEGA_MAX` are now `#ifndef`-guarded too), (3) fixed array sizes. Then types and the
  public API only. Removed the unused `I_PEAK`.
- Internal to `foc_lib.c`: math constants (`PI`, `TWO_PI`, ... no longer leak into user code,
  e.g. CMSIS `arm_math.h` also defines `PI`), `CONSTRAIN`, derived scalings (`TS`, `ADC_2_*`,
  `ADC_OFFSET`, `V_2_CURRENT`), gains, eSMO/HFI/calibration internals and their documentation;
  22 helper functions are `static`; the unused wrappers `PI_Update`, `SVPWM_Modulation`,
  `Inverse_Park_Transform`, `FOC_Current_Loop_Update` are gone.
- API changes: `FOC_Init(foc, pwm_res)` (the offsets start at V_OFFSET's count); new `FOC_Start`,
  `FOC_Stop`, `FOC_Set_Current_Limit`, `FOC_Start_Measurement`, `FOC_Measurement_Busy`.
  `Src/freertos.c` uses them instead of writing foc_mode/motor_mode/inj_* or the speed PI.
- Sims `#include "foc_lib.c"` (white-box); build each tool alone (tools/sim/README.md).
- Verification: compile with FOC_HFI 1 and 0, -Wall -Wextra, no warnings; firmware RAM 78.5 %,
  Flash 10.0 %; the ISR path is all in .ccmram (more of it now inlined into FOC_Update: CCM
  0x2de8 -> 0x2d48 bytes). Sims: 25 scenario outputs (default, FOC_HFI=0, HFI scenarios, unit
  checks) byte-identical to b5d54af except pi_windup's renamed label.
- Bench 2026-10-30 (prop on, TEST_PROFILE 4): unchanged behaviour. Rs 0.02265, Ld/Lq 5.54/7.17 uH,
  sweep A2 0.91 uH; HFI start probe -351 mrad, polarity kept (11 % contrast); 150 hold 149.6
  rad/s el; HFI -> eSMO at ~451. ISR max 2213 cycles (before: 2185-2197), IRQ 2267, of 3600.

## 2026-10-30: HFI always on; saliency probe before the polarity pulses

- User decision: HFI is no longer optional at run time. Removed `foc->hfi_enable` and
  `foc->hfi_start`: every start from standstill is `MOTOR_STARTUP_HFI` and `MOTOR_RUNNING` always
  runs the eSMO <-> HFI switch (`FOC_Current_Loop_HFI`; `FOC_Current_Loop_Update` too).
  Port-level `FOC_HFI` (default 1, `#ifndef` in foc_lib.h, example in `Inc/foc_lib_conf.h`) = 0
  compiles HFI out for a motor without enough saliency: Align -> I-f and the eSMO only.
  `foc->hfi_takeover` stays as a test-only field (default true; the shadow run for measuring
  HFI_OMEGA_LOW sets it false: Align -> I-f start, tracker logged, never in control).
- New saliency probe (stage 1 of MOTOR_STARTUP_HFI, 10 ms, no current): PLL frozen, injection
  45 deg off the tracker axis; with saliency the error reads ~-0.5 rad on either end. Above
  -`HFI_PROBE_MIN` (0.2) no polarity pulses are applied and the start falls back to Align -> I-f.
  Found in sim once HFI became the default: on a non-salient plant (Lq = Ld) the tracker angle is
  meaningless but looks converged, and the 6 A pulses kicked the rotor to -319 rad/s el.
- `Src/freertos.c`: `RUN_HFI` -> `HFI_SHADOW_TEST` (0 normal, 1 shadow); the HFI start line also
  prints the probe value. speed_sim: options HFI / HSTART removed (always on), SHADOW kept.
- Verification: compile (RAM 78.5 %, Flash 10.0 %; FOC_HFI 0 and 1 both compile without
  warnings). Sims: built with -DFOC_HFI=0 all five standard sims are byte-identical to c4c36a9.
  Default build (-DHFI_K=0.14f): Lq = Ld -> no pulses, Align -> I-f at 0.11 s, rotor unmoved;
  salient without saturation -> pulses, below margin -> Align -> I-f; SAT 0.01 -> HFI start, flip,
  bench schedule eSMO -> HFI -> eSMO; wrong-pole seed re-seeded; shadow; prop start at 5 A.
  `startup_sim 0.03 1e-5 1 -1 2.5` (heavy load) coasts out with and without HFI: I-f at
  I_STARTUP gives ~0.020 N m against 0.03 N m (the README's "heavy load succeeds" baseline was
  stale since the motor constants changed).
- **Bench 2026-10-30 (prop on, TEST_PROFILE 4, 5 A cap): hardware-verified, no fault.** Probe
  -370 mrad (limit -200; -500 if HFI_K matched exactly: the standstill K is ~0.22 vs 0.30), then
  |Zp| +pulse 2.754 A, -pulse 3.092 A: 12 % contrast, **flipped** (-1; the previous run kept +1,
  so both branches are now seen on hardware). 150 hold on HFI 149.6 +- 6.3 rad/s el (eSMO - HFI
  -5.5 +- 3.4 deg), 0 hold 0.1, HFI -> eSMO at ~452 rad/s el during the 1500 step. ISR max 2197.

## 2026-10-30: threshold from data; standstill start on HFI (N/S polarity)

- `HFI_OMEGA_LOW` 293 -> **300 rad/s el** (409 rpm), derived from the stage-1 shadow log with the
  new `tools/hfi_threshold.py` (bins smo_hfi by eSMO speed; lowest speed above which the spread
  stays within 1.5x the fastest bins' spread: 275-300 depending on bin width; 300 taken). Re-run
  the script on a new shadow log (RUN_HFI 1, TEST_PROFILE 3) for another motor or setup.
- `MOTOR_STARTUP_HFI` (new mode 9, appended so log mode numbers stay), with `foc->hfi_start`
  (+ hfi_enable, hfi_takeover): tracker locks from theta = 0 at zero current (100 ms), then
  +/-`HFI_POL_CURRENT` d-axis pulses (12 ms each, 2000 A/s slew, current held on the locked angle)
  and compares |Zp| (larger with the magnet's flux: saturation); flips the tracker by pi if the
  negative pulse wins; below `HFI_POL_MARGIN` (2 %) contrast it falls back to Align -> I-f.
  Then MOTOR_RUNNING with HFI in control at zero speed (eSMO reset, takes over above UP).
  Total ~136 ms. Longer pulses (30/80 ms) that followed the tracker turned the sim rotor 17-38 deg
  (a pulse on the wrong end is an unstable equilibrium); the short fixed-angle pulses do not.
- Sim (`speed_sim` new options HSTART, SAT (d-axis saturation model), PROF=start, IMAX): with
  SAT 0.01 / 0.005 per A the rotor (at 114 deg, so the tracker first locks on the S end) is
  detected and flipped (|Zp| contrast 12 % / 6 %), runs from 0 to 150 on HFI, eSMO at ~424-430;
  with no saturation it falls back to Align -> I-f; with the prop inertia at a 5 A cap it
  accelerates under HFI at up to 3.7 A and hands over at the cap. HFI off: all sims identical.
- `Src/freertos.c`: `RUN_HFI 3` (standstill start on HFI), `TEST_PROFILE 4` (150, 0 from 5 s,
  150 from 6.5 s, 1500 from 8 s), prints the polarity result. Compile: RAM 78.5 %, Flash 10.0 %.
- **Bench 2026-10-30 (10x4.7 prop on, RUN_HFI 3, TEST_PROFILE 4, 5 A cap): hardware-verified, no
  fault.** Polarity: |Zp| +pulse 3.010 A, -pulse 2.726 A (5 A pulses, capped): **10 % contrast**,
  5x the 2 % margin; kept (+1), confirmed right (forward start, eSMO later agrees within a few deg).
  Standstill -> RUNNING on HFI in ~0.15 s; accelerated the prop under HFI at up to ~2.9 A (overshoot
  to ~199 rad/s el: motoring only), then 150 hold: HFI speed 149.4 +- 6.0, eSMO - HFI -3.1 +- 2.7
  deg, tracker error rms 3.6 deg, iq 0.52 A. 0 hold: 0.0 +- 1.2. 150 again, then 150 -> 1500 under
  HFI at the cap (logged iq up to 4.74 A with HFI in control), **HFI -> eSMO at ~449 rad/s el**,
  normal 1500 hold. ISR max 2185 cycles of 3600.
  Load coverage: inertia (acceleration) load up to ~4.7 A under HFI; a steady load at low speed was
  not applied (prop drag at 150 rad/s el is negligible; no hand brake this run).

## 2026-10-30: eSMO <-> HFI switching during MOTOR_RUNNING (step 3 of the HFI plan)

- The passive delta-sweep estimator is replaced by an HFI **tracker** that can steer:
  +-`HFI_VOLTAGE` square wave at PWM_FREQ/4 along its own estimate `hfi_theta`; per period the
  current step is projected on the axis applied 2 periods earlier, sliding PWM_FREQ/4 phasors over
  the last 4 periods give eps = Re(Zq conj Zp)/|Zp|^2 = HFI_K sin 2e (no timing assumption),
  PLL (`HFI_PLL_BW` 40 Hz, critically damped) -> `hfi_theta`, `hfi_omega`.
- `FOC_Current_Loop_HFI` (MOTOR_RUNNING with `foc->hfi_enable`): the eSMO runs every period.
  Tracker on below `HFI_OMEGA_ON` (1.6 x LOW), started at the eSMO angle/speed (fixes N/S);
  HFI takes control below `HFI_OMEGA_LOW` (293 rad/s el = 400 rpm, hard-coded, not measured)
  after `HFI_WARMUP` (20 ms); back to the eSMO above `HFI_OMEGA_UP` (1.3 x LOW) once the eSMO is
  locked and agrees (angle < 0.2 rad, speed < 20 %) for 10 ms. Both switches blend the angle
  (`angle_offset`, ANGLE_BLEND_RATE). In HFI control the eSMO's lock flag is ignored; HFI fault
  (|Zp| < `HFI_ZP_MIN` or |e| > `HFI_ERR_MAX` for 20 ms) coasts with `hfi_fault`. The speed loop
  now uses `foc->omega` (the active estimator; identical to `smo.omega_est` with HFI off).
  `HFI_K` (default 0.30, bench) normalises the PLL gain; sims build with `-DHFI_K=0.14f`.
- Startup is unchanged (Align -> I-f -> eSMO); starting directly on HFI still needs the N/S
  polarity step.
- `Src/freertos.c`: `RUN_HFI`, `TEST_PROFILE` (0: 1500/2500, 1: prop staircase, 2: low speed:
  1500, 150 from 2.5 s, 0 from 8.5 s, 1500 from 10 s); log columns on/ctl/smo_hfi/w_hfi/zp/err/flt.
  smo_hfi (eSMO minus HFI angle while both run, below HFI_OMEGA_ON) is the data for choosing
  HFI_OMEGA_LOW: where it starts to grow as the speed falls.
- Verification: compile (RAM 78.5 %, Flash 10.0 %; HFI code inlined in FOC_Update in .ccmram);
  HFI off: all sims byte-identical to c4c36a9. Sim (`speed_sim` PROF=low HFI=0 HALF=0.5,
  -DHFI_K=0.14f): eSMO -> HFI at 286 rad/s el, holds 150 (true 146-152) and 0 rad/s, angle error
  mean < ~1-2 deg (per-sample max 7-11 deg), back to the eSMO at ~415 rad/s el (0.8 deg), no
  fault; tracker seeded +-30 deg pulls in within ~10-100 ms (error sign right); stable with
  HFI_K 2x high or low; prop J/kfan + dead time OK. Bench: not run yet.
- Safety additions (same day): `foc->hfi_takeover` (false = shadow: tracker runs and is logged,
  never in control); takeover also requires |eSMO - HFI| < `HFI_AGREE_ANGLE`, else the tracker is
  re-seeded from the eSMO (a wrong-pole lock seeded at +180 deg reversed the rotor to -946 rad/s
  el at 15 A before this gate; now it re-seeds and takes over correctly, peak 3.2 A); backstop
  guard: in HFI control, rotation against `rotor_dir` beyond `HFI_REVERSE_OMEGA` (50 rad/s el)
  for 50 ms latches `hfi_fault` and coasts.
- More sims: dead time 20/30 mV at the 150 and 0 holds, reverse direction, the exact bench
  schedule (`PROF=bench`) with and without the prop: all switch at ~286-292 / ~410-430 rad/s el,
  no fault. Shadow (`SHADOW=1`): the eSMO keeps control to 150, coasts at 0 (lock lost), as
  designed. With HFI enabled, the 1500/2500 profile is identical to HFI off (tracker never on).
- `Src/freertos.c`: `RUN_HFI` 0 off / 1 shadow / 2 takeover; `TEST_PROFILE 3` = slow sweep
  (800 -> 100 rad/s el at 100 rad/s^2) for the shadow run; `TEST_I_MAX` 5 A. Bench plan: stage 1
  shadow + profile 3 (measures where smo_hfi grows = the eSMO's real limit), stage 2 takeover +
  profile 2.
- **Bench 2026-10-30, stage 1 shadow sweep** (prop removed; RUN_HFI 1, TEST_PROFILE 3, 5 A cap,
  0.075 V at 5 kHz below 469 rad/s el): no fault; the tracker ran on hardware from 469 down to
  the 100 rad/s el hold (|Zp| 2.4-3.9 A, far above HFI_ZP_MIN). eSMO minus HFI angle by speed:
  400-470: -2.4 +- 1.3 deg; 300-400: -2.7 +- 1.4; 200-300: -3.7 +- 2.3; 130-200: -3.7 +- 3.2;
  <130: -4.0 +- 5.2 (min/max -15.5/+6.2). Constant offset ~-2.5..-4 deg; the spread (the eSMO's
  noise; the tracker's should not depend on speed) grows below ~300 rad/s el, so HFI_OMEGA_LOW
  = 293 rad/s el (400 rpm) sits at the knee: **keep it** (now measured, not just assumed).
  The eSMO's lock flag stayed true at 100 rad/s el (emf ~65 mV): useless as a limit, as expected.
  ISR max 2123 cycles of 3600 with eSMO + tracker.
- **Bench 2026-10-30, stage 2 HFI takeover** (prop removed; RUN_HFI 2, TEST_PROFILE 2, 5 A cap):
  **hardware-verified, no fault.** Coast-down from 1500: tracker on at ~456 rad/s el, **HFI took
  control at 296 rad/s el** (eSMO - HFI -5 deg at the switch). 150 hold under HFI (3.9 s): HFI
  speed 150.6 +- 6.4 rad/s el (eSMO alongside: 149.0), eSMO - HFI -3.8 +- 3.2 deg, tracker error
  rms 3.8 deg, iq 0.51 A. 0 hold: HFI speed 0.0 +- 0.5, iq 0.24 A (the eSMO wanders, +-120 deg,
  ignored). Step to 1500: **back to the eSMO at ~450 rad/s el** with iq ~4.5 A while accelerating,
  tracker off at ~545, then the normal 1500 hold. ISR max 2173 cycles of 3600.
  Not yet tested: HFI under load at low speed, starting from standstill on HFI (needs N/S polarity).

## 2026-10-30: cleanup

- Removed write-only `FOC_t` fields: `pwm_a/b/c_buff`, `adc_v_a/b_buff` (4 KB RAM: 84.8 % -> 78.5 %)
  and `backup_theta`. `i_a_buff`/`i_b_buff` stay (current-sign diagnostic in `Src/freertos.c`).
- Removed commented-out debug code: freertos.c open-loop blocks calling the long-gone
  `FOC_Open_Loop_Voltage_Control`, the idle-loop prints and LED toggle; foc_lib.c Rs prints.
- `Injection_Phasor()`: one single-frequency DFT helper shared by
  `FOC_BLDC_Parameter_Calculation_Inductance` and `FOC_BLDC_Saliency_Point` (was duplicated).
- `FOC_HFI_Update`: cos/sin(2 delta) from the double-angle identity at injection time instead of
  a second table look-up per period; `hfi_cnt` stops at HFI_DELAY + 1.
- Verification: compile (RAM 78.5 %, Flash 10.0 %); sims byte-identical to the previous commit
  (calib_sim, saliency_sim, speed_sim incl. HFI/HALF/VSAG, startup_sim, esmo_sim). ISR cycle
  saving not measured on the bench.

## 2026-10-27: passive HFI estimator alongside the eSMO (step 1 of the HFI low-speed plan)

- Goal: check that HF-injection position estimation works on this motor under load current,
  using the eSMO angle (mid speed) as reference, before letting HFI steer anything.
- `FOC_HFI_Update()` (FOC_FAST_CODE, called after `Current_Loop_Core` in MOTOR_RUNNING when
  `foc->hfi_enable`): adds a +-`HFI_VOLTAGE` (0.075 V) square wave at PWM_FREQ/4 (5 kHz, sign held
  2 periods: + + - -) along the control d-axis + delta, delta sweeping 0..pi per `HFI_WINDOW`
  (2000 periods = 0.1 s). Demodulation: per-period step of the UNFILTERED alpha-beta currents
  (the measurement LPF attenuates 5 kHz), projected on the applied axis, correlated with the
  5 kHz reference (1, -j, -1, j); the perpendicular phasors are projected on the parallel
  phasor's phase, so the PWM latch timing drops out. Lock-in on 2*delta gives `hfi_k` (saliency
  signal), `hfi_err` (rotor d-axis minus control angle, mod pi), `hfi_apar` (HF step per
  period) and `hfi_phase` (timing diagnostic). Never writes `foc->theta`; resets on coast/idle.
  Control path otherwise unchanged.
- First version (+-0.15 V at PWM_FREQ/2, sign flipped every period, second-difference demod)
  FAILED on the bench: HF step 0.13 A instead of ~1.2 A, k 0.5-1.4, err ~35 deg. Cause: TIM1
  latches a new duty mid-period (cf. the "~1.5 periods" note at the inverse Park), so each
  sampling interval held half +V and half -V. Reproduced in sim with `speed_sim` env `HALF=0.5`.
- `Src/freertos.c`: `RUN_HFI_PASSIVE 1` enables it at `HFI_ON_MS` (1.5 s) in the speed test;
  5 extra log columns (hfi_k x1e4, hfi_err 0.1 deg, hfi_apar mA, hfi_ph deg, windows), printed by
  a second printf to keep varargs stack use low.
- Verification: compile (RAM 84.7 %, Flash 10.0 %, `FOC_HFI_Update` in .ccmram); sim
  (`speed_sim` env `HFI`, `HALF` 0 / 0.3 / 0.5, constant-saliency plant Ld 7 / Lq 9.3 uH):
  k 0.136-0.153 (theory 0.142), err within 0.3-2.5 deg of the true angle up to iq 11.7 A and with
  dead time; eSMO error and speed holds unchanged. The sim plant has no cross-saturation.
- **Bench 2026-10-27 (Claude-run, unloaded, 0.075 V = +-0.6 A ripple at 5 kHz, 1.5-6.1 s of the
  speed test):**
  - HFI d-axis agrees with the eSMO control angle: err +0.0..+0.6 deg at 1500 rad/s el,
    -1.2..+0.0 deg at 2500 (window-to-window spread ~0.3 deg), +1 deg while accelerating.
    iq only 0.2-1.6 A (unloaded): load dependence NOT tested yet.
  - k 0.29 at 1500, 0.30-0.31 at 2500: about 2x the 1 kHz stationary sweep's ratio (~0.14).
    apar 0.81 A (HF step), i.e. effective L ~4.6 uH at 5 kHz vs 6.4 uH at 1 kHz. Both point to
    frequency-dependent (eddy-current) anisotropy; not yet explained. hfi_ph stable at 176 deg.
  - ISR: ~1590 cycles typical with HFI vs ~1220 without (+~370), max 2008 of 3600.
  - Speed holds, calibration and the saliency sweep (A2 0.92 uH, axis 6 deg) unchanged.
- **Bench 2026-10-30, 10x4.7 propeller load** (`LOAD_STAIRS` 1500/2000/2500/3000 rad/s el,
  `TEST_I_MAX` 10 A, HFI 0.075 V from 1.5 s; Claude-run after user OK):
  - Start-up with the prop: handover at ~600 rad/s el, fine.
  - HFI under load (k unloaded 0.29-0.31, err ~0..+0.6 deg): iq ~2 A (1500 hold) k 0.285-0.295,
    err +1.4..+2.2 deg; iq 3-4 A (2000 hold) k 0.282-0.297, err +1.9..+3.5 deg; speed ramps with
    iq 4-8 A k 0.279-0.293, err +4.5..+5.8 deg. Saliency signal drops only ~3 % up to 8 A; the HFI
    axis moves ~+0.6 deg/A relative to the eSMO angle (cross-saturation or a load-dependent eSMO
    error: not separable without an encoder). Smooth, so compensable.
  - **Run ended by a fault at 4.2 s** (2000 -> 2500 ramp, iq ~8 A): Vbus fell 11.39 -> 10.47 ->
    7.53 V within ~50 ms, then the IHM08M1 over-current comparator latched (brk = 1, MOE = 0; last
    sample iq 30 A, i_a -29 A). The control ISR stopped (MOE gates the ADC trigger), the prop
    coasted. Likely the bench supply reached its current limit (input power ~22 W, ~2 A at 11.3 V,
    at that moment) and the falling bus took the gate driver / control out of range. The firmware
    has no run-time bus under-voltage check: `V_BUS_MIN` is only checked before calibration/start.
    2500 and 3000 rad/s el not reached.
- **Run-time bus under-voltage trip (added after that run):** in `FOC_Update`, next to the
  over-voltage trip: with the outputs on, `V_BUS_UV_PERIODS` (10 = 0.5 ms) consecutive samples
  below `V_BUS_MIN` (8 V) -> `FOC_Coast`, `vbus_uv_fault` latched, calibration ended. Compile
  (RAM 84.8 %, Flash 10.0 %) and sim (`speed_sim` env `VSAG`: 12 -> 6 V in 50 ms at iq ~6 A
  trips at ~7.96 V and coasts with no current spike; regression unchanged).
- **Bench re-run 2026-10-30 (supply limit set to 4 A by the user):** same staircase.
  - The sag repeated at the same point: 2000 -> 2500 ramp, ~2300-2350 rad/s el, iq 8.1-8.4 A:
    Vbus 11.41 -> 11.33 -> 10.45 V, then the new trip coasted (`vbus_uv_fault = 1`, no hardware
    break, MOE stayed 1, phase currents ~0 while coasting, Vbus back to 11.3-11.6 V at once).
    **UV trip hardware-verified.** Input power at the sag ~1.5*vq*iq = 23 W, ~2 A at 11.4 V: half the
    stated 4 A limit, so the cause is not settled (supply limiting/protection behaviour, wiring,
    or a measurement effect at high current). 2500/3000 again not reached.
  - HFI: 1500 hold (iq 1.6-2.4 A) k 0.287-0.292, err +0.7..+1.4 deg; 2000 hold (iq 3.1-4.4 A)
    k 0.285-0.299, err +0.8..+1.9 deg; ramps (iq 6-8 A, accelerating) k 0.300-0.305,
    err +3.5..+4.6 deg. With run 1: k does not fall with load up to 8 A; the axis shift is
    ~0.3 deg/A in steady holds, more during ramps (acceleration may add eSMO lag there).
- The bench motor supply is a fixed 12 V 5 A unit (the only one available); its protection
  apparently trips on the inverter's pulsed bus current at ~20-23 W.
- **Bench gentle staircase 2026-10-30** (`LOAD_STAIRS` now: 1500, 500 rad/s^2 to 2000, hold,
  250 rad/s^2 to 2500, hold; `TEST_I_MAX` 8 A): **completed, no faults**, Vbus 11.33-11.47 V.
  - 1500 hold (iq 1.4-2.7 A): k 0.277-0.291, err -0.2..-0.4 deg.
  - 2000 hold (iq 2.9-4.3 A): k 0.289-0.298, err -0.5..-0.6 deg.
  - 2500 hold (iq 4.4-6.4 A, ~16 W): k 0.293-0.314, err -0.8..-1.0 deg.
  - So steady-state HFI stays within ~1 deg of the eSMO up to ~6 A and k does not fall (rises
    ~7 %). Run-to-run offset of err ~1 deg (this run negative, the previous one positive). With a
    prop, speed and load rise together, so their effects on err cannot be separated here.
  - ISR max 2031 cycles of 3600.

## 2026-10-27: saliency sweep (feasibility check for HF-injection start / low speed)

- Why: HF pulsating injection (e.g. Jiang & Cheng, Defence Technology 33, 2024) needs Ld != Lq;
  the SK3 is a surface-PM outrunner and the Ld/Lq split from calibration was within its bias.
- `FOC_t.inj_angle`: calibration injects and measures in this frame (`foc->theta = inj_angle`
  in `FOC_CALIBRATION`); 0 for Rs/Ld/Lq, so those runs are unchanged (calib_sim identical).
  The Ld-mode run now also records `i_q` (cross axis) into `iq_inj_buff` (the Lq run overwrites it).
- `FOC_BLDC_Saliency_Point()`: per angle, apparent L along the injection axis and the complex
  cross/axis current ratio (the signal an HFI observer tracks).
- `Src/freertos.c` `RUN_SALIENCY_SWEEP 1`: after calibration, the Ld injection (1 kHz,
  `INJECTION_VOLTAGE` 0.14 V, ~2.8 A peak, no DC) at 24 angles over 0..2pi (~0.7 s), then a
  harmonic fit L(phi) = A0 + A1 cos(phi) + A2 cos(2 phi) and the residual. Set 0 to skip.
- Verification: compile (`pio run`: RAM 84.6 %, Flash 10.0 %); sim (`tools/sim/saliency_sim`:
  axis recovered to 0.3 deg, A2 ~5 nH on a non-salient rotor); **hardware: bench run 2026-10-27**
  (Claude-run, rotor free, left aligned near 0 by the Rs run):
  - L0 6.38 uH, **A2 0.92 uH (min-L axis 6 deg el)**, A1 4 nH, residual rms 72 nH.
    Ld ~5.5 / Lq ~7.3 uH; phi and phi+pi agree within ~20 nH (rotor did not move).
  - Cross/axis current ratio: 2nd-harmonic amplitude 0.099 (~0.28 A at 2.8 A axis current,
    ~18 ADC LSB), with an imaginary part of similar size (Rs/eddy phase, not 90 deg): an HFI
    demodulator must use the measured phase, not the paper's ideal 0/pi assumption.
  - Calibration (Rs 0.02270, Ld 5.6 / Lq 7.2 uH) and the 5 s speed test (holds 1500 / 2500
    rad/s el) unchanged.
- Conclusion so far: zero-current (saturation) saliency is real, ~14 % of L0, clearly measurable at
  standstill. Not yet known: how it changes with iq load (cross-saturation), and N/S polarity
  (symmetric injection cannot see it). Nothing HFI-based is implemented.

## 2026-10-27: rotation direction API, direction-relative references

- `bool FOC_Set_Direction(FOC_t *, BLDC_Direction_t)`: sets `rotor_dir` only while the bridge is
  off (`outputs_enabled == false`: idle, coasted or faulted); returns false and keeps the old
  direction if energized (calibration, startup, running), for an invalid value or NULL. The drive
  is motoring only and cannot brake to reverse: stop, set, start again (user decision).
- `speed_ref`, `speed_ref_ramp` and `is_ref` are now relative to `rotor_dir` (user decision):
  positive = in the selected direction. A negative `speed_ref` is treated as 0 (motoring only);
  a negative `is_ref` in torque mode is braking torque. Forward behaviour is unchanged. Before,
  `speed_ref` was signed in the electrical frame, so reverse speed mode with a positive
  reference computed zero current and never ran; reverse torque mode needed `is_ref < 0`.
- `Src/freertos.c`: `RUN_DIRECTION` (default `NORMAL_DIRECTION`) selects the bench test direction
  through `FOC_Set_Direction`. Sims use the setter for their `dir = -1` argument.
- Verification: `pio run` OK (Flash 49300 B). Every existing sim output byte-identical (`cmp`) to
  HEAD 8aa9fb6, including reverse `startup_sim` (was `is_ref = -3`, now `+3` relative).
  `speed_sim ... -1` (reverse, previously impossible) holds −1500 / −2500 rad/s el with the same
  ripple and angle error as forward. Host test of the setter: accepted while off, refused while
  energized / invalid / NULL.
- Bench, first reverse attempt: the setter was refused ("bridge energized") and the test ran
  forward. `FOC_CALIBRATION` sets `outputs_enabled = !inj_fault` every period, so after the last
  injection the bridge keeps switching the zero vector until the mode changes. The bench test now
  sets `foc_mode = FOC_IDLE`, waits 2 ms (outputs off within one PWM period), sets the direction,
  then starts. (Open: calibration leaves the bridge energized when nothing follows it, e.g.
  `RUN_FOC_TEST 0`.)
- Bench, reverse (`RUN_DIRECTION REVERSE_DIRECTION`, standard profile): align, I-f to −500 rad/s el,
  lock, handover at −1264, holds −1499.4 [−1500, −1499] and −2499.5 [−2500, −2499] rad/s el,
  i_q ≈ −0.9 A (mirrors forward), lock never dropped, no break, clean coast. Forward re-run with the
  go-idle-first sequence: holds 1499.6 / 2499.4 rad/s el, normal. IRQ max ~1605 cycles both ways.
  Hardware-verified in both directions; the firmware is back to `NORMAL_DIRECTION`.

## 2026-10-27: board and motor constants overridable by the port

- `#ifndef` guards in `foc_lib.h` on `PWM_RES`, `ADC_RES`, `ADC_V_REF`, `RS`, `LS`, `POLE_PAIRS`,
  `FLUX_LINKAGE`, `V_RATED`, `I_RATED`, `I_PEAK`, `R_SHUNT`, `V_OFFSET`, `V_GAIN`, `V_BUS_GAIN`,
  `CURRENT_SENSE_POLARITY`, `V_BUS_MIN`, `V_BUS_MAX` (`PWM_FREQ` and the startup tunables already
  had them). A port defines them in `foc_lib_conf.h` or with `-D`; derived constants follow.
  Controller gains and calibration settings remain fixed in the header.
- This port keeps the defaults (no overrides in `Inc/foc_lib_conf.h`).
- Verification: firmware disassembly identical to the bench-verified build; sims unchanged; a host
  test overriding `POLE_PAIRS`, `R_SHUNT`, `CURRENT_SENSE_POLARITY`, `V_BUS_MAX` (conf header) and
  `RS` (`-D`) compiled `foc_lib.c` without warnings and picked up every value.

## 2026-10-27: code placement is a port hook, not STM32F3-specific

- `foc_lib.h` no longer names the STM32F3 CCM section. `FOC_FAST_CODE` defaults to nothing;
  with `-DFOC_LIB_USE_CONF` the library includes the port's `foc_lib_conf.h`, which may define
  it. This port's `Inc/foc_lib_conf.h` keeps the CCM placement (`.ccmram.text`); `platformio.ini`
  passes `-DFOC_LIB_USE_CONF` instead of `-DFOC_CCMRAM_CODE`.
- Verification: `pio run` gives machine code identical to the bench-verified build of the lean
  IRQ path below (same disassembly); sims unchanged. No new bench run needed.

## 2026-10-27: lean ADC interrupt path

- `ADC1_2_IRQHandler` (USER CODE 0 in `Src/stm32f3xx_it.c`) first calls `ADC_IRQ_Fast_Dispatch()`
  (`Src/freertos.c`, CCM): on ADC1 JEOS with JEOSIE set it clears JEOC/JEOS, calls the callback
  and returns, skipping both `HAL_ADC_IRQHandler` calls (ADC2 is initialised but never started).
  Anything else falls through to HAL as before. Not done: HAL's `HAL_ADC_STATE_INJ_EOC` state bit
  (nothing reads it). `-DADC_IRQ_FAST_DISPATCH=0` restores the HAL dispatch (A/B timing).
- Callback: `ADC1->JDR1..4` instead of `HAL_ADCEx_InjectedGetValue`, `GPIOC->BSRR` instead of
  `HAL_GPIO_WritePin` (PC8 pulse kept, same span). No flash veneers left inside the callback.
- New timing column `irq_cyc` (and an end-of-run max): from `ADC1_2_IRQHandler` entry to the end
  of the callback (excludes only exception entry and, with HAL dispatch, HAL's work after the
  callback). `isr_cyc` keeps its meaning (callback only).
- Bench (standard profile, fast dispatch twice, HAL dispatch once as A/B):
  callback `isr_cyc` running median 1338 → 1208, max 1684 → 1554; Align 766 → 636.
  `irq_cyc` running median 1262 / max 1608 with fast dispatch vs 1331 / 1677 with HAL dispatch
  (dispatch 54 vs 123 cycles before the callback, plus HAL's unmeasured post-callback work).
  Holds 1500 / 2500 rad/s el within ±1 rad/s, lock never dropped, no break. Hardware-verified.

## 2026-10-27: FOC_Update call tree runs from CCM SRAM

- `FOC_FAST_CODE` (foc_lib.h) puts a function in section `.ccmram.text` when built with
  `-DFOC_CCMRAM_CODE` (superseded the same day by `-DFOC_LIB_USE_CONF` + `Inc/foc_lib_conf.h`, see above; host sims build without it). Tagged: the ADC
  callback in `Src/freertos.c`, `FOC_Update` and everything it calls (startup, current loop,
  harmonic comp, `SMO_Update`, MTPA, coast, calibration update, trig/atan2/sqrt, Park slow
  paths, `Normalize_Angle`, `PI_Reset`, `Park_Transform`). 8.1 KB of the 16 KB CCM.
- The stock linker script already maps `*(.ccmram*)` to CCMRAM with a flash load address
  (`_siccmram`, `_sccmram`, `_eccmram`), but the startup file copies only `.data`: `main()`
  (USER CODE 1) copies the section, then DSB/ISB, before `HAL_Init`. The PlatformIO Flash/RAM
  summary does not count `.ccmram` (check with `arm-none-eabi-size -A`).
- `trig_table` stays in main SRAM (no I-bus/D-bus contention on CCM).
- Remaining flash calls on the per-period path, via long-branch veneers: the HAL IRQ handler
  into the callback, `HAL_ADCEx_InjectedGetValue` ×4 and `HAL_GPIO_WritePin` ×2 (IRQ-path
  follow-up).
- Verification: `pio run` OK; sims unchanged. Bench (standard profile): ISR cycles Align
  1051 → 766, I-f 1809 → 1275, transition 1705 → 1213, running median 1844 → 1338, run max
  2267 → 1684 (of 3600). Holds 1500 / 2500 rad/s el within ±1 rad/s, lock never dropped, no
  break, calibration Rs 0.022794. Hardware-verified for this profile.

## 2026-10-27: FOC_Update speed-up (precomputed constants, inlined hot path)

- Precomputed instead of per period: `PI_Controller_t.Ki_Ts` / `Kb_Ts` (in `PI_Change_Params`;
  also the eSMO PLL), `FOC_t.ipark_adv` (IPARK_DELAY_COMP·Ts), `harm_k` (HARM_KI·Ts),
  `mtpa_k` / `mtpa_inv_4l` (MTPA, no division), `SMO_t.esmo_ratio_ts` / `esmo_x_lo` (LPF
  branch in the wc·Ts domain), `ADC_2_CURRENT` with the current LPF alpha folded in (one
  multiply + one FMA per phase). `FOC_Precompute()` refreshes the FOC fields from `FOC_Init`
  and `FOC_Recalc_Params`; `SMO_BLDC_Specs_Update` refreshes the PLL's `Ki_Ts`.
- Inverse-Park angle is wrapped once before the transform: the advance used to push it past
  2π (or below 0) for a few % of periods, which took the noinline `Normalize_Angle` path.
- Inlined into `FOC_Update` (`FOC_INLINE` = always_inline): PI step, Park, inverse Park, SVPWM,
  closed-loop step. The public functions remain as wrappers (the sims use them).
- `Harm_Phasor`: 7 → 4 divisions (one reciprocal of the RL denominator; the positive
  |1+CG|² scale is dropped because only the phasor direction is used).
- Verification: `pio run` OK (RAM 55148 B, Flash 56928 B). `tools/sim`: `esmo_sim`, `speed_sim`,
  `svpwm_check`, `pi_windup`, `math_check`, `normalize_check` identical; float rounding from the
  reassociated products shifts a few last digits elsewhere (`startup_sim` coast case: max
  current-vector jump 155.7 → 155.5 deg; real-motor case: RUNNING at 1.2623 instead of 1.2624 s,
  peak 3.0 → 3.1 A as printed; `calib_sim` currents ±1 µA). (Corrected 2026-10-27: this entry and
  commit dbf5e60 first said "identical", because the shell's `diff` wrapper hid the
  differences; checked with `cmp`.) Bench, two runs of the standard
  profile: ISR cycles Align 1133 → 1051, I-f 1907 → 1809, transition 1796 → 1705, running
  median 1954 → 1844, run max 2546 → 2267 (of 3600). Holds 1500 / 2500 rad/s el within
  ±1–3 rad/s, lock never dropped, no break; i_d means within the 50 Hz log's sampling spread.
  Hardware-verified for this profile.
- Not done (outside `FOC_Update`, proposed): executing the hot path from CCM SRAM (0 wait
  states vs 2 for flash); trimming the HAL IRQ path (`HAL_ADC_IRQHandler` called twice by
  `ADC1_2_IRQHandler`, 4 `HAL_ADCEx_InjectedGetValue` calls, `HAL_GPIO_WritePin`), which
  `isr_cyc` does not even include. Calibration's `Fast_Sin` of a growing angle (slow path every
  period) is left as is.

## 2026-10-27: legacy SMO removed, eSMO is the only observer

- Removed the legacy `SMO_Update` (MathWorks-style SMO), `SMO_Sign`, the `SMO_USE_ESMO`
  switch and the `smo.update` function pointer. `ESMO_Update` is renamed `SMO_Update` and is
  called directly from `FOC_Update` / the startup state machine.
- Removed now-unused constants `F_LPF_BEMF`, `INTEGRAL_LIMIT_PLL` and `SMO_t` fields
  (`bldc`, `k_smo`, `bemf_gain_smo`, `lambda_smo`, `gamma_smo`, `inverse_gamma_smo`,
  `lambda_lpf`, `gamma_lpf`, `e_alpha_est`, `e_beta_est`, `last_i_*_error`, `last_e_theta`).
  The `ESMO_*` constants and `esmo_*` fields keep their names. The init printout now shows
  the eSMO coefficients (F, G, z_gain).
- Also removed the unused `SQRT3`, `SPEED_RATED`, `INTEGRAL_LIMIT_I` and `INTEGRAL_LIMIT_SPEED`
  (no code referenced them; RAM/Flash unchanged).
- `tools/sim/esmo_sim` lost its two legacy cases.
- Verification: `pio run` OK (RAM 55092 B / 84.1 %, Flash 56296 B). Sim: `esmo_sim`,
  `speed_sim` and five `startup_sim` cases print output byte-identical to the previous commit
  (except the coefficient line).
- Bench run 2026-10-27 (flashed and captured by Claude at the user's request, standard
  `RUN_FOC_TEST 1` profile, Vbus 11.3 V): Rs 0.022748 ohm, Ld/Lq signs (−,−), handover at
  1180 rad/s el; holds 1500 → 1499.4 (1499–1500) and 2500 → 2499.5 (2499–2500) rad/s el,
  i_d mean 44 / −26 mA, lock never dropped, no break, clean coast. ISR max 2546 of 3600 cycles
  (unchanged from before). Hardware-verified for this profile.

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
  `adc_v_c` (third shunt), `ADC_INJ_SMP` override (default 0, experiment showed
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
