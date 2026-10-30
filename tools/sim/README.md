# tools/sim — host-side simulations of foc-lib

These programs compile `Lib/foc-lib/foc_lib.c` **unchanged** for the PC and drive it
with a simulated inverter and motor. They are models, not hardware: rotor inertia
(2e-5 kg m^2), loads and sensor noise are assumptions. They are not part of the
PlatformIO build (`src_dir = Src`).

## Requirements

A host C compiler: gcc from MSYS2 (UCRT64), MinGW-w64 or WSL. Check with `gcc --version`.

## Build (from the repo root)

```bash
mkdir -p build-sim
for t in startup_sim esmo_sim calib_sim saliency_sim svpwm_check pi_windup math_check normalize_check speed_sim; do
  gcc -O2 -std=gnu99 -I Lib/foc-lib tools/sim/$t.c -lm -o build-sim/$t
done
```

Each tool `#include`s `foc_lib.c` (white-box: the library's internal functions and constants are
`static` / private to it), so do not also compile `foc_lib.c`.

PowerShell, one tool:

```powershell
gcc -O2 -std=gnu99 -I Lib/foc-lib tools/sim/startup_sim.c -lm -o build-sim/startup_sim.exe
```

HFI is always on (`FOC_HFI` 1): every start goes through `MOTOR_STARTUP_HFI` and the sim plants
are non-salient (Lq/Ld 1) unless an argument says otherwise, so the start falls back to Align -> I-f
after the ~0.11 s saliency probe. For HFI behaviour build with `-DHFI_K=0.14f` (the sim plant's
saliency signal; the default 0.30 is the bench value). `-DFOC_HFI=0` builds the eSMO-only library
(byte-identical to c4c36a9 on calib/saliency/speed/startup/esmo_sim).

Startup tunables can be overridden for an experiment without editing the header, e.g.
`-DOMEGA_FORCE=500.0f -DSTARTUP_ACCEL=800.0f -DI_STARTUP=4.0f -DBEMF_LOCK_COUNT=1000000`.
Also guarded: `-DPWM_FREQ=10000.0f` (compare PWM rates), `-DHARM_COMP=0` (6x harmonic
compensation off), `-DHARM_KI=...`, `-DALPHA_LPF_MEAS=1.0f`.

## Tools

Baselines below for the 2026-10-26 header (Turnigy SK3 3542-1250KV constants, 20 kHz PWM)
unless marked otherwise.

| Tool | What it checks | Baseline result |
|---|---|---|
| `speed_sim` | `startup_sim` in speed mode with the bench profile (1500 rad/s el, step to 2500 at 3 s); prints the TRUE plant current ripple in both holds. Env `HD`/`HP` (back-EMF -6/+6 harmonics), `LD_EST`/`RS_EST` (controller parameter mismatch), `XD` (extra delay), `J` | `0.002 1e-7 1 -1 5.5 1 0.63e-3 7`: holds 1500/2500, ripple sd 0.01 A; with `HD=0.04 HP=0.02`: 0.01 A (comp on) vs ~1.0 A (`-DHARM_COMP=0`) |
| `calib_sim` (dead time) | Env `VDT` (per-phase dead-time voltage) / `I0` (soft knee, A) add an inverter error to the calibration plant | `VDT=0.02 I0=0.6`: Rs 0.0222 for 0.0220 (two-point levels 0.08/0.14 V) |
| `startup_sim` | Whole drive: ADC (quantised, noisy) → `FOC_Update` → SVPWM duties applied one period later → PMSM dq model + inertia + friction + fan load. Prints state transitions, handover metrics, closed-loop angle error | `0.005 1e-6 1 -1 2.5` and `0.002 1e-7 ...`: RAMP→TRANSITION→RUNNING. `0.01 1e-5` and the old "heavy load" `0.03 1e-5`: coast after LOCK_TIMEOUT (I-f at I_STARTUP, ~0.020 N m, is too weak for these loads with the SK3 constants); the same with `-DFOC_HFI=0` (2026-10-30) |
| `esmo_sim` | eSMO alone on a speed-imposed PMSM (forward, reverse, ±30 % Rs/Ls, IPM, current LPF) | 6 lines with `drops=0`; 0.03–0.9 deg |
| `calib_sim` | Rs / Ld / Lq calibration on a blocked rotor. Arg: Lq/Ld ratio | Rs 0.0220 (true 0.0220); Ld 8.1 uH for 7 uH. (The blocked rotor cannot show the rotor-alignment swing that `RS_ALIGN_ROUNDS` handles on the bench.) |
| `speed_sim` (HFI) | Build with `-DHFI_K=0.14f` (the sim plant's saliency signal). `PROF=low` (1500, 150 from 2 s, 0 from 5 s, 1500 from 7 s); `PROF=bench` (the freertos.c TEST_PROFILE 2 schedule); `PROF=start` (150 from standstill, 1500 from 3 s); `SAT=<per A>` d-axis saturation (needed for the HFI polarity step; else it falls back to Align -> I-f); `IMAX` current cap; `SHADOW=1` (hfi_takeover false); `SEED=<deg>` offsets the tracker start (sign test; 180 = wrong pole, re-seeded by the takeover gate); `HALF=f` latches each duty a fraction f into the period (0.5 = bench); `VDT`/`I0` dead time. Prints each tracker/control switch and every 0.1 s the TRUE rotor-minus-control angle | `PROF=low HALF=0.5 ... 0.002 1e-7 1.33 -1 9 1 0.63e-3 7`: start falls back to Align -> I-f (no saturation), eSMO -> HFI at ~293 rad/s el, holds 150 and 0, back to eSMO at ~428, no fault; `SAT=0.01 PROF=start`: HFI start, polarity flipped, 0 -> 150 on HFI; `SEED=+-30` pulls in; `Lq/Ld 1`: probe stops the pulses |
| `speed_sim` (bus sag) | Env `VSAG=<t s>`: bus falls 12 -> 6 V over 50 ms from t (current-limited supply) | `VSAG=4.1 J=1.2e-4 HALF=0.5 ... 0.002 2.6e-7 1.33 -1 4.4 1 0.63e-3 7`: -> IDLE at 4.134 s (~7.96 V), no current spike |
| `saliency_sim` | Saliency sweep (`FOC_BLDC_Saliency_Point` at 24 angles + the `Src/freertos.c` fit) on a blocked salient rotor. Args: Lq/Ld ratio, rotor d-axis angle (deg el); env `VDT`/`I0` as `calib_sim` | `1.2 20`: min-L axis 19.9 deg, A2 0.57 uH (L1 0.7 uH true), A1 0.003 uH; `1.0 0` (non-salient): A2 0.005 uH |
| `svpwm_check` | Applied vs commanded voltage of `SVPWM_Modulation` over all angles | gain 0.9995–1.0005, angle error < 0.03 deg |
| `pi_windup` | `PI_Update` back-calculation anti-windup on an RL plant under saturation | integral 6.9 V at 50 ms, settles 1.2 ms after saturation ends |
| `math_check` | `Normalize_Angle`, `Fast_Sin/Cos/Sin_Cos`, `Fast_Atan2`, `Fast_Sqrt` vs double libm, incl. edge cases | sin/cos < 5e-6 for |a| <= 200; atan2 < 2e-6 rad; sqrt exact |
| `normalize_check` | `Normalize_Angle` reduction accuracy vs true 2pi | FMA path ~2.4e-7 rad; without FMA (host default) Cody-Waite to 800 rad |

## startup_sim usage

```
startup_sim <Tc_Nm> <kfan_Nm_per_(rad/s)^2> <Lq/Ld> <coast_time_s or -1> <t_end_s> [dir 1|-1] [plant_flux_Wb] [plant_pole_pairs]
```

- `Tc` Coulomb load torque, `kfan` fan/prop torque coefficient (mechanical rad/s).
- `coast_time_s` > 0 calls `FOC_Coast()` at that time and reports the winding current after it.
- `plant_flux_Wb` / `plant_pole_pairs` make the simulated motor differ from the library
  constants — use this to test a wrong `FLUX_LINKAGE` / `POLE_PAIRS` in the header.

Examples:

```bash
build-sim/startup_sim 0.03 1e-5 1 -1 2.5                 # heavy load, forward
build-sim/startup_sim 0.03 1e-5 1 -1 2.5 -1              # reverse
build-sim/startup_sim 0.2 0 1 -1 1.6                     # stalled rotor: must coast after LOCK_TIMEOUT
build-sim/startup_sim 0.005 5e-6 1 2.0 2.3               # coast at speed: winding current must stop
build-sim/startup_sim 0.002 1e-6 1 -1 2.5 1 0.63e-3 7    # real 3542 motor vs CURRENT header constants -> coasts out (the stall)
```

To simulate the library with corrected constants, build against an edited copy of
`foc_lib.h` (e.g. `POLE_PAIRS 7`, `FLUX_LINKAGE 0.00063f`) and pass the same plant values.
2026-10 result: corrected constants hand over and run at `OMEGA_FORCE` 300 and 500
(`STARTUP_ACCEL` 800); 1000 / 2000 falls out of step under a prop-like load at 3 A.

## Timing model (matters for the delay compensation)

The simulation applies the duties computed in ISR k during the PWM period that starts at
k+1 (one-period shadow-register delay) and samples currents at the start of each period.
`ESMO_DELAY_COMP` / `IPARK_DELAY_COMP` were tuned under this assumption at `PWM_FREQ`
10 kHz and re-checked at 20 kHz (eSMO angle error halves). If the real TIM1 / ADC timing
differs, the simulation and the bench will disagree on the residual angle error — trust the
bench. (The 20 kHz bench run and the harmonic compensation both behaved as simulated.)
