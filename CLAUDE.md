# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

The canonical project instructions live in `AGENTS.md` (shared with other agents) and are imported here:

@AGENTS.md

## Notes for Claude Code

- `AGENTS.md` and `STATUS.md` are hand-maintained and drift from the code. Treat their line numbers, constant values and file sizes as hints and confirm against the source before relying on them. In particular, the PWM/ISR rate is whatever `PWM_FREQ` in `Lib/foc-lib/foc_lib.h` says now, not the figure quoted in the docs.
- Git history starts on 2026-10-26; earlier changes are recorded only in `STATUS.md`. After any non-trivial change, update `STATUS.md` too: what changed, and whether it is compile-verified, simulation-verified (`tools/sim`) or hardware-verified. Never mark anything hardware-verified unless the user reports a bench run.
- Verify with `pio run` and report the RAM/Flash line it prints. Build output is short; grep for `RAM:|Flash:|SUCCESS|FAILED|error`.
- Serial console: USART2 via the ST-Link VCP, **230400 bps 8N1, `COM5`** on the user's Windows 11 host (`pio device monitor`; speed and port are set in `platformio.ini`). If you read the port yourself, do not leave it open while the user runs the monitor or `pio run -t upload` — only one program can hold COM5.
- The ISR drives PC8 high for the length of `HAL_ADCEx_InjectedConvCpltCallback`, so ISR execution time can be measured on a scope at that pin. Keep this when editing the callback.
- Motor/current/calibration constants (`RS`, `LS`, `POLE_PAIRS`, `I_RATED`, `I_PEAK`, `INJECTION_VOLTAGE`, `INJECTION_MAX_CURRENT`, ...) sit together near the top of `foc_lib.h`. Changing injection voltage or current limits changes real current through the bench motor, so state the resulting current at the assumed `RS` when you propose such a change. Since the 2026-10 SVPWM fix the applied voltage equals the commanded voltage, so the DC injection current is about `INJECTION_VOLTAGE / RS` minus an inverter voltage error (bench at 20 kHz: 0.08 V → 2.7 A, 0.14 V → 5.3 A). Rs is measured at two DC levels (`INJECTION_VOLTAGE_RS_LOW`, `INJECTION_VOLTAGE`) and taken from the slope, which cancels that error only where it is constant — keep both levels well above the dead-time knee when changing either. The low level first holds for `RS_ALIGN_ROUNDS` (0.3 s) so the rotor finishes swinging into alignment; recording earlier biased Rs by up to ±30 %. `I_ALIGN` and `I_STARTUP` are commanded currents (current-controlled), not voltages.

## Hand-off context (updated 2026-10-26)

- **The motor runs sensorless closed loop on the bench** (Align → I-f → Transition → Running, torque mode, no load). The earlier stall was inverted current sensing (fixed with `CURRENT_SENSE_POLARITY`) plus motor constants for the wrong motor (now Turnigy Aerodrive SK3 3542-1250KV, 7 pole pairs confirmed from back-EMF). Bench steps 2–5 of `STATUS.md` passed; read its "Bench 2026-10-26" section first.
- Unloaded torque mode accelerates past `ESMO_OMEGA_MAX` and coasts at ~1.5 s — expected, not a bug.
- **Speed control works on the bench** (holds 1500 and 2500 rad/s el within ~1 rad/s of the estimate). It runs in `FOC_Speed_Control()` inside `FOC_Update` at 1 kHz, not in a task, and is **motoring only** (no regenerative braking — user decision for a bench supply that cannot sink current; keep it unless the user changes that). `V_BUS_MAX` trips on bus over-voltage. PWM is 20 kHz; the 6x current ripple from the motor's back-EMF harmonics is handled by `FOC_Harmonic_Comp()` (bench: −75–80 %). Calibration at 20 kHz reads Rs 0.02275 Ω repeatably (spec 0.022).
- Prefer instrumenting over guessing: `StartDefaultTask` logs the drive state at 50 Hz during the run (integers only — the task has a 512 B stack). Never `printf` from the ISR.

## Running the bench yourself

The user has let Claude flash and capture runs directly (motor clamped, supply current-limited). Confirm before a run that spins the motor in a new way.

- Capture: open COM5 at 230400 with the PlatformIO Python's pyserial **before** `pio run -t upload` (upload goes over SWD, not COM5), read for a fixed time, then close the port.
- Live state without halting: `tool-openocd` (`interface/stlink.cfg`, `target/stm32f3x.cfg`) with `mdw` reads RAM/peripherals while the target runs. The ELF has no debug info: get `FOC_t` field offsets by compiling a probe with `arm-none-eabi-gcc -g` and `arm-none-eabi-gdb` (`p &((FOC_t*)0)->field`), and the `foc` base address from the firmware ELF.
- `RUN_FOC_TEST` in `Src/freertos.c` (USER CODE PD) selects calibration-only (0) or calibration + 5 s run (1).

## Guard rails for `foc_lib`

- **eSMO vs TI SPRUJ26A**: the code intentionally differs from the document's Eq. 11/17 (sign of z), 15/16 (sign of the lag compensation), 18–20 (missing Ts) and 18/19 (Lq on beta). Each literal version was simulated and fails. Do not align the code with the document.
- `ESMO_DELAY_COMP` and `IPARK_DELAY_COMP` depend on `PWM_FREQ` and on when TIM1 latches new duties (tuned at 10 kHz, re-checked in sim at 20 kHz — the current `PWM_FREQ` — with a one-period shadow delay). Re-tune if either changes; bench check: `i_d` ≈ 0 at constant speed with `id_ref = 0`.
- The startup lock check uses `FLUX_LINKAGE` (`EMF_PLAUSIBLE_RATIO`). A wrong flux constant makes the startup coast out after `LOCK_TIMEOUT`. When changing motor constants, also re-check `OMEGA_FORCE` (back-EMF at handover should be well above `MIN_BEMF_AMPLITUDE`, ideally ≥ 0.25 V) and that `I_STARTUP` gives enough torque (`1.5 * POLE_PAIRS * FLUX_LINKAGE * I_STARTUP`).
- `foc->outputs_enabled == false` must disable the PWM outputs in the HAL. Do not reintroduce a path where `FOC_Update` runs the modulator while outputs are disabled. The HAL side is `Bridge_Outputs()` in `Src/freertos.c`: it switches the six gate pins to GPIO at their off levels. **Never coast by clearing TIM1 MOE**: the injected ADC is triggered by the OC4 output, which MOE also gates, so the control ISR stops (calibration hangs). MOE is left to the over-current break.
- Current-sense polarity is board-specific: the IHM08M1 reads below the offset for current into the motor, hence `CURRENT_SENSE_POLARITY -1`. Calibration cannot catch a wrong sign (Rs/Ld/Lq go through `fabsf`); the freertos.c "signed means" line and the signed `Ld_est/Lq_est` print can — expect `i_a > 0`, `i_b ≈ -i_a/2`, Ld/Lq signs (−,−).
- The IHM08M1's L6398 low-side input (LIN) is active-low. TIM1 low-side idle state must be SET and automatic output (AOE) disabled — now in `stm32-foc.ioc`, and also forced at runtime in `TIMER_config()`. Otherwise MOE = 0 turns every low side on (brakes) and AOE re-enables the bridge after a break.
- Keep the `TRIG_NOINLINE` slow paths, the bit-pattern range test in `Trig_In_Range`, and the `__FP_FAST_FMAF` branch in `Normalize_Angle`; they are measured optimisations, not style choices. No `-ffast-math`.
- Do not write `smo.theta_est` from outside the observer; the transforms use `foc->theta`.
- ISR budget at 20 kHz is 3600 cycles; the measured max is ~2600 (`isr_cyc` in the run log). Measure again after adding anything to the ISR path, and keep the 1 kHz work (speed loop, harmonic phasors) in separate periods.
- Do not raise `CURRENT_CONTROL_F_BANDWIDTH` toward `PWM_FREQ/10` (2 kHz): it puts the 6x harmonic on the current loop's sensitivity peak (sim: 3x the ripple at 2500 rad/s el).
- After changing `foc_lib.c/.h`, if a host gcc is available, run the relevant `tools/sim` checks (`tools/sim/README.md`) in addition to `pio run`.
