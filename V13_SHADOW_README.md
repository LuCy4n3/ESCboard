# STSPIN32G4 — Stage B: V13 Lean shadow capture

**Build ID:** `26101613` | **Only diagnostic V13 output; original control remains raw-current-based.**

## IMPORTANT: not an exact V13 reference yet

The supplied file `stspin32g4_v13_lean(1).m` calls an external function
`v12_current_activity(adcABC)` which was not supplied. This function produces
`alphaSchedule(k)`, essential for the mathematically exact original output.
The supplied original MATLAB file is preserved under `reference/`.

This firmware faithfully ports the V13 **update body** (8-sample confirmation,
V3 fast/drift/confidence, coherent early residual, electrical-only voltage
events, three-phase alpha/beta reconstruction) but uses a temporary, explicit
`V13_SHADOW_BASE_ALPHA = 0.008f` as its baseline gain. That is **NOT** an
implementation of `v12_current_activity`. Firmware telemetry
`v13_reference_complete` = **0** intentionally, and must remain 0 until
that missing function is supplied, ported, and numerically verified.
Do not draw a conclusion about the genuine V13 vs raw or about current-loop
suitability based solely on this interim approximation.

## Firmware architecture, observed in this source tree

- `TIM1`: 40-kHz center-aligned PWM; CH4/TRGO drives ADC2 injected conversion.
- `ADC2`: injected rank 1/2/3 read JDR1/2/3 in `FastMotor_ADC2InjectedIRQ`;
  firmware's current U/V/W mapping is rank 1/2/3. ADC2 channels are
  IN1 / IN10 / IN11; the `.ioc` maps OPAMP1/2/3 pins separately, so their
  physical external wiring cannot be proven solely from firmware.
- Injected-current offset calibration uses a disabled bridge and the same
  ADC2 injected trigger. Scaling is from `fast_motor_config.h`:
  gain 5, 5 mOhm, `3.3/(4095*5*.005)` A per ADC count.
- Protection operates on **unfiltered** ADC converted currents and retains
  original rail, current, current-sum and bus-voltage checks.
- HFI scan/converge → true six-step slingshot/acquisition → BEMF/current-loop
  FOC boost → observer handover → RUN. HFI and PM-flux/BEMF rotor estimation,
  angle blending, PI regulators and speed reversal remain unchanged.
- Raw `clarke(iu, iv)` still feeds motor control. `V13_Update` only updates
  separate `g.v13_shadow` state and diagnostic data, after protection.

## Sampling and PWM alignment limitations

- Default `FM_V13_SHADOW_DECIMATION=1` -> one filter step at every 40-kHz ADC
  callback (25 microseconds per period; **4250 total core cycles** at 170 MHz).
  At this rate filter settling/slew may differ greatly from the original 1-kHz
  study. The constant decays are precomputed at filter init using actual fs.
- `dutyABC` uses TIM1 CCR1/2/3 **read before control updates**. PWM preload
  means a CCR register read is a *previous-command proxy*, not proof of the
  active physical compare latch at the ADC aperture. A hardware trace or
  explicit PWM latched-data alignment is still needed for strict equivalence.
- In true six-step one phase is Hi-Z. It has no physically meaningful
  phase PWM duty. Voltage innovation is intentionally masked until all three
  phase legs drive normally, to avoid false electrical evidence. This is a
  protective diagnostic deviation from MATLAB's fully defined duty input.
- 1-kHz mirror inside ADC callback stores one simultaneous raw/V13 sample;
  UART sends at nominal **100 snapshots/s**. The trace is **not** a continuous
  recording at the 40-kHz ADC rate; spikes between samples can be missed.
  V13 max cycles and full ISR max/overrun counters catch events between frames.

## Telemetry setup

Use the separate `commread_config_v13_shadow.json` provided alongside ZIP.
29 channels x 7 bytes per channel = 203 bytes per snapshot. At 100 Hz,
20300 bytes/s; approximately 44% of the nominal 460800 baud 8N1 serial line.
The original AA 55 / 1-byte address / LE uint32 protocol is unchanged.
Signed channel values use XOR 0x80000000 before sending. Firmware's normal
RX `15 07 01 HI LO` speed command remains unchanged.

Important fields:

- `firmware_build_id`: **26101613** confirms correct executable.
- `v13_reference_complete`: **0** until supplied baseline activity function
  is ported and all-sample equivalence is checked.
- `phase_u/v/w_current_a` vs `v13_phase_u/v/w_a`: same 1-kHz diagnostic tick.
- `v13_alpha`, `v13_confidence`, `v13_step_latch`, `v13_voltage_evidence`:
  V13 internals; `v13_voltage_valid` is 0 in six-step/bridge-disabled mode.
- `v13_cycles_last`, `v13_cycles_max`, `isr_cycles_last`, `isr_cycles_max`,
  `isr_overrun_count`: DWT timing and missed 25-us deadline indication.
- `v13_decimation` and `v13_sample_index`: verifies effective update rate.

**CAUTION:** Address 179–181 in the previous capture JSON stood for
`phase_u/v/w_applied_v`; they are now **shadow current values**. Load the new
JSON; never decode this firmware using the old profile. Existing production
37-channel telemetry can be restored by changing
`TELEMETRY_V13_SHADOW_CAPTURE` in `telemetry.h` to 0 (20 Hz), but that profile
is not designed to capture V13 currents.

## Build and commissioning

1. Import this folder as an existing STM32CubeIDE project and do **Clean +
   Build**, checking that `Core/Src/v13_lean.c` is compiled. No stale Debug
   binaries are provided in the new ZIP to prevent flashing the old ELF.
2. Verify full target compilation/link and compare flash and RAM before
   uploading. This environment has no `arm-none-eabi-gcc`, so **no ARM ELF/HEX
   was built here**. Host compiler syntax checks and native smoke test were
   performed (see `tests/`).
3. Initially power from a current-limited supply, **motor unloaded/without
   propeller**, and avoid high-current or high-speed runs until ISR time is
   measured. Confirm `firmware_build_id`, `v13_reference_complete=0`, and
   that the original raw-current controller runs correctly.
4. Watch `isr_overrun_count` and `isr_cycles_max` closely. If adding V13 at
   40 kHz violates 4250-cycle deadline, stop the test and change
   `FM_V13_SHADOW_DECIMATION` to 2, 4 or 40 for **diagnostic only** (corresponding
   to 20, 10 or 1 kHz). Reduced-rate results **cannot** be treated as 40-kHz
   filter behavior; each rate requires a clean build and retest.
5. Capture stationary, very slow, low speed, transitions, moderate/high RPM,
   both directions; keep full original safety protections. Keep track of
   unloaded vs loaded runs and supply settings.
6. Send the Excel/CSV telemetry and missing `v12_current_activity.m`.
   With that helper, the next step is exact baseline port + deterministic
   MATLAB/C replay comparisons and gain/latency evaluation on your captures.

## Static/native verification completed here

- Compiled `v13_lean.c` with native GCC C11 `-O2 -Wall -Wextra -Werror`.
- Native smoke: deterministic output, input NaN containment, state reset,
  zero-sum phase-current reconstruction (PASS).
- Host GCC C syntax-checked the modified `fast_motor.c`, `telemetry.c`,
  `v13_lean.c` against the supplied STM32 headers (PASS); this does **not**
  replace target build, linking, profiling, or physical motor testing.
- Confirmed exactly 29 emitted telemetry address fields match the 29 JSON
  channel addresses, without duplicates.
