# Experimental build

This ESC firmware is a development build for bench testing, sensorless startup
research, current-filter diagnostics and real-time workload measurements.
The current configuration reports firmware build ID `26101703`.

The active source files in `Core/` define the experiment. Older binaries, source
snapshots and captures may describe different configurations. Build identifiers
help match a recording to its firmware; they do not establish hardware validation.

## Current experiment

- PWM, current control and protection run at 40 kHz.
- The RUN raw-current observer experiment updates at 20 kHz.
- A separate current-filter experiment computes at 20 kHz in shadow mode.
- Filter-to-observer assist is disabled in the current configuration. The shadow
  output is recorded for analysis rather than used as current-loop feedback.
- Raw measurements continue to feed the control and protection paths.

Review `Core/Inc/fast_motor_config.h` before changing experiment switches, motor
parameters, sensing scales or protection limits. Source filenames and internal
identifiers are preserved so the documentation update does not alter the build.

## Startup and commissioning

The current default speed target is **2000 RPM**. The motor can start after power-up
without a UART command. To require a zero target at startup, set `FM_TARGET_RPM`
to `0.0f` and rebuild before powering the bridge.

Use a controlled bench setup with an unloaded, secured motor, no propeller and a
current-limited supply appropriate to the confirmed hardware. Monitor fault,
bus-voltage, current, observer and ISR timing telemetry. A software trip setting
does not establish the board's electrical or thermal rating.

## Diagnostic capture

The experimental RAM buffer stores 256 consecutive 20 kHz updates, covering
12.8 ms. Each 32-byte frame contains raw and filtered phase currents, ISR timing,
PWM command-register values, bus voltage, filter diagnostics and flags.

The five-byte command `15 07 BC 00 00` arms a new capture. Collection requires
the appropriate RUN mode. Arming or closing the capture tool does not stop the
motor. Export uses the normal binary UART stream at 460800 baud, 8N1.

See the [usage guide](README.md#experimental-capture-and-offline-analysis) for
tool discovery, acquisition and offline decoding commands. Those commands keep
the supplied tool filename intact while describing the operation as an experiment.

## Interpreting results

A complete export contains 256 frames. Check sample continuity, capture status,
execution cycles and overrun counters before comparing waveforms. The PWM fields
are command-register snapshots, not measurements of the physical phase voltages.

Signal differences, spectral energy and variation metrics can describe the
recorded behavior. They do not establish true-current estimation accuracy without
a synchronized independent reference. This short capture also limits the
precision of low-speed phase-lag measurements.

Changes to sampling rates, observer scheduling or filter execution require a
fresh target build and timing/hardware checks. The documentation update does
not modify firmware behavior, numerical constants, protection or wire formats.
