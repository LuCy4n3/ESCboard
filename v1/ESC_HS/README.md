# ESC_HS

**ESC v1.** This project is stored in `v1/ESC_HS/`. The project name remains
`ESC_HS`. See the [repository version index](../../README.md) for both generations
and [ESC_HF v2](../../v2/ESC_HF/README.md) for the newer hardware design.

Run the commands in this guide from the `v1/ESC_HS/` project directory.

Experimental sensorless brushless-motor ESC firmware for an STSPIN32G4-based
power stage. The STM32CubeIDE project targets the STM32G431VB MCU and combines
high-frequency-injection (HFI) startup, six-step acquisition, field-oriented
current control, a back-EMF observer and UART diagnostics.

**The current firmware initializes its speed target to 2000 RPM and can start the
motor after power-up without a UART command.** Before powering the bridge, review
`FM_TARGET_RPM` in `Core/Inc/fast_motor_config.h`. Set it to `0.0f` and rebuild if
you require a zero target at startup. This cleanup preserves the firmware's
existing behavior and configuration.

## Hardware and BOM status

The supplied KiCad project is in `esc_design/`. See [hardware/BOM.md](hardware/BOM.md)
for the source-derived BOM, component counts and unresolved design fields.
The [Excel BOM](hardware/outputs/esc_hs_bom/BOM.xlsx) includes grouped parts, the
component inventory and review findings. CSV exports are also available in `hardware/`.
The PCB has a duplicated `R34` reference, and several parts still need exact values
or part numbers confirmed before purchasing.

The v2 `ESC_HF` hardware uses STM32H750 + DRV8323 and is a different
design. Its schematic and BOM must not be used as the ESC_HS hardware definition.

## Project contents

| Path | Purpose |
| --- | --- |
| `Core/Src/main.c` | Peripheral setup, UART command parser and foreground service. |
| `Core/Src/fast_motor.c` | Motor state machine, control, protection and capture logic. |
| `Core/Inc/fast_motor_config.h` | Motor model, sensing scales, limits and experiment switches. |
| `Core/Src/stspin_driver.c` | STSPIN gate-driver configuration over I2C. |
| `Core/Src/telemetry.c`, `Core/Inc/telemetry.h` | Telemetry serialization and address definitions. |
| `Core/Src/v13_lean.c`, `Core/Inc/v13_capture.h` | V13 current-filter experiment and RAM capture format. |
| `Drivers/` | Bundled STM32 HAL and CMSIS sources, including their license files. |
| `ESC_HS.ioc` | CubeMX peripheral configuration. |
| `.project`, `.cproject`, `.mxproject` | STM32CubeIDE project metadata. |
| `STM32G431VBTX_FLASH.ld` | Linker script: 128 KiB flash and 32 KiB RAM. |
| `tools/v13_shadow20k.py` | Capture/export tool matching the current 20 kHz format. |
| `V13_SHADOW_README.md` | Historical build-26101613 notes; some settings and commands are obsolete. |
| `esc_design/` | ESC_HS hierarchical KiCad schematic, PCB and local MOSFET footprint library. |
| `hardware/` | BOM exports, source reconciliation and design findings. |
| `tools/export_bom.py` | Rebuild the CSV BOM and report from a KiCad XML netlist. |

## Current source configuration

The authoritative settings are the active source files, rather than the older
README or the locally retained `Debug/` executables.

| Setting | Current source value |
| --- | --- |
| `FM_FIRMWARE_BUILD_ID` | `26101703` |
| MCU system clock | 170 MHz |
| `FM_PWM_HZ` | 40 kHz PWM/control (25 us period) |
| `FM_RUN_RAW_OBSERVER_20KHZ` | `1`: RUN observer experiment at 20 kHz |
| `FM_V13_20K_SHADOW_TEST` | `1`: separate 20 kHz V13 shadow experiment |
| `FM_V13_RUN_OBSERVER_ASSIST` | `0` |
| UART | USART1, 460800 baud, 8 data bits, no parity, 1 stop bit |
| `FM_TARGET_RPM` / `FM_MAX_RPM` | +2000 / 32000 RPM |
| `FM_POLE_PAIRS` | 7 |
| Current conversion assumptions | 3.3 V ADC reference, 12-bit ADC, gain 5, 5 mOhm shunts |
| Bus-voltage conversion | 21:1 divider |

These values describe the checked-in configuration, not verified ratings for an
unknown board or motor. The motor model is labeled as an A2212/5T 2450 KV starting
model in the source. Review and measure the parameters for the actual hardware.

## Build and flash

1. Install STM32CubeIDE with STM32G4 support and connect an appropriate ST-LINK/SWD
   programmer. The local installation found during cleanup is CubeIDE 1.18.0.
2. In CubeIDE, use **File > Import > General > Existing Projects into Workspace**
   and select this directory. Keep the name `ESC_HS` because project metadata uses it.
3. Select the intended Debug or Release configuration. Review the MCU, linker
   script and motor/sensing settings before building.
4. Use **Project > Clean**, then **Build Project**. Confirm the current sources,
   including `v13_lean.c`, are compiled and the build fits the linker's memory limits.
5. With the power stage in a controlled bench setup, use the IDE's debug/flash
   workflow to program the freshly built ELF. Confirm the reported firmware build
   ID is `26101703` for this source configuration.

Generated outputs normally appear in `Debug/` or `Release/`. Existing binaries are
kept locally and ignored by Git; their presence does not prove they match the source.
The project contains custom peripheral/interrupt code, so review diffs carefully
after regenerating code from the `.ioc` file.

## Connections and initial use

The following are MCU signal assignments from the project. They are not connector
pin numbers; confirm the board wiring using the matching schematic.

| Function | Source assignment |
| --- | --- |
| UART transmit / receive | PA9 / PA10 (USART1 TX / RX) |
| Gate-driver configuration | PC8 / PC9 (I2C3 SCL / SDA), 7-bit address `0x47` |
| Phase PWM pairs | PE9/PE8, PE11/PE10, PE13/PE12 (TIM1 CH1/1N, CH2/2N, CH3/3N) |
| DC bus sensing | PF0, ADC1_IN10 |
| Current-sense paths | OPAMP1/2/3 and ADC2 injected sampling; verify phase order on the board. |

Use a UART adapter compatible with the MCU's 3.3 V logic, cross TX/RX, and connect
the appropriate signal ground. Close other serial programs before opening the
same COM port. The command interface uses binary bytes, not typed ASCII text.

For first operation, use a current-limited supply, secure the unloaded motor and
remove any propeller. Review the nonzero startup target before applying power.
Watch state, fault, bus voltage, phase-current and ISR-overrun telemetry during
commissioning. A firmware trip threshold is not a certified power-stage rating.

## UART commands

Each command is exactly five bytes. Speed is a signed 16-bit RPM target in
big-endian two's-complement form. `main.c` accepts targets from -32000 to +32000.

| Operation | Hex bytes |
| --- | --- |
| Request zero speed / bridge-off idle | `15 07 BB 00 00` |
| Set +1000 RPM | `15 07 BB 03 E8` |
| Set -1000 RPM | `15 07 BB FC 18` |
| Arm a new V13 RAM capture | `15 07 BC 00 00` |

A zero target requests the firmware's idle/coast behavior; it is not an emergency
power disconnect. Closing a serial program does not send a zero target or stop
the motor. Earlier notes referring to command byte `01` do not describe this parser.

Telemetry consists of seven-byte records: `AA 55`, one address byte, and a
little-endian 32-bit payload. Channels serialized with `append_signed_offset`
use an XOR sign bias of `0x80000000`; other channels use their defined unsigned
or integer representation. Consult `telemetry.c` for the active channel set and
`telemetry.h` for addresses and units. Not every defined channel is emitted in
every compile-time mode. Periodic telemetry is nominally every 50 ms.

## V13 capture and offline analysis

The current RAM format contains 256 consecutive 20 kHz frames (12.8 ms). Each
32-byte frame includes the ISR tick, raw and filtered phase currents, PWM command
registers, bus voltage, filter diagnostics, cycle count and flags. Current values
are quantized at 4 mA/count. Capture collection requires the appropriate RUN mode;
arming a capture does not start or stop the motor.

Create a Python environment and an output directory:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/requirements.txt
New-Item -ItemType Directory -Force captures
```

With the motor already commissioned and running in the intended mode, acquire a
capture using the actual serial port:

```powershell
.\.venv\Scripts\python.exe tools/v13_shadow20k.py --serial COM4 --arm --seconds 20 --out captures/v13_20k.csv
```

The tool sends only the capture-arm command, receives UART bytes and writes a CSV,
raw `.bin` and `.summary.json`. Serial export sends one RAM frame per telemetry
period, so collecting a full buffer takes about 12.8 seconds after it is ready;
allow more time if RUN has not been reached. Check that the report contains all
256 samples and inspect discontinuities/timing before interpreting results.

Decode a previously recorded raw UART stream without opening a serial port:

```powershell
.\.venv\Scripts\python.exe tools/v13_shadow20k.py --bin captures/v13_20k.bin --out captures/v13_redecoded.csv
```

Do not pass an arbitrary MCU binary or a different capture format to `--bin`.
The neighboring 1024-sample/1 kHz decoders and historical capture profiles target
different formats. Numerical summaries describe signal differences and noise
proxies, not true-current estimation accuracy without an independent reference.

## Git layout and verification

The local repository includes active firmware, HAL/CMSIS sources and licenses,
CubeIDE project files, documentation, the hardware design/BOM and the matching
capture tool. Generated build trees, machine-specific language settings, `.orig`
and `.pre_zc` snapshots, Python environments and `captures/` remain local and ignored.

No remote or initial commit is created by the cleanup. Original firmware and local
build/snapshot files are preserved. Documentation/configuration and offline-tool
checks do not replace an ARM build, flashing, timing measurements or motor tests.
