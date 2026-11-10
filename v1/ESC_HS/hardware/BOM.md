# ESC_HS bill of materials

Generated from the supplied `esc_design/esc_design.kicad_sch` hierarchy
and reconciled against `esc_design/esc_design.kicad_pcb`. Values and
footprint identifiers are preserved as authored. The earlier provisional
hardware checklist is replaced by this source-derived BOM.

## Files

- [Excel BOM](outputs/esc_hs_bom/BOM.xlsx): grouped parts, component inventory and review findings.
- [Grouped CSV](BOM.csv): quantities, references, values, footprints and confirmation notes.
- [Component CSV](BOM_components.csv): each schematic reference, including exclusions and PCB counts.
- [Source data and reconciliation](bom_data.json): extracted fields, findings and source hashes.

## Counts and population rules

- 148 schematic component references across 10 sheets.
- 149 PCB footprints. Duplicate references: R34.
- 140 schematic BOM entries in 36 groups.
- 8 references excluded by the schematic BOM flags. DNP references: 0.
- Schematic-only references: none. PCB-only references: none.

Quantities follow the schematic population flags. The duplicate PCB `R34`
is reported separately and does not silently add another schematic part.
`P1`–`P5` are PCB copper connection pads included by the source BOM flags,
not five separately purchased components. Four mounting holes (`H1`–`H4`)
and four solder jumpers (`JP1`–`JP3`, `JP5`) are excluded. `JP4` remains included.

## Items requiring confirmation

| References | Finding |
| --- | --- |
| D1, D2, D3, D4, D5, D6, D7, L1, R16, R17, U10 | Exact value/part requires selection |
| P1, P2, P3, P4, P5 | PCB copper connection pad; no separate component to purchase |
| Q1, Q2, Q3, Q4, Q5, Q6 | Missing component value; Symbol IRFB7530PbF vs footprint IRFS7530PBF: confirm package/part |
| Q7 | Value SQ3427CEV vs symbol/datasheet Si2319CDS: confirm selected device |
| R34 | Duplicate PCB reference (2 footprints); schematic quantity retained |
| U2 | Schematic/PCB footprint mismatch |

`U2` schematic footprint: `SMA_STM`. PCB footprint:
`STPS1H100AY:SMA_STM`. This identifier comparison does not establish a geometry difference.

Duplicated `R34` locations in the supplied PCB coordinates: F.Cu at (111.79555, 72.575001) mm; F.Cu at (117.325, 71.775) mm.

The BOM preserves specific part-like values such as `STSPIN32G4`,
`WSLP27265L000FEA`, `MAX74810ARMZ-RL` and `TMUX6136PWR`. An explicit MPN
field is left blank where the design has not populated it. No supplier
stock, pricing, approved replacements or missing component ratings are invented.
Passive voltage/power ratings and tolerances, and the unresolved generic
parts above, must be selected before this becomes a purchasing BOM.

## Design portability and review scope

`U1` is explicitly STSPIN32G4, matching the firmware family. This is the
ESC_HS design, separate from the neighboring STM32H750/DRV8323 ESC_HF board.
The included `sym-lib-table` refers to an absolute external path. Only the
IRFS7530PbF custom footprint library is bundled; the other custom footprint
definitions are embedded in the PCB. Bundle external libraries and make
their paths project-relative for portable editing.

The review covers component fields, hierarchy, population flags and
schematic/PCB reconciliation. It does not include full electrical ERC,
layout DRC, device-rating approval or hardware validation. Firmware and
schematic/PCB files have not been altered.

## Regenerate CSV and report

From the `v1/ESC_HS/` project directory in PowerShell:

```powershell
New-Item -ItemType Directory -Force captures
& 'C:\Program Files\KiCad\8.0\bin\kicad-cli.exe' sch export netlist --format kicadxml --output captures/bom_netlist.xml esc_design/esc_design.kicad_sch
python tools/export_bom.py --netlist captures/bom_netlist.xml --pcb esc_design/esc_design.kicad_pcb --out hardware/bom_data.json --csv
```

Adjust the KiCad executable path for your installation. These commands refresh
the CSV files, JSON and this report. The Excel file is a snapshot of this
export and must be refreshed separately after design changes.

## Grouped source BOM

| Qty | Value | References | Footprint |
| ---: | --- | --- | --- |
| 16 | 100n | C1, C2, C3, C5, C7, C9, C10, C11, C15, C18, C20, C21, C22, C23, C24, C25 | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder |
| 2 | 10u | C4, C16 | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder |
| 3 | 1u | C6, C8, C17 | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder |
| 3 | 4n7 | C12, C13, C14 | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder |
| 1 | 220u | C19 | Capacitor_THT:CP_Radial_D12.5mm_P5.00mm |
| 3 | 1u | Cboot1, Cboot2, Cboot3 | Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder |
| 4 | 220n | CM1, CM2, CM3, CM4 | Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder |
| 1 | 10u | Cout1 | Capacitor_SMD:C_1206_3216Metric |
| 7 | D | D1, D2, D3, D4, D5, D6, D7 | Diode_SMD:D_1206_3216Metric_Pad1.42x1.75mm_HandSolder |
| 2 | Conn_01x05_Pin | J1, J3 | Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical |
| 1 | Conn_01x07_Pin | J2 | Connector_PinSocket_2.54mm:PinSocket_1x07_P2.54mm_Vertical |
| 1 | Jumper_2_Bridged | JP4 | TestPoint:TestPoint_2Pads_Pitch2.54mm_Drill0.8mm |
| 1 | WE-PD2_3521 | L1 | WE-PD2_3521:WE-PD2_3521 |
| 5 | ~ | P1, P2, P3, P4, P5 | Connector_PCBEdge:Conn_Power |
| 6 | ~ | Q1, Q2, Q3, Q4, Q5, Q6 | IRFS7530PbF:IRFS7530PBF |
| 1 | SQ3427CEV-T1_GE3 | Q7 | Package_TO_SOT_SMD:SOT-23 |
| 1 | BSS138 | Q8 | Package_TO_SOT_SMD:SOT-23 |
| 9 | 20k | R3, R5, R43, R58, R59, R64, R65, R69, R70 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 19 | 1k | R4, R21, R22, R23, R28, R29, R30, R35, R36, R37, R51, R52, R53, R57, R60, R62, R63, R67, R68 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 10 | 10k | R6, R45, R46, R47, R48, R49, R50, R61, R66, R71 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 6 | 10 | R7, R8, R10, R11, R13, R14 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 12 | 100k | R9, R12, R15, R19, R20, R26, R27, R33, R55, R72, R73, R74 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 1 | R | R16 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 1 | R | R17 | OptoDevice:R_LDR_4.9x4.2mm_P2.54mm_Vertical |
| 3 | 50k | R18, R25, R32 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 3 | WSLP27265L000FEA | R24, R31, R38 | WSLP27265L000FEA:RES_WSLP2726L2000FEA |
| 1 | 100k | R34 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 6 | 330 | R39, R40, R41, R42, R44, R54 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 1 | 150 | R56 | Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder |
| 1 | TS04-66-43-BK-100-SMT-TR | S1 | TS04-66-43-BK-100-SMT-TR:SW_TS04-66-43-BK-100-SMT-TR |
| 1 | STSPIN32G4 | U1 | STSPIN32G4:QFN50P900X900X100-65N |
| 1 | STPS1H100AY | U2 | SMA_STM |
| 3 | MAX74810ARMZ-RL | U3, U4, U5 | MAX74810ARMZ:MAX74810ARMZ_ADI |
| 2 | TMUX6136PWR | U6, U7 | TMUX6136:SOP65P640X120-16N |
| 1 | LD1117S50TR | U9 | LD1117S50TR:SOT230P700X180-4N |
| 1 | 93LCxxBxxOT | U10 | Package_TO_SOT_SMD:SOT-23-6 |
