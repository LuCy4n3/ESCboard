# ESC_HF

**ESC v2.** The project name remains `ESC_HF`. This hardware generation uses
STM32H750 and DRV8323, with a separate schematic and PCB from
[ESC_HS v1](../../v1/ESC_HS/README.md).

## Contents

| Path | Contents |
| --- | --- |
| `ESC_HF/ESC_HF.kicad_pro` | KiCad project. |
| `ESC_HF/ESC_HF.kicad_sch` | Schematic. |
| `ESC_HF/ESC_HF.kicad_pcb` | PCB layout. |
| `componente/` | Component-library resources supplied with the design. |
| `ESC_HF/grb/`, `ESC_HF/grb.zip` | Existing manufacturing exports. |

Open `ESC_HF/ESC_HF.kicad_pro` in KiCad. Review the selected library paths when
opening the project on a different machine. Bundled 3D model links use
`${KIPRJMOD}/../componente/` so they resolve from this KiCad project after the move.
Manufacturing exports are retained
as supplied; regenerate them from the intended PCB revision before fabrication.

This directory currently contains hardware design files. It does not contain a
dedicated v2 firmware project or a newly generated v2 BOM. The v1 firmware targets
STSPIN32G4/STM32G431 and must be ported for the v2 controller and peripheral mapping.

The original schematic, circuit/layout, component resources and local backups
were preserved during version organization. Only the old absolute 3D model links
in the PCB and four footprint files were changed to project-relative paths.
Backups, lock files and caches are ignored by the
repository's shared `.gitignore`.

See the [repository version index](../../README.md) for the full layout and v1 BOM links.
