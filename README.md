# ESC_HS

This repository contains two generations of the ESC project. The repository name
remains `ESC_HS`. The original project names `ESC_HS` and `ESC_HF` are unchanged.

| Version | Project | Contents | Documentation |
| --- | --- | --- | --- |
| v1 | `ESC_HS` | STSPIN32G4/STM32G431 firmware, matching KiCad design, capture tools and BOM. | [v1 usage](v1/ESC_HS/README.md) |
| v2 | `ESC_HF` | STM32H750 + DRV8323 KiCad hardware design and component libraries. | [v2 hardware](v2/ESC_HF/README.md) |

```text
ESC_HS/                         existing Git repository
  .gitignore                    shared rules for both versions
  README.md                     version index
  v1/
    ESC_HS/                     original CubeIDE project name
      Core/
      Drivers/
      esc_design/
      hardware/                 v1 BOM and review findings
      tools/
      ESC_HS.ioc
      README.md
  v2/
    ESC_HF/                     original hardware project name
      ESC_HF/                   KiCad project, schematic, PCB and Gerbers
      componente/               component symbols and footprints
      README.md
```

## Using v1

Import `v1/ESC_HS/` as the existing `ESC_HS` project in STM32CubeIDE. If an IDE
workspace still points to the old repository root, remove that workspace entry
without deleting files, then import the new directory. The project name remains
`ESC_HS`, so its workspace-relative build configuration can keep using that name.

Run the commands in the v1 usage guide from `v1/ESC_HS/`. Its schematic and BOM
links remain relative to that project directory:

- [BOM and design findings](v1/ESC_HS/hardware/BOM.md)
- [Excel BOM](v1/ESC_HS/hardware/outputs/esc_hs_bom/BOM.xlsx)
- [Grouped BOM CSV](v1/ESC_HS/hardware/BOM.csv)

The preserved v1 firmware can start with a 2000 RPM target after power-up. Review
the commissioning instructions and configuration before powering the bridge.

## Using v2

Open `v2/ESC_HF/ESC_HF/ESC_HF.kicad_pro` in KiCad. The nested hardware project and
the component-library directory retain their original names and contents.
This version currently contains hardware files. The v1 BOM and firmware describe
a different controller and power-stage design; v2 needs its own component export
and firmware implementation.

## Git organization

Both versions share this repository's existing history. Run Git commands from
this directory. Generated build files, KiCad backups/caches, source snapshots and
captures stay local under their respective version folders and are ignored.

The reorganization preserves project sources, original design files and local
build outputs. It does not change firmware behavior or hardware components.
