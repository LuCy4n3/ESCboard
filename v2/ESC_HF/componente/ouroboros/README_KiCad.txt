OUROBOROS / SNAKE LOGO FOR KICAD
================================

1. Easiest: import the SVG directly
   Open KiCad PCB Editor -> File -> Import -> Graphics...
   Select Ouroboros_KiCad.svg, choose F.SilkS (front silkscreen),
   set scale/dimensions as needed and place on the PCB.

2. Reusable footprint
   PCB Editor -> Preferences -> Manage Footprint Libraries...
   -> Project Specific Libraries -> Add Existing Library
   -> choose the folder Ouroboros.pretty.
   In PCB Editor, use Place -> Add Footprint -> search for Ouroboros_50mm.
   The footprint is about 50 mm wide and is on F.SilkS.

3. The transparent PNG can also be imported via KiCad Image Converter
   (bitmap2component) as a PCB footprint; use F.SilkS output.

4. Notes:
   Complex image with very fine details. A fab may not print lines narrower
   than approximately 0.12-0.15 mm or small gaps at normal board sizes.
   Zoom in on the silkscreen in KiCad and check manufacturer min specs.
   This is a trace of the supplied source and may retain source artifacts.
