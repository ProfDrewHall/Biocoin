# PCB Design Files

This directory contains the PCB design deliverables for the BioCoin hardware, including source design files, fabrication outputs, assembly files, and shared Altium libraries.

## Folder Overview

### `Biocoin/`

PCB files for the BioCoin board.

- `design/`: Altium project files, PCB layout, schematics, harness files, and output job definitions
- `gerber/`: fabrication outputs for board manufacturing
- `assembly/`: bill of materials and pick-and-place files for assembly
- `Schematic.PDF`: exported schematic reference

Notable files:

- `design/Biocoin_v1.3.PrjPcb`
- `design/Biocoin_v1.3.PcbDoc`
- `assembly/BOM_Final_Biocoin_v1.3_AVDD=3.0V.xlsx`
- `assembly/BOM_Final_Biocoin_v1.3_AVDD=3.6V.xlsx`
- `assembly/Pick_and_Place_Assembly_Biocoin_v1.3.txt`

### `Docking Station/`

PCB files for the BioCoin docking station / motherboard.

- `design/`: Altium project files, PCB layout, schematics, and output job definitions
- `gerber/`: fabrication outputs for board manufacturing
- `assembly/`: bill of materials and pick-and-place files for assembly
- `Schematic.PDF`: exported schematic reference

Notable files:

- `design/Biocoin_Motherboard_v1.0.PrjPcb`
- `design/Biocoin_Motherboard.PcbDoc`
- `assembly/BOM_Final_DockingStation_v1.0.xlsx`
- `assembly/Pick_and_Place_Assembly_DockingStation_v1.0.txt`

### `Shared Libraries/`

Reusable Altium schematic and PCB libraries shared across the designs in this folder.

- `Modules.SchLib`
- `Modules.PcbLib`
- `Schlib.SchLib`
- `PcbLib.PcbLib`

## Manufacturing And Assembly Notes

For each board:

- Use the files in `gerber/` for PCB fabrication
- Use the files in `assembly/` for population and placement
- Use the `design/` folder if you need to open or modify the board in Altium Designer
- Use `Schematic.PDF` for quick review without opening the design tools

## Click-To-Order Links

### Biocoin

- [Order 10 sets](https://pcbminions.com/product/product_1773404171-10sets)
- [Order 100 sets](https://pcbminions.com/product/product_1773404171-100sets)

### Docking Station

- [Order 10 sets](https://pcbminions.com/product/product_1773677769-10sets)
- [Order 100 sets](https://pcbminions.com/product/product_1773677769-100sets)

## Tooling

These files are in Altium Designer format (`.PrjPcb`, `.PcbDoc`, `.SchDoc`, `.OutJob`, `.SchLib`, `.PcbLib`).
