# Biocoin Platform

Biocoin is a modular wearable electrochemical sensing platform that integrates custom electronics, embedded firmware, and host-side software for multi-analyte detection in biofluids such as sweat and interstitial fluid.

Developed by the [BioEE Group](https://bioee.ucsd.edu) at the University of California, San Diego, Biocoin combines multimodal biosignal acquisition with wireless communication, microfluidic interfacing, and open analytical tools.

---

## Repository Overview

This repository contains the design assets for the Biocoin platform across hardware, firmware, software, and mechanical fixtures.

| Folder | Description |
|:-------|:------------|
| [`firmware/`](firmware/) | PlatformIO-based firmware for the nRF52840 controller and AD5940 analog front-end, including BLE control and data streaming. |
| [`software/`](software/) | Python host software for BLE communication, technique execution, logging, and result export. |
| [`PCB/`](PCB/) | PCB schematics, layouts, Gerbers, assembly files, and shared component libraries. |
| [`fixtures/`](fixtures/) | Mechanical design assets, fabrication exports, and fixture models for assembly and validation. |

---

## Architecture

The Biocoin system is organized as:

```text
Host software (Python CLI/package in software/)
  -> BLE GATT protocol
  -> nRF52840 firmware (firmware/)
     - SensorManager and technique implementations
     - AD5940 driver and board support
     - BLE, power, battery, and storage modules
  -> AD5940 analog front-end
  -> Electrochemical cell and integrated sensors
```

The current firmware structure is consistent with a FreeRTOS-based design using `SensorManager`, technique-specific sensor modules, and BLE streaming tasks.

---

## Supported Techniques

| Technique | Description | Output |
|:----------|:------------|:-------|
| **CA** | Chronoamperometry | Current vs. time |
| **CV** | Cyclic Voltammetry | Current vs. voltage |
| **DPV** | Differential Pulse Voltammetry | Peak current vs. potential |
| **SWV** | Square-Wave Voltammetry | Differential current vs. potential |
| **IMP / EIS** | Impedance spectroscopy | Magnitude and phase |
| **OCP** | Open-circuit potential | WE potential vs. RE |
| **TEMP** | Temperature monitoring | Voltage-derived temperature data |
| **IONTOPH** | Iontophoretic stimulation | Current control and monitoring |

Each technique can be configured from the host and controlled over BLE. Data-producing techniques stream results back to the host software.

---

## Development Requirements

### Firmware

- [PlatformIO](https://platformio.org/) for build and flash workflows
- A supported nRF52840 board configuration for the Biocoin hardware
- Optional DFU tooling such as Nordic `nrfutil`

### Software

- Python 3.13 or newer
- Recommended: [`uv`](https://docs.astral.sh/uv/) for environment and dependency management
- Python dependencies defined in [`software/pyproject.toml`](software/pyproject.toml):
  - `bleak`
  - `colorama`
  - `numpy`

### Hardware and Mechanical

- Altium Designer or another tool capable of reviewing the PCB source/export files
- CAD software such as SolidWorks or FreeCAD for fixture review and export

---

## Typical Workflow

1. Build and flash the firmware in [`firmware/`](firmware/).
2. Use the Python tooling in [`software/`](software/) to connect over BLE.
3. Run a supported electrochemical technique and collect results.
4. Review or update board and fixture assets in [`PCB/`](PCB/) and [`fixtures/`](fixtures/).

---

## Example Directory Layout

```text
Biocoin/
|-- firmware/   # Embedded firmware, docs, scripts, and tests
|-- software/   # Python package, CLI runners, tests, logs, and results
|-- PCB/        # Board designs, Gerbers, assembly outputs, libraries
`-- fixtures/   # Mechanical assets and fabrication files
```

---

## Licensing

This repository contains both software and hardware design assets. Unless otherwise noted:

- [`firmware/`](firmware/) and [`software/`](software/) are provided under the MIT License.
- [`PCB/`](PCB/) and [`fixtures/`](fixtures/) are provided under the CERN-OHL-W v2.0 license.

See the top-level [`LICENSE`](LICENSE) file for the repository-level license notice.

---

## Authors and Maintainers

**BioEE Group, University of California, San Diego**

Principal Investigator: [Prof. Drew A. Hall](https://bioee.ucsd.edu)

Contributors:

- Risab Sankar
- Tyler Hack
- Members of the BioEE Group (UCSD ECE)
