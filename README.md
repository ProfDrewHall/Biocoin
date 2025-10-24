# BioCoin Platform

The **BioCoin** project is a modular wearable electrochemical sensing platform that integrates **custom electronics**, **embedded firmware**, and **host-side software** for multi-analyte detection in biofluids such as sweat and interstitial fluid.

Developed by the [BioEE Group](https://bioee.ucsd.edu) at the **University of California, San Diego**, BioCoin combines multimodal biosignal acquisition with wireless communication, microfluidic interfacing, and open-source analytical tools.

---

## 🧩 Repository Overview

This repository contains all design assets for the BioCoin platform — spanning **hardware**, **firmware**, **software**, and **fixtures** used for assembly and testing.

| Folder | Description |
|:--------|:-------------|
| [`firmware/`](firmware/) | Source code for the nRF52840-based controller managing the AD5940 analog front-end. Implements multiple electrochemical techniques (CA, CV, DPV, EIS, OCP, TEMP, Iontophoresis). |
| [`software/`](software/) | Host-side applications (Python, desktop, or mobile) for BLE communication, data acquisition, visualization, and parameter control. |
| [`PCB/`](PCB/) | Complete PCB design files (schematics, layouts, Gerbers, and BOMs) for the BioCoin board and electrode interface modules. |
| [`fixtures/`](fixtures/) | 3D models, assembly jigs, and test fixtures used for validation, calibration, and mechanical integration. |

---

## ⚙️ Architecture

The BioCoin ecosystem follows a modular hierarchy:

```
┌─────────────────────────────┐
│         Host Device         │
│ (Python App / Mobile App)   │
│   └── BLE interface          │
└──────────────┬──────────────┘
               │
        BLE (GATT Protocol)
               │
┌──────────────┴──────────────┐
│     nRF52840 Controller     │
│ (runs firmware/ on FreeRTOS)│
│  ├─ SensorManager            │
│  ├─ AD5940 AFE driver        │
│  └─ Power/BLE modules        │
└──────────────┬──────────────┘
               │
        SPI / Analog Front-End
               │
┌──────────────┴──────────────┐
│        AD5940 AFE           │
│  ├─ Electrochemical cell    │
│  └─ Integrated sensors      │
└─────────────────────────────┘
```

---

## 🧠 Supported Techniques

| Technique | Description | Output |
|:-----------|:-------------|:--------|
| **CA** | Chronoamperometry | Current vs. time |
| **CV** | Cyclic Voltammetry | Current vs. voltage |
| **DPV** | Differential Pulse Voltammetry | Peak current vs. potential |
| **EIS / Impedance** | Frequency sweep impedance | Magnitude/phase |
| **OCP** | Open-Circuit Potential | WE potential vs. RE |
| **TEMP** | Temperature monitor | ADC voltage or °C equivalent |
| **IONTOPH** | Iontophoretic stimulation | Current control and monitoring |

Each technique can be configured, started, and streamed over BLE.

---

## 🧰 Development Setup

### Prerequisites
- **PlatformIO** (VS Code plugin or CLI) for firmware builds
- **Altium Designer** for PCB review
- **Python 3.10+** with `bleak`, `pandas`, and `matplotlib` for host applications
- **3D CAD software** (Fusion 360, SolidWorks, or FreeCAD) for mechanical fixtures

### Typical Workflow

1. Build and flash the firmware (`firmware/`) to the BioCoin device.
2. Connect via BLE using the host `software/` tools.
3. Acquire data and visualize current, impedance, or potential.
4. Modify PCB or mechanical designs as needed (`PCB/`, `fixtures/`).

---

## 📦 Example Directory Layout

```
BioCoin/
│
├── firmware/          # nRF52840 + AD5940 control
├── software/          # Host app and analysis tools
├── PCB/               # Hardware design (schematic, layout)
└── fixtures/          # Mechanical and test fixtures
```

---

## 🧾 Licensing

Unless otherwise noted:

- **Firmware** and **software** are released under the [MIT License](https://opensource.org/licenses/MIT).
- **PCB** and **fixtures** may be released under the [CERN-OHL-W v2.0](https://ohwr.org/project/cernohl) hardware license.
- Please check each subdirectory for its own `LICENSE` file.

---

## 👤 Authors & Maintainers

**BioEE Group — University of California, San Diego**
Principal Investigator: [Prof. Drew A. Hall](https://bioee.ucsd.edu)

Contributors:
- Risab Sankar
- Tyler Hack
- Members of the BioEE Group (UCSD ECE)
