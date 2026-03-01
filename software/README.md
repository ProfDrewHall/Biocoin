# Biocoin Device Software

This repository contains Python code to interface with the Biocoin hardware platform over BLE.
It supports configuration and execution of all currently implemented techniques:

- Chronoamperometry (CA)
- Cyclic Voltammetry (CV)
- Differential Pulse Voltammetry (DPV)
- Square-Wave Voltammetry (SWV)
- Impedance Spectroscopy (IMP)
- Open-Circuit Potential (OCP)
- Temperature Monitoring (TEMP)
- Iontophoresis (IONTOPH)

The software is organized as a reusable package (`biocoin`) plus standalone technique runner scripts under `src/`.

---

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/ProfDrewHall/Biocoin.git
cd Biocoin/software
```

### 2. Install `uv`

macOS / Linux:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Windows (PowerShell):

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 3. Run a technique script

Example:

```bash
uv run python src/run_CA.py --help
```

`uv run` will create/sync the environment automatically as needed.

---

## Technique Runner Scripts

Each technique has a dedicated CLI script that exposes its parameters:

```bash
uv run python src/run_CA.py --help
uv run python src/run_CV.py --help
uv run python src/run_DPV.py --help
uv run python src/run_SWV.py --help
uv run python src/run_IMP.py --help
uv run python src/run_OCP.py --help
uv run python src/run_TEMP.py --help
uv run python src/run_IONTO.py --help
```

Typical output CSV paths default to `./results/*_output.csv` for data-producing techniques.

---

## Technique Lifecycle Contract

General flow:

1. `await technique.configure(...)`
2. `data = await technique.run(...)`
3. Consume `data` as a NumPy array

Returned shape expectations:

- Timed techniques (`CA`, `OCP`, `TEMP`, `IMP`) can return a variable number of samples based on BLE/runtime timing.
- Sweep techniques (`CV`, `DPV`, `SWV`) are expected to return deterministic point counts from their configured vectors.
- `Iontophoresis` does not stream samples and returns an empty `(0, 2)` array.

---

## Directory Layout

```text
src/
|-- run_CA.py
|-- run_CV.py
|-- run_DPV.py
|-- run_SWV.py
|-- run_IMP.py
|-- run_OCP.py
|-- run_TEMP.py
|-- run_IONTO.py
|-- biocoin/
|   |-- __init__.py
|   |-- device.py
|   |-- errors.py
|   |-- techniques/
|   |   |-- __init__.py
|   |   |-- base_technique.py
|   |   |-- ca.py
|   |   |-- cv.py
|   |   |-- dpv.py
|   |   |-- swv.py
|   |   |-- impedance.py
|   |   |-- ocp.py
|   |   |-- temp.py
|   |   |-- iontophoresis.py
|   |   |-- pulse_voltammetry.py
|   |   `-- validation.py
|   `-- utils/
|       `-- ble_util.py
`-- utils/
    `-- logging_util.py
```

---

## License

MIT License

---

## Authors

**Drew A. Hall**
University of California, San Diego
Contact: drewhall@ucsd.edu

**Risab Sankar**
University of California, San Diego
Contact: rsankar@ucsd.edu

**Tyler Hack**
University of California, San Diego
Contact: thack@ucsd.edu
