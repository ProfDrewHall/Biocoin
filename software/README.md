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

The software requires Python 3.13 or newer. The backend uses Python 3.13 generic class syntax, so Streamlit must also run from a Python 3.13+ environment.

### 1. Clone the repository

```bash
git clone https://github.com/ProfDrewHall/Biocoin.git
cd Biocoin/software
```

### 2. Install Python 3.13

macOS with Homebrew:

```bash
brew install python@3.13
```

Confirm the installed version:

```bash
python3.13 --version
```

### 3. Create and activate a virtual environment

From the `software` directory:

```bash
python3.13 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

After activation, `python --version` should report Python 3.13 or newer.

### 4. Install the package and dependencies

```bash
python -m pip install -e .
```

### 5. Run a technique script

Example:

```bash
python src/run_CA.py --help
```

### 6. Run the Streamlit GUI

```bash
python -m streamlit run src/Home.py
```

Use `python -m streamlit` instead of `streamlit` so the GUI runs with the active Python 3.13 virtual environment.

### Optional: Use `uv`

If you prefer `uv`, install it first:

```bash
brew install uv
```

Then run the GUI with Python 3.13:

```bash
uv run --python 3.13 streamlit run src/Home.py
```

---

## Technique Runner Scripts

Each technique has a dedicated CLI script that exposes its parameters:

```bash
python src/run_CA.py --help
python src/run_CV.py --help
python src/run_DPV.py --help
python src/run_SWV.py --help
python src/run_IMP.py --help
python src/run_OCP.py --help
python src/run_TEMP.py --help
python src/run_IONTO.py --help
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
