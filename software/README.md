# BioCoin Device Software

This repository contains Python code to interface with the BioCoin hardware platform over BLE using the Bluefruit stack.
It supports configuration and execution of multiple electrochemical sensing techniques, including CA, OCP, TEMP, and
Iontophoresis. The software enables data acquisition, processing, and export to CSV format for further analysis.

---

## 📦 Setup

### 1. Clone the repository

```bash
git clone https://github.com/ProfDrewHall/BioCoin.git
cd BioCoin/software
```

### 2. Install the package manager `uv`

```bash
curl -Ls https://astral.sh/uv/install.sh | sh
```

### 3. Install dependencies

```bash
uv install
```

---


## ⚙️ Features

- BLE-based communication with the BioCoin device
- Support for multiple techniques:
  - Chronoamperometry (CA)
  - Open-Circuit Potential (OCP)
  - Temperature Monitoring (TEMP)
  - Iontophoresis (IONTOPH)

---

## 🚀 Example Usage

Run an experiment from `main.py`:

```bash
uv run src/main.py
```

Each technique configuration follows a consistent interface. For example:

```python
ca = ChronoAmperometry(device)
await ca.configure(sampling_interval=1.0, processing_interval=1.0, max_current=100.0, pulse_potential=200.0, channel=1)
data = await ca.run(duration=15)
```

---

## 📂 Directory Layout

```
src/
├── biocoin/
│   ├── device.py
│   └── techniques/
│       ├── ca.py
│       ├── ocp.py
│       ├── temp.py
│       └── iontophoresis.py
│   └── utils/
│       ├── ble_utils.py
├── main.py
└── utils/
    └── logging.py
```

---

## 📄 License

MIT License

---

## 👤 Authors

**Drew A. Hall**
University of California, San Diego
Contact: drewhall@ucsd.edu

**Risab Sankar**
University of California, San Diego
Contact: rsankar@ucsd.edu

**Tyler Hack**
University of California, San Diego
Contact: thack@ucsd.edu
