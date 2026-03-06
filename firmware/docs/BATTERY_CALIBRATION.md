# Battery Calibration Workflow

This workflow captures battery discharge voltage data and fits a voltage-to-percent curve for firmware constants in `src/HWConfig/config.h`.

## Prerequisites

- Flash **debug** firmware (the raw battery-voltage BLE characteristic is debug-only).
- Python 3.10+.
- Script deps from `scripts/pyproject.toml`:
  - `bleak`
  - `numpy`
  - `scipy`
  - `matplotlib`

Example setup (from `firmware/`):

```bash
cd scripts
uv sync
```

or:

```bash
cd scripts
pip install bleak numpy scipy matplotlib
```

## 1) Log Battery Data Over BLE

Run the logger while the device discharges:

```bash
python log_battery_ble.py \
  --address <BLE_MAC_OR_ID> \
  --debug-voltage \
  --debug-voltage-char-uuid 00001526-1212-EFDE-1523-785FEABC93AA \
  --interval-sec 120 \
  --output battery_log.csv
```

Notes:

- `--debug-voltage-char-uuid` points to the debug-only raw battery voltage characteristic.
- `battery_log.csv` is generated locally and should not be committed.

## 2) Fit Curve and Generate Coefficients

```bash
python analyze_battery_curve.py \
  --input battery_log.csv \
  --fit both \
  --out-coeff battery_fit_coefficients.json \
  --out-fit-fig battery_fit_v_percent.png \
  --out-time-fig battery_voltage_vs_time.png
```

Generated artifacts:

- `battery_fit_coefficients.json`
- `battery_fit_v_percent.png`
- `battery_voltage_vs_time.png`

The PNG files are analysis artifacts and should not be committed.

## Artifact Format

`battery_fit_coefficients.json` contains:

- `fit_mode`
- `target_percent_source`
- `n_samples`
- `linear`: `{ a, b, rmse }` (if requested)
- `sigmoid`: `{ k, v0, rmse }` (if requested)

Firmware uses the sigmoid coefficients:

- `kBatteryFitSlope`  <= `sigmoid.k`
- `kBatteryFitMidVoltage` <= `sigmoid.v0`

## 3) Update Firmware Constants

Copy sigmoid values into `src/HWConfig/config.h`:

- `battery::kBatteryFitSlope`
- `battery::kBatteryFitMidVoltage`

Then rebuild and validate battery percentage behavior over a known discharge curve.
