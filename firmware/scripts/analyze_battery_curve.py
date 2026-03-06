#!/usr/bin/env python3
"""
Analyze battery discharge data and fit V->% models.

Input CSV format:
timestamp_utc,device_address,battery_char_uuid,battery_percent,debug_voltage_mV

Notes:
- This script expects a voltage column (`debug_voltage_mV` or `debug_voltage_mv`).
- Samples with voltage below 3.2V are excluded before fitting.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze battery curve from CSV")
    parser.add_argument("--input", required=True, help="Input CSV path")
    parser.add_argument(
        "--fit",
        choices=["linear", "sigmoid", "both"],
        default="both",
        help="Fit model(s) for V->%",
    )
    parser.add_argument("--out-fit-fig", default="battery_fit_v_percent.png", help="Output figure for V vs % fit")
    parser.add_argument("--out-time-fig", default="battery_voltage_vs_time.png", help="Output figure for voltage vs time")
    parser.add_argument("--out-coeff", default="battery_fit_coefficients.json", help="Output JSON for coefficients")
    return parser.parse_args()


def parse_iso8601(s: str) -> dt.datetime:
    if s.endswith("Z"):
        s = s[:-1] + "+00:00"
    return dt.datetime.fromisoformat(s)


def load_data(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    times: List[dt.datetime] = []
    voltages: List[float] = []

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = parse_iso8601(row["timestamp_utc"])
                mv_raw = row.get("debug_voltage_mV")
                if mv_raw is None or str(mv_raw).strip() == "":
                    mv_raw = row.get("debug_voltage_mv", "")
                if mv_raw is None or str(mv_raw).strip() == "":
                    continue
                mv = float(mv_raw)
            except Exception:
                continue

            times.append(t)
            voltages.append(mv / 1000.0)

    if not voltages:
        raise ValueError(
            "No usable rows found. Need non-empty debug voltage samples in "
            "'debug_voltage_mV' (or legacy 'debug_voltage_mv')."
        )

    t0 = times[0]
    time_minutes = np.array([(t - t0).total_seconds() / 60.0 for t in times], dtype=float)
    voltages_v = np.array(voltages, dtype=float)

    keep = voltages_v >= 3.2
    time_minutes = time_minutes[keep]
    voltages_v = voltages_v[keep]
    if len(voltages_v) == 0:
        raise ValueError("No samples >= 3.2V remain after truncation.")

    # Build fit target from run endpoints: start=100%, end=0%.
    if len(time_minutes) < 2 or float(time_minutes[-1]) <= 0.0:
        percents = np.full_like(time_minutes, 100.0, dtype=float)
    else:
        frac = time_minutes / float(time_minutes[-1])
        percents = np.clip(100.0 * (1.0 - frac), 0.0, 100.0)

    return time_minutes, voltages_v, percents


def linear_model(v: np.ndarray, a: float, b: float) -> np.ndarray:
    return a * v + b


def sigmoid_model(v: np.ndarray, k: float, v0: float) -> np.ndarray:
    return 100.0 / (1.0 + np.exp(-k * (v - v0)))


def rmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


def fit_linear(v: np.ndarray, p: np.ndarray) -> Tuple[Tuple[float, float], np.ndarray, float]:
    coeff = np.polyfit(v, p, 1)
    a, b = float(coeff[0]), float(coeff[1])
    pred = linear_model(v, a, b)
    return (a, b), pred, rmse(p, pred)


def fit_sigmoid(v: np.ndarray, p: np.ndarray) -> Tuple[Tuple[float, float], np.ndarray, float]:
    k0 = 20.0
    v0_0 = float(np.median(v))
    (k, v0), _ = curve_fit(sigmoid_model, v, p, p0=[k0, v0_0], maxfev=10000)
    k, v0 = float(k), float(v0)
    pred = sigmoid_model(v, k, v0)
    return (k, v0), pred, rmse(p, pred)


def save_plots(
    time_minutes: np.ndarray,
    v: np.ndarray,
    p: np.ndarray,
    fit_mode: str,
    out_fit_fig: Path,
    out_time_fig: Path,
    linear_params: Tuple[float, float] | None,
    sigmoid_params: Tuple[float, float] | None,
) -> None:
    v_line = np.linspace(float(np.min(v)), float(np.max(v)), 300)

    plt.figure(figsize=(8, 5))
    plt.scatter(v, p, s=18, alpha=0.8, label="Data")
    if fit_mode in ("linear", "both") and linear_params is not None:
        a, b = linear_params
        plt.plot(v_line, linear_model(v_line, a, b), label=f"Linear: p={a:.3f}*V + {b:.3f}")
    if fit_mode in ("sigmoid", "both") and sigmoid_params is not None:
        k, v0 = sigmoid_params
        plt.plot(v_line, sigmoid_model(v_line, k, v0), label=f"Sigmoid: k={k:.3f}, V0={v0:.3f}")
    plt.xlabel("Voltage (V)")
    plt.ylabel("Percent (%)")
    plt.title("Battery Voltage to Percent Fit")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_fit_fig, dpi=150)

    plt.figure(figsize=(8, 4.5))
    plt.plot(time_minutes, v, marker="o", markersize=3)
    plt.xlabel("Time (minutes)")
    plt.ylabel("Voltage (V)")
    plt.title("Battery Voltage vs Time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_time_fig, dpi=150)


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)
    out_fit_fig = Path(args.out_fit_fig)
    out_time_fig = Path(args.out_time_fig)
    out_coeff = Path(args.out_coeff)

    time_minutes, v, p = load_data(input_path)

    linear_params = None
    sigmoid_params = None
    linear_rmse = None
    sigmoid_rmse = None

    if args.fit in ("linear", "both"):
        linear_params, _, linear_rmse = fit_linear(v, p)
        print(f"Linear fit: a={linear_params[0]:.6f}, b={linear_params[1]:.6f}, rmse={linear_rmse:.4f}")

    if args.fit in ("sigmoid", "both"):
        sigmoid_params, _, sigmoid_rmse = fit_sigmoid(v, p)
        print(f"Sigmoid fit: k={sigmoid_params[0]:.6f}, v0={sigmoid_params[1]:.6f}, rmse={sigmoid_rmse:.4f}")

    save_plots(time_minutes, v, p, args.fit, out_fit_fig, out_time_fig, linear_params, sigmoid_params)

    payload = {
        "fit_mode": args.fit,
        "target_percent_source": "endpoint_time_interpolation",
        "n_samples": int(len(v)),
        "fit_figure": str(out_fit_fig),
        "time_figure": str(out_time_fig),
    }
    if linear_params is not None:
        payload["linear"] = {"a": linear_params[0], "b": linear_params[1], "rmse": linear_rmse}
    if sigmoid_params is not None:
        payload["sigmoid"] = {"k": sigmoid_params[0], "v0": sigmoid_params[1], "rmse": sigmoid_rmse}

    out_coeff.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote coefficients: {out_coeff}")
    print(f"Wrote fit plot: {out_fit_fig}")
    print(f"Wrote time plot: {out_time_fig}")


if __name__ == "__main__":
    main()
