#!/usr/bin/env bash
set -euo pipefail

fail=0

echo "[smoke] Checking SensorManager registry cases..."
for mode in CA CV DPV SWV IMP OCP TEMP IONTOPHORESIS; do
  if ! rg -n "case SensorType::${mode}:" src/sensors/SensorManager.cpp >/dev/null 2>&1; then
    echo "[smoke] Missing SensorType::${mode} in createSensor switch"
    fail=1
  fi
done

echo "[smoke] Checking per-mode control/parser entry points..."
declare -a files=(
  "src/sensors/EChem_CA.cpp"
  "src/sensors/EChem_CV.cpp"
  "src/sensors/EChem_DPV.cpp"
  "src/sensors/EChem_SWV.cpp"
  "src/sensors/EChem_Imp.cpp"
  "src/sensors/EChem_OCP.cpp"
  "src/sensors/EChem_Temp.cpp"
  "src/sensors/Iontophoresis.cpp"
)

for f in "${files[@]}"; do
  if ! rg -n "::loadParameters\\(" "$f" >/dev/null 2>&1; then
    echo "[smoke] Missing loadParameters() in ${f}"
    fail=1
  fi
  if ! rg -n "::start\\(" "$f" >/dev/null 2>&1; then
    echo "[smoke] Missing start() in ${f}"
    fail=1
  fi
  if ! rg -n "::stop\\(" "$f" >/dev/null 2>&1; then
    echo "[smoke] Missing stop() in ${f}"
    fail=1
  fi
done

if [[ "$fail" -ne 0 ]]; then
  echo "[smoke] FAILED"
  exit 1
fi

echo "[smoke] PASSED"
