#!/bin/sh
set -eu

fail=0

echo "[smoke] Checking SensorManager registry cases..."
for mode in CA CV DPV SWV IMP OCP TEMP IONTOPHORESIS; do
  if ! rg -n "case SensorType::${mode}:" src/sensors/core/sensor_manager.cpp >/dev/null 2>&1; then
    echo "[smoke] Missing SensorType::${mode} in createSensor switch"
    fail=1
  fi
done

echo "[smoke] Checking per-mode control/parser entry points..."
for f in \
  "src/sensors/techniques/ca/echem_ca.cpp" \
  "src/sensors/techniques/cv/echem_cv.cpp" \
  "src/sensors/techniques/dpv/echem_dpv.cpp" \
  "src/sensors/techniques/swv/echem_swv.cpp" \
  "src/sensors/techniques/imp/echem_imp.cpp" \
  "src/sensors/techniques/ocp/echem_ocp.cpp" \
  "src/sensors/techniques/temp/echem_temp.cpp" \
  "src/sensors/techniques/iontophoresis/iontophoresis.cpp"
do
  if [ ! -f "$f" ]; then
    echo "[smoke] Missing file: ${f}"
    fail=1
    continue
  fi

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

if [ "$fail" -ne 0 ]; then
  echo "[smoke] FAILED"
  exit 1
fi

echo "[smoke] PASSED"
