#!/usr/bin/env bash
set -euo pipefail

fail=0

echo "[sanity] Checking for legacy/mistyped include paths..."
if rg -n "sensors/(EChem_CA|EChem_CV|EChem_DPV|EChem_SWV|EChem_Imp|EChem_OCP|EChem_Temp|Iontophoresis|SensorManager|Sensor)\.h|drivers/AD5940_Helper\.h|drivers/AD5940_hal\.h|tansmitdata_task" src >/dev/null 2>&1; then
  rg -n "sensors/(EChem_CA|EChem_CV|EChem_DPV|EChem_SWV|EChem_Imp|EChem_OCP|EChem_Temp|Iontophoresis|SensorManager|Sensor)\.h|drivers/AD5940_Helper\.h|drivers/AD5940_hal\.h|tansmitdata_task" src
  fail=1
fi

echo "[sanity] Checking for file-scope using namespace in .cpp..."
if rg -n '^\s*using namespace\s+' src --glob '*.cpp' >/dev/null 2>&1; then
  rg -n '^\s*using namespace\s+' src --glob '*.cpp'
  fail=1
fi

echo "[sanity] Checking for CRLF characters in tracked text files..."
while IFS= read -r f; do
  [[ -f "$f" ]] || continue
  if LC_ALL=C grep -q $'\r' "$f"; then
    echo "$f"
    fail=1
  fi
done < <(git ls-files | grep -E '(\.c|\.cpp|\.h|\.hpp|\.md|\.ini|\.txt|\.yml|\.yaml|\.sh)$|(^|/)\.gitignore$|(^|/)\.clang-format$')

if [[ "$fail" -ne 0 ]]; then
  echo "[sanity] FAILED"
  exit 1
fi

echo "[sanity] PASSED"
