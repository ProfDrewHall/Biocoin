# BioCoin Firmware

Firmware for the BioCoin wearable electrochemical platform.
This code runs on an nRF52840-based board and controls an Analog Devices AD5940 for multi-modal electrochemical sensing. It exposes a BLE interface for configuring techniques, starting/stopping tests, and streaming results to a host (Python app, mobile, etc.).

Current release: **v1.1.0**

> Techniques supported: **CA**, **CV**, **DPV**, **SWV**, **Impedance (EIS)**, **OCP**, **TEMP**, and **Iontophoresis**.

---

## 🚀 Features

- Modular sensor techniques (`sensors/*`) with a common `Sensor` base class
- AD5940 HAL integration (clock/FIFO/LPDAC/LPAMP/DFT/sequencer, RTIA calibration) - based on ADI code
- Low-power operation with wakeup timer & sleep
- BLE data streaming and status updates
- Typed parameter blocks with bounds checks where applicable
- Lock-free byte streaming of results (float payloads)

---

## 📦 Getting Started

### Prerequisites
- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Note: If you are building on a linux-based platform, you'll need to install the udev rules (https://docs.platformio.org/en/latest/core/installation/udev-rules.html)
- Board: Biocoin board based on nRF52840
- Tooling for DFU (optional): Nordic `nrfutil`

### Build
From VS Code (PlatformIO) or CLI:
```bash
pio run
```

### Flash
USB (default PlatformIO upload target):
```bash
pio run -t upload
```

DFU (example with `nrfutil`, adjust paths/softdevice/bootloader as needed):
```bash
# Example only — adapt to your bootloader/DFU flow
nrfutil dfu usb-serial -pkg firmware_dfu.zip -p <COM_PORT> -b 115200
```

---

## 📁 Repository Structure (high level)

``` 
src/
  main.cpp
  HWConfig/
    constants.h         # Aggregates build checks + config + pins
    build_checks.h      # Compile-time environment validation
    config.h            # Firmware/version/runtime constants
    pins.h              # Board pin assignments
  variants/biocoin/
    variant.h
    variant.cpp
  sensors/
    echem_ca.{h,cpp}
    echem_cv.{h,cpp}
    echem_dpv.{h,cpp}
    echem_swv.{h,cpp}
    echem_imp.{h,cpp}
    echem_ocp.{h,cpp}
    echem_temp.{h,cpp}
    iontophoresis.{h,cpp}
    sensor.{h,cpp}
    sensor_manager.{h,cpp}
    datamover_task.{h,cpp}
  bluetooth/
    bluetooth.{h,cpp}
    gatt.{h,cpp}
    transmitdata_task.h
    transmitdata_task.cpp
  battery/
    battery.{h,cpp}
    battery_task.{h,cpp}
  power/
    power.{h,cpp}
    led_task.{h,cpp}
  storage/
    storage.{h,cpp}
  drivers/
    ad5940_hal.{h,cpp}
    ad5940_helper.{h,cpp}
  util/
    debug_log.h
    task_sync.h
    util.{h,cpp}
```

---

## 🧠 Architecture

All techniques inherit from `sensor::Sensor` and (for streaming) use `SensorQueue<T>` to push typed samples. `SensorManager` owns the active technique and mediates:

- `loadParameters(uint8_t* data, uint16_t len)`
- `start()` / `stop()`
- `ISR()` (invoked via AFE interrupt / data mover task)

**Data streaming:** techniques push `float` samples; the BLE TX pipeline sends a raw byte vector (little-endian floats) produced by `SensorQueue<T>::popBytes()`.

---

## SDK

This project now uses a repo-local PlatformIO variant in `src/variants/biocoin` (configured in `platformio.ini`) so board pin mapping/build settings are version-controlled with firmware source.
No manual edits to PlatformIO package files are required.
For lowest-power operation, SDK-level `WInterrupts` changes are still required (see `SDK Modifications/WInterrupts-port.c` and `SDK Modifications/WInterrupts-port.h`).

---

## Documentation

- `docs/README.md` - docs index and navigation
- `docs/BLE_PROTOCOL.md` - protocol and payload reference
- `docs/BUILD_FLASH.md` - build/flash environment notes
- `docs/POWER.md` - low-power behavior and SDK patch requirements
- `docs/RELEASE_CHECKLIST.md` - release prep checklist
- `test/README.md` - targeted parser/state transition test plan and vectors

---

## 📡 BLE Protocol (summary)

- A **Parameters** characteristic accepts a binary payload:
  - Byte `0`: `sensor::SensorType` (technique selector)
  - Bytes `1..N`: packed parameter struct for that technique (see below)
- A **Control** characteristic accepts a command (`START` / `STOP`)
- A **Status** characteristic reports `TestState` (`NOT_RUNNING`, `RUNNING`, `ERROR`, etc.)
- A **Data** characteristic streams measurement results as raw bytes (floats)
- TX transport detail: outgoing BLE data uses a bounded FreeRTOS stream buffer. If producers outrun BLE notify throughput, excess bytes are dropped and logged.

> UUIDs/handles are defined in the BLE layer (see `src/bluetooth/gatt.*`). Host apps should send the correct packed struct for the selected technique.

---

## 🧾 Parameter Payloads (packed)

Below are the **packed** C layouts (host must match layout, types, and units). Each write to the Parameters characteristic must begin with the technique selector byte (`sensor::SensorType`) followed by the corresponding struct bytes.

### Chronoamperometry (CA)
```c++
struct CA_PARAMETERS {
  float  samplingInterval;    // [s]
  float  processingInterval;  // [s]
  float  maxCurrent;          // [uA] range hint
  float  pulsePotential;      // [mV] WE-RE pulse amplitude
  uint8_t channel;            // 0..3 (board mux)
} __attribute__((packed));
```

### Cyclic Voltammetry (CV)
```c++
struct CV_PARAMETERS {
  float  processingInterval;  // [s]
  float  maxCurrent;          // [uA] range hint
  float  Estart;              // [mV]
  float  Evertex1;            // [mV]
  float  Evertex2;            // [mV]
  float  Estep;               // [mV] step between DAC codes
  float  pulseWidth;          // [ms]
  uint8_t channel;            // 0..3
} __attribute__((packed));
```

### Differential Pulse Voltammetry (DPV)
```c++
struct DPV_PARAMETERS {
  float  processingInterval;  // [s]
  float  maxCurrent;          // [uA] range hint
  float  Estart;              // [mV]
  float  Estop;               // [mV]
  float  Epulse;              // [mV] pulse height
  float  Estep;               // [mV] ramp step
  float  pulseWidth;          // [ms]
  float  pulsePeriod;         // [ms]
  uint8_t channel;            // 0..3
} __attribute__((packed));
```

### Square Wave Voltammetry (SWV)
```c++
struct SWV_PARAMETERS {
  float  processingInterval;  // [s]
  float  maxCurrent;          // [uA] range hint
  float  Estart;              // [mV]
  float  Estop;               // [mV]
  float  Eamplitude;          // [mV] square-wave amplitude
  float  Estep;               // [mV] staircase step
  float  pulsePeriod;         // [ms]
  uint8_t channel;            // 0..3
} __attribute__((packed));
```

### Impedance / EIS (IMP)
```c++
struct IMP_PARAMETERS {
  float  samplingInterval;    // [s]
  float  processingInterval;  // [s]
  uint8_t IMP_4wire;          // 1=4-wire, 0=2-wire
  uint8_t AC_coupled;         // 1=AC, 0=DC
  float  maxCurrent;          // [uA] range hint
  float  Eac;                 // [mV] AC excitation
  float  frequency;           // [Hz]
} __attribute__((packed));
```

### Open Circuit Potential (OCP)
```c++
struct OCP_PARAMETERS {
  float  samplingInterval;    // [s]
  float  processingInterval;  // [s]
  uint8_t channel;            // board/AFE mux (see note)
} __attribute__((packed));
```

### Temperature Monitor (TEMP)
```c++
struct TEMP_PARAMETERS {
  float  samplingInterval;    // [s]
  float  processingInterval;  // [s]
  uint8_t channel;            // ADCMUXP_* selection (board dependent)
} __attribute__((packed));
```

### Iontophoresis
```c++
struct IONTOPHORESIS_PARAMETERS {
  float  samplingInterval;    // [s] monitor interval
  float  stimCurrent;         // [uA] commanded
  float  maxCurrent;          // [uA] safety threshold
} __attribute__((packed));
```

> ⚠️ **Channels:** `channel` maps to board-level MUX/AD5940 inputs (e.g., `ADCMUXP_*`). Keep host-side enums in sync with the firmware build for your hardware variant.

---

## 🧪 Running a Technique (host flow)

1. **Select technique**: Write `SensorType` + packed struct to the **Parameters** characteristic.
2. **Start**: Write `START` to **Control**.
3. **Stream**: Subscribe to **Data** notifications (float samples as little-endian bytes).
4. **Stop**: Write `STOP` to **Control**.
5. **Status**: Read/subscribe to **Status** (`NOT_RUNNING`, `RUNNING`, `ERROR`, etc.).

---

## ⚙️ Implementation Notes

- AD5940 reference voltage should match your measured value (see configs in each technique).
- RTIA selection/auto-cal: Some techniques call `AD5940_CalibrateRTIA` unless external RTIA is forced.
- Power: Firmware disables SPI / enters sleep when possible to reduce current.
- Data types:
  - **Current** streams are in **µA** (computed from ADC code, PGA gain, Vref, and RTIA).
  - **Voltage** streams (e.g., OCP, TEMP) are in **mV**.
- Byte order: little-endian for all streamed floats.

---

## 🧰 Troubleshooting

- **No data**: Ensure `processingInterval >= samplingInterval` (where applicable).
- **Invalid parameters**: Firmware replies via **Status** (`INVALID_PARAMETERS`) if struct size or values are off.
- **Out of range currents**: Pick appropriate `maxCurrent` so the code selects a safe RTIA/gain.
- **MTU**: For larger throughput, request higher ATT MTU on the central (typical 185–247+ depending on OS).

---

## ToDo List
Near term:
- Implement the current range for CA
- Implement advanced parameters
- Add comments / file headers / Readme.MD
- Add static_assert to check config constants
- Cleanup / refactor / reorganize code

Long term:
- Add other techniques: EIS, NPV, LSV, etc.
- Add CRC to parameters for BLE comm?
- Add a serial # to the BLE characteristics

---

## 📜 License

MIT

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

---

### Acknowledgments
Thanks to the BioEE group at UC San Diego and collaborators for contributions and testing.
