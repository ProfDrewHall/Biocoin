# BioCoin Firmware

Firmware for the BioCoin wearable electrochemical platform.
This code runs on an nRF52840-based board and controls an Analog Devices AD5940 for multi-modal electrochemical sensing. It exposes a BLE interface for configuring techniques, starting/stopping tests, and streaming results to a host (Python app, mobile, etc.).

> Techniques supported: **CA**, **CV**, **DPV**, **Impedance (EIS)**, **OCP**, **TEMP**, and **Iontophoresis**.

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
sensors/
  EChem_CA.{h,cpp}      # Chronoamperometry
  EChem_CV.{h,cpp}      # Cyclic Voltammetry
  EChem_DPV.{h,cpp}     # Differential Pulse Voltammetry
  EChem_Imp.{h,cpp}     # Impedance / EIS
  EChem_OCP.{h,cpp}     # Open Circuit Potential
  EChem_Temp.{h,cpp}    # Temperature monitor (ADC voltage)
  iontophoresis.h       # Iontophoresis control
  Sensor.h              # Base class + queue

drivers/
  ad5940_hal.*          # AD5940 abstraction and helpers

bluetooth/
  gatt.*                # GATT characteristics, status, data TX
  transmitdata_task.*   # BLE streamer

util/
  debug_log.*           # Logging helpers

power/
  power.*               # AFE and peripheral power control

HWConfig/
  constants.h           # System constants (clocks, etc.)
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

There are some modifications needed to the underlying Arduino framework since this code boostraps the Adafruit feather board based on the nRF52840. Download and install the Adafruit design, then make the modifications in the SDK_Modifications folder to the variants file. These add the additional pins not exposed in the Adafruit design and change the interrupt type (for lower power operation).

---

## 📡 BLE Protocol (summary)

- A **Parameters** characteristic accepts a binary payload:
  - Byte `0`: `sensor::SensorType` (technique selector)
  - Bytes `1..N`: packed parameter struct for that technique (see below)
- A **Control** characteristic accepts a command (`START` / `STOP`)
- A **Status** characteristic reports `TestState` (`NOT_RUNNING`, `RUNNING`, `ERROR`, etc.)
- A **Data** characteristic streams measurement results as raw bytes (floats)

> UUIDs/handles are defined in the BLE layer (see `bluetooth/gatt.*`). Host apps should send the correct packed struct for the selected technique.

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
- Figure out how to do "include shadowing" to avoid having to modify the Adafruit library files directly

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
