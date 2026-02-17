// Module: Top-level hardware constants include.
// Purpose: Expose config, pins, and invariant checks through one shared header.

#pragma once

#include <Arduino.h>
#include <cstdint>

#include "HWConfig/build_checks.h"
#include "HWConfig/config.h"
#include "HWConfig/pins.h"

// AD5940 conversion constants used across electrochemistry techniques.

#define SYS_CLOCK_FREQ 64000000.0f // Defined system frequency of the MCU

#define AD5940_MAX_DAC_OUTPUT 2400.0
#define AD5940_MIN_DAC_OUTPUT 200.0
#define AD5940_6BIT_DAC_1LSB ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 64)
#define AD5940_12BIT_DAC_1LSB ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 4095)
