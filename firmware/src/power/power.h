/**
 * @file power.h
 * @brief System power rail and peripheral power-control API.
 */

#pragma once

#include <cstdint>

namespace power {

  enum class PullConfig { Disabled, Pullup, Pulldown };

  /**
   * @brief Initialize power subsystem, pin modes, and default low-power state.
   */
  void init();
  /**
   * @brief Configure board GPIO directions for all power-controlled peripherals.
   */
  void initPins();
  /**
   * @brief Enable onboard DC-DC regulator mode when supported.
   */
  void enableDCDC();
  /**
   * @brief Put GPIO into disconnected-input state to reduce leakage current.
   * @param pin Arduino pin number.
   */
  void disconnectInputGPIO(uint32_t pin);
  /**
   * @brief Re-enable GPIO input buffer with selected pull configuration.
   * @param pin Arduino pin number.
   * @param pull Pull configuration to apply.
   */
  void reconnectInputGPIO(uint32_t pin, PullConfig pull);

  /**
   * @brief Enable power rails and mux path for AD5940 front-end operation.
   * @param muxChannel Mux channel index routed to AFE input.
   */
  void powerOnAFE(uint8_t muxChannel = 0);
  /**
   * @brief Enable rails required for temperature-sensor measurement mode.
   */
  void powerOnTempSensor();
  /**
   * @brief Enable rails/switching required for iontophoresis stimulation mode.
   */
  void powerOnIontophoresis();
  /**
   * @brief Disable powered peripherals and return pins to low-power idle state.
   */
  void powerOffPeripherials();
} // namespace power
