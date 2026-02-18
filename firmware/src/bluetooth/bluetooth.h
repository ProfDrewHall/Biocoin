/**
 * @file bluetooth.h
 * @brief Bluetooth stack initialization and connection lifecycle API.
 */

#pragma once

#include <bluefruit.h>

namespace bluetooth {

  extern BLEService bleService;
  extern BLEBas blebas;
  extern uint16_t dataSize;
  constexpr uint16_t kATTHeaderLen = 3;                 // 3 bytes are used by the ATT protocol header

  // Function declarations
  /**
   * @brief Initialize BLE stack, services, advertising, and TX task.
   */
  void init();
  /**
   * @brief Initialize standard BLE services (DFU, DIS, BAS).
   */
  void initStandardServices();

  // Event notifications
  /**
   * @brief Connection callback to negotiate link settings and start runtime services.
   * @param conn_handle Connected peer handle.
   */
  void onConnect(uint16_t conn_handle);
  /**
   * @brief Disconnect callback to stop runtime services tied to active link.
   * @param conn_handle Disconnected peer handle.
   * @param reason BLE stack disconnect reason code.
   */
  void onDisconnect(uint16_t conn_handle, uint8_t reason);

} // namespace bluetooth
