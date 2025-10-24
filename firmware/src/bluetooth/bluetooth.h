#pragma once

#include <bluefruit.h>

namespace bluetooth {

  extern BLEService bleService;
  extern BLEBas blebas;
  extern uint16_t dataSize;
  constexpr uint16_t kATTHeaderLen = 3;                 // 3 bytes are used by the ATT protocol header

  // Function declarations
  void init();
  void initStandardServices();

  // Event notifications
  void onConnect(uint16_t conn_handle);
  void onDisconnect(uint16_t conn_handle, uint8_t reason);

} // namespace bluetooth
