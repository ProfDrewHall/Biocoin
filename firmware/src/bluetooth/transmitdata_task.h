#pragma once

#include <cstdint>
#include <vector>

namespace bluetooth {

  void createTransmitTask();
  void stopTransmitTask();
  void startTransmitTask(const std::vector<uint8_t>& data);
  void clearTransmitBuffer();

} // namespace bluetooth
