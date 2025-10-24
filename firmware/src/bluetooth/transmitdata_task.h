#pragma once

#include <cstdint>
#include <queue>

namespace bluetooth {

  extern std::queue<uint8_t> TX_queue;

  void createTransmitTask();
  void startTransmitTask(const std::vector<uint8_t>& data);

} // namespace bluetooth
