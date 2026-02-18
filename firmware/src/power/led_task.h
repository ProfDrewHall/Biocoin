/**
 * @file led_task.h
 * @brief LED task lifecycle API for user/status indication.
 */

#pragma once

namespace power {
  /**
   * @brief Start low-priority heartbeat LED task used for runtime liveness indication.
   */
  void startHeartbeatTask();
} // namespace power
