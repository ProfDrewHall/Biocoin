/**
 * @file datamover_task.h
 * @brief Data mover task lifecycle API for AFE interrupt processing.
 */

#pragma once

namespace sensor {
  /**
   * @brief Create data-mover task that services AFE interrupts.
   */
  void createDataMoverTask();
  /**
   * @brief Request data-mover task shutdown and wait for task exit.
   */
  void stopDataMoverTask();
  /**
   * @brief Wake/schedule data-mover task to process newly available sensor data.
   */
  void startDataMoverTask();
} // namespace sensor
