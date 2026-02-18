/**
 * @file sensor.h
 * @brief Base sensor interface and typed sensor queue utilities.
 */

#pragma once

#include <cstdint>
#include <queue>
#include <type_traits>
#include <vector>

enum class RunState : uint8_t { STOPPED = 0, RUNNING };

namespace sensor {

  class Sensor {
  public:
    virtual ~Sensor() = default;

    // Standard control interface
    /**
     * @brief Start sensor technique acquisition.
     * @return True when run starts successfully.
     */
    virtual bool start() = 0;
    /**
     * @brief Stop sensor technique acquisition.
     * @return True when run stops successfully.
     */
    virtual bool stop() = 0;
    /**
     * @brief Apply host-provided parameter payload to this sensor technique.
     * @param data Technique-specific packed parameter bytes.
     * @param len Payload length in bytes.
     * @return True when payload is valid and applied.
     */
    virtual bool loadParameters(uint8_t* data, uint16_t len) = 0;

    // Interrupt service routine
    /**
     * @brief Handle technique-specific interrupt processing path.
     */
    virtual void ISR() = 0;

    // Standard data interface
    /**
     * @brief Retrieve serialized measurement data bytes from sensor queue.
     * @param num_items Maximum number of bytes requested.
     * @return Byte vector of serialized samples.
     */
    virtual std::vector<uint8_t> getData(size_t num_items) = 0;
    /**
     * @brief Get number of queued sample bytes currently available.
     * @return Byte count ready for transmission.
     */
    virtual size_t getNumBytesAvailable() const = 0;
    /**
     * @brief Print queued samples to debug output.
     */
    virtual void printResult() = 0;

    // Status
    RunState getRunState() const { return runState; }
    bool isRunning() const { return runState == RunState::RUNNING; }

  protected:
    // Utility functions for ADC scaling and current calculation
    /**
     * @brief Convert ADC code into current in microamps using gain/reference/RTIA settings.
     * @param code Raw ADC code.
     * @param PGAGain ADC PGA gain constant.
     * @param VRef ADC reference voltage in volts.
     * @param rTIA Transimpedance resistance in ohms.
     * @return Current in microamps.
     */
    float calculateCurrent(uint32_t code, uint32_t PGAGain, float VRef, float rTIA);

    // State control
    void setRunning() { runState = RunState::RUNNING; }
    void setStopped() { runState = RunState::STOPPED; }

  private:
    RunState runState{RunState::STOPPED};
  };

  template <typename T>
  class SensorQueue {
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

  public:
    size_t size() const { return queue.size() * sizeof(T); };
    size_t count() const { return queue.size(); };

    template <typename Fn>
    void forEach(Fn func) {
      std::queue<T> copy = queue; // Make a copy since reading is destructive
      while (!copy.empty()) {
        func(copy.front());
        copy.pop();
      }
    }

  protected:
    void push(const T& item) { queue.push(item); }
    bool pop(T& item) {
      if (queue.empty()) return false;
      item = queue.front();
      queue.pop();
      return true;
    }
    std::vector<uint8_t> popBytes(size_t numItems) {
      const size_t elemSize = sizeof(T);
      size_t maxElems = std::min(queue.size(), numItems / elemSize);
      std::vector<uint8_t> byteData;
      byteData.reserve(maxElems * elemSize);

      for (size_t i = 0; i < maxElems; ++i) {
        T value = queue.front();
        queue.pop();
        uint8_t* p = reinterpret_cast<uint8_t*>(&value);
        byteData.insert(byteData.end(), p, p + elemSize);
      }
      return byteData;
    }

    void clear() {
      std::queue<T> empty;
      queue.swap(empty);
    }

  private:
    std::queue<T> queue;
  };

} // namespace sensor
