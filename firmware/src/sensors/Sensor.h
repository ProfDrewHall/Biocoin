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
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool loadParameters(uint8_t* data, uint16_t len) = 0;

    // Interrupt service routine
    virtual void ISR() = 0;

    // Standard data interface
    virtual std::vector<uint8_t> getData(size_t num_items) = 0;
    virtual size_t getNumBytesAvailable() const = 0;
    virtual void printResult() = 0;

    // Status
    RunState getRunState() const { return runState; }
    bool isRunning() const { return runState == RunState::RUNNING; }

  protected:
    // Utility functions for ADC scaling and current calculation
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
