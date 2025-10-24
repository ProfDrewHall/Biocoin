#pragma once

#include <Arduino.h>

namespace storage {

  void init();
  void writeDeviceName(const String& name);
  String readDeviceName();

} // namespace storage
