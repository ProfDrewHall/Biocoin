#include "storage/storage.h"

#include "HWConfig/config.h"
#include "util/debug_log.h"

#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>

namespace storage {
  constexpr size_t kMaxDeviceNameLen = BLE_GAP_DEVNAME_MAX_LEN + 1; // Include null terminator.
  constexpr const char* kDeviceFilename = "/devicename.txt";
} // namespace storage

static Adafruit_LittleFS_Namespace::File file(InternalFS);

void storage::init() {
  dbgInfo("Initializing filesystem...");
  InternalFS.begin();
}

void storage::writeDeviceName(const String& name) {
  String sanitizedName = name;
  if (sanitizedName.length() > BLE_GAP_DEVNAME_MAX_LEN - 1) {
    sanitizedName = sanitizedName.substring(0, BLE_GAP_DEVNAME_MAX_LEN - 1);
  }

  dbgInfo("Writing new device name: " + sanitizedName);

  InternalFS.remove(kDeviceFilename); // Remove the old file

  if (file.open(kDeviceFilename, Adafruit_LittleFS_Namespace::FILE_O_WRITE)) {
    file.write(sanitizedName.c_str(), sanitizedName.length() + 1); // write + null terminator
    file.close();
    file.flush(); // Make sure it gets written before we restart

    dbgInfo("Updated device name: " + readDeviceName());
    dbgInfo("Restarting to apply changes...");
    NVIC_SystemReset();
  } else {
    dbgError("Failed to write name — file open error");
  }
}

String storage::readDeviceName() {
  char buffer[kMaxDeviceNameLen] = {0};

  if (file.open(kDeviceFilename, Adafruit_LittleFS_Namespace::FILE_O_READ) && file.size() != 0) {
    size_t len = file.read(buffer, sizeof(buffer));
    file.close();

    if (len > 0) return String(buffer);
  }

  return kDefaultName;
}
