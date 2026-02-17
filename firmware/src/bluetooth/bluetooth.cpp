#include "bluetooth.h"

#include <bluefruit.h>
#include "HWConfig/constants.h"
#include "battery/battery.h"
#include "bluetooth/gatt.h"
#include "bluetooth/transmitdata_task.h"
#include "util/debug_log.h"
#include "sensors/EChem_CA.h"
#include "storage/storage.h"

#include <queue>

constexpr size_t kSlaveLatency =
    (bluetooth::kConnectionIntervalEff / bluetooth::kConnectionInterval - 1); // Number of connection intervals to skip
constexpr uint16_t kDefaultMTU = 23;

namespace bluetooth {
  BLEService bleService(kUUIDService);
  BLEDis bledis; // Device Info over BLEkUUIDService
  BLEBas blebas; // Battery % over BLE
  BLEDfu bledfu; // Over the air updates

  uint16_t dataSize = kDefaultMTU - kATTHeaderLen;      // Required to be 3 bytes less 

} // namespace bluetooth


// Check the configuration
static_assert(bluetooth::kAdvSlowRate > 0.625f, "BLE Slow advertising rate must be >0.625");
static_assert(bluetooth::kAdvFastRate > 0.625f, "BLE Fast advertising rate must be >0.625");
static_assert(kSlaveLatency > 0, "Slave latency must be greater than 0");
static_assert((sizeof(bluetooth::kManufacturer) - 1) <= 20, "Manufacturer string exceeds BLE DIS limit");
static_assert((sizeof(bluetooth::kModel) - 1) <= 20, "Model string exceeds BLE DIS limit");


void bluetooth::init() {
  dbgInfo("Initializing Bluetooth...");

  String device_name = storage::readDeviceName();
  dbgInfo(String("Advertising as: ") + device_name);

  // Bluefruit BLE initialization
  Bluefruit.autoConnLed(false);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(kTxPower);
  Bluefruit.setName(device_name.c_str());
  Bluefruit.ScanResponse.addName();  // There is no room for Name in Advertising packet. Use Scan response for Name

  //Bluefruit.Periph.setConnInterval(static_cast<uint16_t>(kConnectionInterval/1.25),
  //                                 static_cast<uint16_t>(kConnectionInterval/1.25)); //in units of 1.25ms. Min/Max range is from 7.5ms to 4000ms
  //Bluefruit.Periph.setConnSlaveLatency(kSlaveLatency); //Number of connection events that can be skipped if there is no new data to send
  //Bluefruit.Periph.setConnSupervisionTimeout(static_cast<uint16_t>(kSupervisorTimeout/10)); // Max time between "effective connection intervals" before the connection is terminated. 32 seconds is the max.
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  initStandardServices(); // Initialize standard GATT services (e.g., DIS, BAS, DFU)
  initGatt();             // Set up all custom services and characteristics

  // Advertise BioCoin service
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleService);
  Bluefruit.Advertising.restartOnDisconnect(true); // Enable auto advertising if disconnected
  Bluefruit.Advertising.setInterval(static_cast<uint16_t>(kAdvFastRate/0.625f), 
                                    static_cast<uint16_t>(kAdvSlowRate/0.625f));      // Advertising are in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(kAdvFastTimeout);        // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                  // 0 = Don't stop advertising after n seconds

  // Create the task to handle dumping the BLE data
  createTransmitTask();
}

void bluetooth::initStandardServices() {
  dbgInfo("Initializing standard BLE services...");

  // DFU (Device Firmware Update) Service -- Note: THIS NEEDS TO BE FIRST
  bledfu.begin();

  // Device Information Service
  bledis.setManufacturer(kManufacturer);
  bledis.setModel(kModel);
  bledis.setFirmwareRev(kFirmwareRev);
  bledis.begin();

  // Battery Service
  blebas.begin();
  blebas.write(0); // Start with dummy level; updated by the battery module
}

void bluetooth::onConnect(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  char central_name[BLE_GAP_DEVNAME_MAX_LEN + 1] = {0};
  conn->getPeerName(central_name, sizeof(central_name));
  dbgInfo(String("Connected to ") + central_name);

  // Get the maximum trasnmit unit (MTU) and try to negotiate a larger one
  uint16_t MTU = conn->getMtu();
  conn->requestPHY(BLE_GAP_PHY_AUTO); //*BLE_GAP_PHY_2MBPS
  conn->requestDataLengthUpdate();

  if(MTU < kMTURequest)
    conn->requestMtuExchange(kMTURequest);

  // Update the BLE connection interval
  conn->requestConnectionParameter(static_cast<uint16_t>(kConnectionInterval/1.25), 
                                   kSlaveLatency, 
                                   static_cast<uint16_t>(kSupervisorTimeout/10)); // conn interval in units of 1.25ms, slave latency in integer units, supervisor timeout in units of 10ms
  
  // Note, we need to wait for the parameters to be updated. They are requested and handled by the BlueFruit library event hander. There needs to be a delay here                                   
  delay(1250);

  // Show the connection parameters
  uint16_t newMTU = conn->getMtu();
  dbgInfo(String("  MTU: ") + newMTU + String(" bytes"));
  dataSize = newMTU - kATTHeaderLen;      // Required to be 3 bytes less 

  uint8_t phy = conn->getPHY();
  dbgInfo(String("  PHY: ") + 
          (phy == BLE_GAP_PHY_1MBPS ? "1 Mbps" :
           phy == BLE_GAP_PHY_2MBPS ? "2 Mbps" :
           phy == BLE_GAP_PHY_CODED ? "Coded" : "Unknown"));

  // Only TX octets are available through Bluefruit API
  uint16_t tx_octets = conn->getDataLength();
  dbgInfo(String("  Max RX/TX Octets: ") + tx_octets);
  float interval_ms = conn->getConnectionInterval() * 1.25f;
  dbgInfo(String("  Conn Interval: ") + String(interval_ms, 2) + " ms");
  dbgInfo(String("  Slave Latency: ") + conn->getSlaveLatency());
  float timeout_ms = conn->getSupervisionTimeout() * 10.0f;
  dbgInfo(String("  Supervision Timeout: ") + String(timeout_ms, 2) + " ms");
                              
  battery::start();                                        // Start the battery monitoring

  // Clear the queue or let it do it on its own?
}

void bluetooth::onDisconnect(uint16_t conn_handle, uint8_t reason) {
  dbgInfo(String("Disconnected, reason = ") + reason);


  // CLear the queue or let it do it on its own?

  battery::stop();
}
