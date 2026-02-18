#include "drivers/ad5940_hal.h"

#include "HWConfig/constants.h"
#include "util/debug_log.h"
#include "power/power.h"

#include <Arduino.h>
#include <SPI.h>
#include <nrf.h>


static const SPISettings SPISetting(SYS_CLOCK_FREQ / 8, MSBFIRST, SPI_MODE0);

/******************************************************************************
 * @brief Method to deselect the chip select function of the AD5940
 *
 *****************************************************************************/
void AD5940_CsSet(void) {
  digitalWrite(PIN_SPI_CS, HIGH);
}

/******************************************************************************
 * @brief Method to select the chip select function of the AD5940
 *
 *****************************************************************************/
void AD5940_CsClr(void) {
  digitalWrite(PIN_SPI_CS, LOW);
}

/******************************************************************************
 * @brief Method to reset the AD5940
 *
 *****************************************************************************/
void AD5940_RstSet(void) {
  digitalWrite(PIN_AFE_RESET, HIGH);
}

/******************************************************************************
 * @brief Method to clear the reset on the AD5940
 *
 *****************************************************************************/
void AD5940_RstClr(void) {
  digitalWrite(PIN_AFE_RESET, LOW);
}

/******************************************************************************
 * @brief Method to for delaying the AD5940 for a multiple time of 10 micro
 * seconds
 * @details Check if time is larger than 1638.3
 * According to the Arduino specification the largest values for
 * delayMicroseconds() to be accurate is 16383 micro seconds. For larger
 * values delay() in combination with delayMicroseconds() is used.
 *
 * @param time: multiplier which defines the amount of times the AD5940 should
 * be delay by 10 micro seconds
 *
 *****************************************************************************/
void AD5940_Delay10us(uint32_t iTime) {
  // Value is smaller threshold
  if (iTime < 1638) {
    delayMicroseconds(iTime * 10);
  }
  // Value is larger than threshold
  else if (iTime >= 1638) {
    uint32_t iTimeDelayMicro = iTime % 1000;
    uint32_t iTimeDelay = iTime - iTimeDelayMicro;
    delay(iTimeDelay / 100);
    delayMicroseconds(iTimeDelayMicro * 10);
  }
}

/******************************************************************************
 * @brief Method to to write and read data from the SPI port
 * @details: The timing of the SPI must coincide with the frequencies of the
 * MCU and the AD5940.
 * System clock frequency for the AD5940 was chosen as 16 MHz.
 * System clock frequency for the MCU is for the Adafruit Feather M0 48 MHz.
 *
 * For the SPI to work the clock must be equal to the system clock of the MCU
 * devided by a multiply of 2, while staying below the system clock of the
 * AD5940.
 * The highest possible value is therfore 12 MHz.
 *
 * Therefore every 83.3333 ns an SPI clock pulse is send.
 * To send one byte 666.666 ns are required.
 *
 * As send mode the most significant bit is send first: MSBFIRST
 *
 * The SPI operates in mode 0:
 * https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
 *
 * @param pSendBuffer: Send buffer holding all data that should be send via SPI
 * to the AD5940
 * @param pRecvBuff: Receive buffer storing all data transmitted by the AD5940
 * to the MCU via SPI.
 * @param length: Length of transmitted data
 *
 *****************************************************************************/
void AD5940_ReadWriteNBytes(unsigned char* pSendBuffer, unsigned char* pRecvBuff, unsigned long length) {
  SPI.beginTransaction(SPISetting);

  // Repeat sending until length of data is reached
  for (unsigned long i = 0; i < length; i++)
    // Transfer data bidirectional, which means that data is read and written
    // at the same time.
    *pRecvBuff++ = SPI.transfer(*pSendBuffer++);

  SPI.endTransaction();
}

/*
void current_monitor_callback(TimerHandle_t xTimerID)
{
  pinMode(PIN_CURRENT_SENSE_OUT, INPUT);
  Measure_VCCS_Current(PIN_CURRENT_SENSE_OUT);
  power::disconnectInputGPIO(PIN_CURRENT_SENSE_OUT);
  //Disconnect_Input_GPIO(PIN_CURRENT_SENSE_OUT);
  //PrintResult_IONTOPH();
}
*/
void Start_AD5940_SPI() {
  SPI.begin(); // Start SPI connection
  pinMode(PIN_SPI_CS, OUTPUT);
  pinMode(PIN_AFE_RESET, OUTPUT);

  // Set chip select to enable the AD5940 to receive SPI commands when the
  // signal is pulled down to low later
  AD5940_CsSet();

  // Set reset pin for the AD5940 (Beware that the reset pin is negated!)
  AD5940_RstSet();
}

void Stop_AD5940_SPI(void) {
  SPI.end();
  pinMode(PIN_SPI_MISO, INPUT_PULLDOWN); // This is necessary to avoid the pin being floating and causing high power consumption
}

void enableAFEInterrupt(void (*handler)()) {
  // configure MCU to AFE GPIO as an interrupt with falling edge
  //pinMode(PIN_AFE_IntPin_GPIO0, INPUT_PULLUP);
  pinMode(PIN_AFE_IntPin_GPIO0, INPUT_PULLUP_SENSE);                      // Modified for port-based interrupt (lower power)
  attachInterrupt(digitalPinToInterrupt(PIN_AFE_IntPin_GPIO0), handler, FALLING);
}

void disableAFEInterrupt() {
  detachInterrupt(digitalPinToInterrupt(PIN_AFE_IntPin_GPIO0));
}

// Control the external mux to select the correct sensor input
// Truth table for the MUX: 00 = S1 = WE1, 01 = S2 = WE4, 10 = S3 = WE3, 11 = S4 = WE2
int8_t setAFEChannel(uint8_t ch) {
  static const uint8_t sel_bits[] = {0x00, 0x03, 0x02, 0x01};
  if (ch > 3) return -1;

  dbgInfo(String("Setting channel to: ") + ch);
  digitalWrite(PIN_MUX_EN, HIGH); // Ensure that the mux is powered
  digitalWrite(PIN_MUX_A0, sel_bits[ch] & 0x01);
  digitalWrite(PIN_MUX_A1, sel_bits[ch] & 0x02);

  delay(5); // Needed to let the chips startup before we do anything

  return 0;
}
