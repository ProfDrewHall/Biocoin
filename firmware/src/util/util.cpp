/**
 * @file util.cpp
 * @brief General firmware utility function implementations.
 */

#include "util/debug_log.h"

#include <Arduino.h>
#include <nrf.h>


void dbgPrintResetReason() {

  uint32_t reason = readResetReason();

  dbgInfo("Reset reason(s):");

  if (reason == 0) {
    dbgInfo("  None (reset reason register was cleared)\n");
  }

  if (reason & POWER_RESETREAS_RESETPIN_Msk) {
    dbgInfo("  Reset pin\n");
  }
  if (reason & POWER_RESETREAS_DOG_Msk) {
    dbgInfo("  Watchdog reset\n");
  }
  if (reason & POWER_RESETREAS_SREQ_Msk) {
    dbgInfo("  Soft reset (NVIC_SystemReset)\n");
  }
  if (reason & POWER_RESETREAS_LOCKUP_Msk) {
    dbgInfo("  CPU lockup\n");
  }
  if (reason & POWER_RESETREAS_OFF_Msk) {
    dbgInfo("  Wake from System OFF\n");
  }
  if (reason & POWER_RESETREAS_LPCOMP_Msk) {
    dbgInfo("  LPCOMP wake-up\n");
  }
  if (reason & POWER_RESETREAS_DIF_Msk) {
    dbgInfo("  Debug interface wake-up\n");
  }
  if (reason & POWER_RESETREAS_NFC_Msk) {
    dbgInfo("  NFC wake-up\n");
  }
}

static const char* getDirection(uint32_t cnf) {
  return ((cnf & GPIO_PIN_CNF_DIR_Msk) >> GPIO_PIN_CNF_DIR_Pos) ? "OUTPUT" : "INPUT ";
}

static const char* getInputConnect(uint32_t cnf) {
  return ((cnf & GPIO_PIN_CNF_INPUT_Msk) >> GPIO_PIN_CNF_INPUT_Pos) ? "DISCONNECTED" : "CONNECTED   ";
}

static const char* getPull(uint32_t cnf) {
  switch ((cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) {
  case GPIO_PIN_CNF_PULL_Disabled:
    return "NONE ";
  case GPIO_PIN_CNF_PULL_Pulldown:
    return "DOWN ";
  case GPIO_PIN_CNF_PULL_Pullup:
    return "UP   ";
  default:
    return "???  ";
  }
}

static const char* getDrive(uint32_t cnf) {
  switch ((cnf & GPIO_PIN_CNF_DRIVE_Msk) >> GPIO_PIN_CNF_DRIVE_Pos) {
  case GPIO_PIN_CNF_DRIVE_S0S1:
    return "S0S1";
  case GPIO_PIN_CNF_DRIVE_H0S1:
    return "H0S1";
  case GPIO_PIN_CNF_DRIVE_S0H1:
    return "S0H1";
  case GPIO_PIN_CNF_DRIVE_H0H1:
    return "H0H1";
  case GPIO_PIN_CNF_DRIVE_D0S1:
    return "D0S1";
  case GPIO_PIN_CNF_DRIVE_D0H1:
    return "D0H1";
  case GPIO_PIN_CNF_DRIVE_S0D1:
    return "S0D1";
  case GPIO_PIN_CNF_DRIVE_H0D1:
    return "H0D1";
  default:
    return "????";
  }
}

static const char* getSense(uint32_t cnf) {
  switch ((cnf & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos) {
  case GPIO_PIN_CNF_SENSE_Disabled:
    return "  -  ";
  case GPIO_PIN_CNF_SENSE_High:
    return "HIGH ";
  case GPIO_PIN_CNF_SENSE_Low:
    return " LOW ";
  default:
    return "  ?  ";
  }
}

void dbgPrintDetailedPinStatus() {
  dbgInfo("  Pin | Dir   | Pull | Drive | InputConn | Sense | Level");
  dbgInfo("  --------------------------------------------------------");

  for (int pin = 0; pin < PINS_COUNT; ++pin) {
    uint32_t pin_number = g_ADigitalPinMap[pin];
    if (pin_number == 0xFFFFFFFF) continue;

    uint32_t cnf = NRF_GPIO->PIN_CNF[pin_number];
    uint32_t level = (NRF_P0->IN >> pin_number) & 0x1;

    String line = "  ";
    line += String(pin);
    line += "   | ";
    line += getDirection(cnf);
    line += " | ";
    line += getPull(cnf);
    line += " | ";
    line += getDrive(cnf);
    line += " | ";
    line += getInputConnect(cnf);
    line += " | ";
    line += getSense(cnf);
    line += " | ";
    line += level ? "HIGH" : "LOW ";

    dbgInfo(line);
  }
}

void dbgPrintInterrupts() {
  dbgInfo("=== NVIC Interrupt Status ===");

  for (int i = 0; i < 8; ++i) {  // nRF52 supports up to 240 IRQs, 32 per ISER/ISPR index
    uint32_t enabled = NVIC->ISER[i];
    uint32_t pending = NVIC->ISPR[i];

    if (enabled == 0 && pending == 0) continue;  // skip empty groups

    dbgInfo("  Group " + String(i * 32) + "–" + String((i + 1) * 32 - 1));

    for (int bit = 0; bit < 32; ++bit) {
      int irq = i * 32 + bit;
      if ((enabled >> bit) & 0x1 || (pending >> bit) & 0x1) {
        String status = "  IRQ " + String(irq);
        if ((enabled >> bit) & 0x1) status += " [EN]";
        if ((pending >> bit) & 0x1) status += " [PENDING]";
        dbgInfo(status);
      }
    }
  }

  // Print current active IRQ (non-zero means ISR running)
  uint32_t ipsr = __get_IPSR();
  if (ipsr > 0) {
    dbgInfo("  Currently in interrupt: IRQ " + String(ipsr - 16));
  }
}
