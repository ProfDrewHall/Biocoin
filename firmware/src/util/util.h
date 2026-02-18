/**
 * @file util.h
 * @brief General firmware utility declarations.
 */

#pragma once

/**
 * @brief Print MCU reset reason flags for debugging startup/reset behavior.
 */
void dbgPrintResetReason();
/**
 * @brief Print GPIO/pin configuration summary for debug diagnostics.
 */
void dbgPrintDetailedPinStatus();
/**
 * @brief Print relevant interrupt enable/pending status for troubleshooting.
 */
void dbgPrintInterrupts();
