/**
 * @file build_checks.h
 * @brief Compile-time validation of board, variant, and build assumptions.
 * @details Fails the build early when variant/board configuration does not match hardware expectations.
 */

#pragma once

#include <Arduino.h>

// Ensure we are compiling against the repo-local variant.
#ifndef BIOCOIN_LOCAL_VARIANT
#error "Local variant not in use. Configure platformio.ini to use src/variants/biocoin."
#endif

// This board uses LFRC (not external 32 kHz crystal).
#ifdef USE_LFXO
#error "This board does not have a 32kHz XTAL. Disable USE_LFXO in variant.h."
#endif

// This board does not have the QSPI flash configured.
#ifdef EXTERNAL_FLASH_USE_QSPI
#error "This board does not have an external flash chip. Disable EXTERNAL_FLASH_USE_QSPI in variant.h."
#endif

// Ensure P1 pins are enabled in the active variant.
#if (PINS_COUNT != 38)
#error "Update variant pin count to 38 to enable required P1 pins."
#endif
