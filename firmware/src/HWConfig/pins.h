/**
 * @file pins.h
 * @brief Board pin assignments and hardware signal mapping.
 * @details Centralizes GPIO mappings so hardware revisions can be managed from a single header.
 */

#pragma once

// MUX/monitoring/indicator signals
#define PIN_MUX_A0 37
#define PIN_MUX_A1 34
#define PIN_MUX_EN 35
#define PIN_VDIV 20
#define PIN_LED 28

// AFE and auxiliary GPIOs
#define PIN_AFE_GPIO1 16
#define PIN_AFE_GPIO2 17
#define PIN_AFE_GPIO3_AUX 12
#define PIN_AFE_GPIO4_AUX 5

// Iontophoresis/power-control signals
#define PIN_IONTOPH_NMOS_CTRL 10
#define PIN_IONTOPH_PMOS_CTRL 6
#define PIN_CURRENT_SENSE_OUT 21
#define PIN_BOOST_EN 13
#define PIN_BUCKBOOST_AUX_EN 8
#define PIN_BUCKBOOST_ANALOG_EN 29
#define PIN_AFE_VDD_CTRL 9
#define PIN_TEMP_VDD_CTRL 11

#define PIN_UARTRX 1

// AD5940 interface
#define PIN_AFE_IntPin_GPIO0 15
#define PIN_AFE_RESET 18
#define PIN_SPI_CS 14
