#include <Arduino.h>
#include <unity.h>

#include "sensors/techniques/ca/echem_ca.h"

#include <cmath>
#include <cstring>

namespace {
using sensor::CAParameterPayload;
using sensor::EChem_CA;

bool loadPayload(EChem_CA& ca, const CAParameterPayload& payload) {
  uint8_t bytes[sizeof(CAParameterPayload)];
  std::memcpy(bytes, &payload, sizeof(bytes));
  return ca.loadParameters(bytes, static_cast<uint16_t>(sizeof(bytes)));
}

CAParameterPayload makeValidPayload() {
  CAParameterPayload p{};
  p.samplingInterval = 0.100f;
  p.processingInterval = 1.000f;
  p.maxCurrent = 1.000f;
  p.pulsePotential = 0.0f;
  p.channel = 0u;
  return p;
}
} // namespace

void test_ca_rejects_bad_payload_size() {
  EChem_CA ca;
  uint8_t badBytes[sizeof(CAParameterPayload) - 1] = {};
  TEST_ASSERT_FALSE(ca.loadParameters(badBytes, static_cast<uint16_t>(sizeof(badBytes))));
}

void test_ca_accepts_valid_payload() {
  EChem_CA ca;
  const CAParameterPayload p = makeValidPayload();
  TEST_ASSERT_TRUE(loadPayload(ca, p));
}

void test_ca_rejects_invalid_intervals() {
  EChem_CA ca;
  CAParameterPayload p = makeValidPayload();
  p.samplingInterval = 0.0f;
  TEST_ASSERT_FALSE(loadPayload(ca, p));

  p = makeValidPayload();
  p.processingInterval = 0.0f;
  TEST_ASSERT_FALSE(loadPayload(ca, p));

  p = makeValidPayload();
  p.processingInterval = 0.050f; // less than sampling interval
  TEST_ASSERT_FALSE(loadPayload(ca, p));
}

void test_ca_rejects_invalid_channel() {
  EChem_CA ca;
  CAParameterPayload p = makeValidPayload();
  p.channel = 4u;
  TEST_ASSERT_FALSE(loadPayload(ca, p));
}

void test_ca_rejects_non_finite_values() {
  EChem_CA ca;
  CAParameterPayload p = makeValidPayload();
  p.samplingInterval = NAN;
  TEST_ASSERT_FALSE(loadPayload(ca, p));
}

void test_ca_validates_pulse_potential_bounds() {
  EChem_CA ca;
  CAParameterPayload p = makeValidPayload();

  // Vzero defaults to midscale (1300mV), so valid bias range is [-1100mV, +1100mV].
  p.pulsePotential = -1100.0f;
  TEST_ASSERT_TRUE(loadPayload(ca, p));

  p.pulsePotential = 1100.0f;
  TEST_ASSERT_TRUE(loadPayload(ca, p));

  p.pulsePotential = -1100.1f;
  TEST_ASSERT_FALSE(loadPayload(ca, p));

  p.pulsePotential = 1100.1f;
  TEST_ASSERT_FALSE(loadPayload(ca, p));
}

void test_ca_accepts_large_processing_ratio() {
  EChem_CA ca;
  CAParameterPayload p = makeValidPayload();
  p.samplingInterval = 0.001f;
  p.processingInterval = 10.0f; // exercises FIFO threshold max clamp path
  TEST_ASSERT_TRUE(loadPayload(ca, p));
}

void test_ca_start_fails_without_parameters() {
  EChem_CA ca;
  TEST_ASSERT_FALSE(ca.start());
}

void test_ca_start_fails_after_invalid_parameters() {
  EChem_CA ca;
  CAParameterPayload p = makeValidPayload();
  p.samplingInterval = 0.0f; // invalid -> loadParameters fails and bParaChanged stays false
  TEST_ASSERT_FALSE(loadPayload(ca, p));
  TEST_ASSERT_FALSE(ca.start());
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_ca_rejects_bad_payload_size);
  RUN_TEST(test_ca_accepts_valid_payload);
  RUN_TEST(test_ca_rejects_invalid_intervals);
  RUN_TEST(test_ca_rejects_invalid_channel);
  RUN_TEST(test_ca_rejects_non_finite_values);
  RUN_TEST(test_ca_validates_pulse_potential_bounds);
  RUN_TEST(test_ca_accepts_large_processing_ratio);
  RUN_TEST(test_ca_start_fails_without_parameters);
  RUN_TEST(test_ca_start_fails_after_invalid_parameters);
  UNITY_END();
}

void loop() {}
