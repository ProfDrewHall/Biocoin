#include <Arduino.h>
#include <unity.h>

#include "sensors/techniques/cv/echem_cv.h"
#include "sensors/techniques/dpv/echem_dpv.h"
#include "sensors/techniques/imp/echem_imp.h"
#include "sensors/techniques/iontophoresis/iontophoresis.h"
#include "sensors/techniques/ocp/echem_ocp.h"
#include "sensors/techniques/swv/echem_swv.h"
#include "sensors/techniques/temp/echem_temp.h"

void test_all_techniques_require_parameters_before_start() {
  sensor::EChem_CV cv;
  sensor::EChem_DPV dpv;
  sensor::EChem_Imp imp;
  sensor::EChem_OCP ocp;
  sensor::EChem_SWV swv;
  sensor::EChem_Temp temp;
  sensor::Iontophoresis ionto;

  TEST_ASSERT_FALSE(cv.start());
  TEST_ASSERT_FALSE(dpv.start());
  TEST_ASSERT_FALSE(imp.start());
  TEST_ASSERT_FALSE(ocp.start());
  TEST_ASSERT_FALSE(swv.start());
  TEST_ASSERT_FALSE(temp.start());
  TEST_ASSERT_FALSE(ionto.start());
}

void setup() {
  UNITY_BEGIN();
  RUN_TEST(test_all_techniques_require_parameters_before_start);
  UNITY_END();
}

void loop() {}
