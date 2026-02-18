#pragma once

#include "drivers/ad5940_hal.h"
// We need to do this since the driver is a C file.
#ifdef __cplusplus
extern "C" {
#endif

#include "ad5940.h"

#ifdef __cplusplus
}
#endif

int32_t AD5940_ConfigureClock();
int32_t AD5940_ConfigureFIFO(uint32_t FIFO_Size, uint32_t FIFO_Src);
int32_t AD5940_ConfigureSequencer(uint32_t SEQ_MemSize);
int32_t AD5940_ConfigureGPIO();
int32_t AD5940_MeasureLFOSC(float* pFreq);
int32_t AD5940_ConfigureInterrupts(uint32_t interrupts);
int32_t AD5940_CalibrateRTIA(float AdcClkFreq, float SysClkFreq, uint32_t LptiaRtiaSel, float RcalVal,
                             fImpPol_Type* RtiaCalValue);
void AD5940_ConfigureAFEReferences(bool use_lp_bandgap, bool use_lp_refbuf, bool use_lp1v1 = true,
                                   bool use_lp1v8 = true);
void AD5940_ConfigureLPLoop(float Vzero, float sensorBias, float TIA_Rf, float TIA_Rl, bool bExtRtia,
                            uint32_t LptiaRtiaSel);
void AD5940_ConfigureDSP(uint32_t muxNsrc, uint32_t muxPsrc, uint32_t ADCPgaGain, uint32_t ADCSinc2Osr, uint32_t ADCSinc3Osr);
