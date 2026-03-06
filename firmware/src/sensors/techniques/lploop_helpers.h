/**
 * @file lploop_helpers.h
 * @brief Shared helpers for AD5940 low-power loop configuration.
 */

#pragma once

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"

namespace sensor::lpLoop {
  inline uint32_t clampDacData6Bit(int32_t value) {
    if (value < 0) return 0u;
    if (value > 63) return 63u;
    return static_cast<uint32_t>(value);
  }

  inline uint32_t clampDacData12Bit(int32_t value) {
    if (value < 0) return 0u;
    if (value > 4095) return 4095u;
    return static_cast<uint32_t>(value);
  }

  inline void configureAmperometricLoop(BoolFlag extRtia, uint32_t lpTiaRf, uint32_t lpTiaRl, uint32_t lpTiaRtiaSel,
                                        int32_t dacData6Bit, int32_t dacData12Bit) {
    LPLoopCfg_Type lpLoop = {};
    lpLoop.LpAmpCfg.LpAmpSel = LPAMP0;
    lpLoop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
    lpLoop.LpAmpCfg.LpPaPwrEn = bTRUE;
    lpLoop.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lpLoop.LpAmpCfg.LpTiaRf = lpTiaRf;
    lpLoop.LpAmpCfg.LpTiaRload = lpTiaRl;
    if (extRtia == bTRUE) {
      lpLoop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
      lpLoop.LpAmpCfg.LpTiaSW = LPTIASW(9) | LPTIASW(2) | LPTIASW(4) | LPTIASW(5);
    } else {
      lpLoop.LpAmpCfg.LpTiaRtia = lpTiaRtiaSel;
      lpLoop.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(2) | LPTIASW(4);
    }

    lpLoop.LpDacCfg.LpdacSel = LPDAC0;
    lpLoop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA | LPDACSW_VZERO2LPTIA;
    lpLoop.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lpLoop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lpLoop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lpLoop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lpLoop.LpDacCfg.DataRst = bFALSE;
    lpLoop.LpDacCfg.PowerEn = bTRUE;

    const uint32_t dacData6BitClamped = clampDacData6Bit(dacData6Bit);
    int32_t dacData12BitAdjusted = dacData12Bit;
    const int32_t vzeroBase = static_cast<int32_t>(dacData6BitClamped * 64u);
    if (dacData12BitAdjusted < vzeroBase) --dacData12BitAdjusted;
    lpLoop.LpDacCfg.DacData6Bit = dacData6BitClamped;
    lpLoop.LpDacCfg.DacData12Bit = clampDacData12Bit(dacData12BitAdjusted);

    AD5940_LPLoopCfgS(&lpLoop);
  }

  inline void configureOpenCircuitLoop(float vZeromV, float vBiasmV) {
    LPLoopCfg_Type lpLoop = {};
    lpLoop.LpDacCfg.LpdacSel = LPDAC0;
    lpLoop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lpLoop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA | LPDACSW_VBIAS2PIN;
    lpLoop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lpLoop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lpLoop.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lpLoop.LpDacCfg.DataRst = bFALSE;
    lpLoop.LpDacCfg.PowerEn = bTRUE;
    lpLoop.LpDacCfg.DacData6Bit = clampDacData6Bit(static_cast<int32_t>((vZeromV - AD5940_MIN_DAC_OUTPUT) /
                                                                         AD5940_6BIT_DAC_1LSB));
    lpLoop.LpDacCfg.DacData12Bit = clampDacData12Bit(static_cast<int32_t>((vBiasmV - AD5940_MIN_DAC_OUTPUT) /
                                                                           AD5940_12BIT_DAC_1LSB));

    lpLoop.LpAmpCfg.LpAmpSel = LPAMP0;
    lpLoop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
    lpLoop.LpAmpCfg.LpPaPwrEn = bTRUE;
    lpLoop.LpAmpCfg.LpTiaPwrEn = bFALSE;
    lpLoop.LpAmpCfg.LpTiaRf = LPTIARTIA_10K;
    lpLoop.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
    lpLoop.LpAmpCfg.LpTiaSW = LPTIASW(8) | LPTIASW(2) | LPTIASW(15);

    AD5940_LPLoopCfgS(&lpLoop);
  }
} // namespace sensor::lpLoop
