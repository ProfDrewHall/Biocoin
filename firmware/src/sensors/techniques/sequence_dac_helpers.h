/**
 * @file sequence_dac_helpers.h
 * @brief Shared helpers for AD5940 DAC sequence block generation.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/techniques/sequence_constants.h"

#include <cstdint>

namespace sensor::sequenceDac {
  constexpr uint32_t kLpdacSettleClocks = 10u;

  inline bool initializeBlockState(int32_t seqLenMaxRaw, uint32_t stepCount, uint32_t block0StartAddr,
                                   uint32_t& stepsRemaining, uint32_t& stepsPerBlock, uint32_t& block0Addr,
                                   uint32_t& block1Addr, uint32_t& currentBlock) {
    const int32_t minSeqLen = static_cast<int32_t>(kSeqLenOneStep * 4u);
    if (seqLenMaxRaw < minSeqLen) return false;

    const int32_t usableSeqLen = seqLenMaxRaw - static_cast<int32_t>(kSeqLenOneStep * 2u);
    if (usableSeqLen <= 0) return false;

    stepsPerBlock = static_cast<uint32_t>(usableSeqLen) / kSeqLenOneStep / 2u;
    if (stepsPerBlock == 0u) return false;

    stepsRemaining = stepCount;
    block0Addr = block0StartAddr;
    block1Addr = block0Addr + stepsPerBlock * kSeqLenOneStep;
    currentBlock = kCurrBlk0;
    return true;
  }

  inline BoolFlag toggleSeq(BoolFlag seq) {
    return seq ? bFALSE : bTRUE;
  }

  inline void writeDacStepToSleep(uint32_t currAddr, uint32_t nextAddr, uint32_t dacData, BoolFlag seqToSeq1) {
    uint32_t seqCmd[kSeqLenOneStep];
    seqCmd[0] = SEQ_WR(REG_AFE_LPDACDAT0, dacData);
    seqCmd[1] = SEQ_WAIT(kLpdacSettleClocks);
    seqCmd[2] = SEQ_WR(seqToSeq1 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                       (nextAddr << BITP_AFE_SEQ1INFO_ADDR) | (kSeqLenOneStep << BITP_AFE_SEQ1INFO_LEN));
    seqCmd[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(currAddr, seqCmd, kSeqLenOneStep);
  }

  inline void writeDacStepToInterrupt(uint32_t currAddr, uint32_t nextAddr, uint32_t dacData, BoolFlag seqToSeq1) {
    uint32_t seqCmd[kSeqLenOneStep];
    seqCmd[0] = SEQ_WR(REG_AFE_LPDACDAT0, dacData);
    seqCmd[1] = SEQ_WAIT(kLpdacSettleClocks);
    seqCmd[2] = SEQ_WR(seqToSeq1 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                       (nextAddr << BITP_AFE_SEQ1INFO_ADDR) | (kSeqLenOneStep << BITP_AFE_SEQ1INFO_LEN));
    seqCmd[3] = SEQ_INT0();
    AD5940_SEQCmdWrite(currAddr, seqCmd, kSeqLenOneStep);
  }

  inline void writeFinalStop(uint32_t currAddr) {
    uint32_t seqCmd[kSeqLenOneStep];
    seqCmd[0] = SEQ_NOP();
    seqCmd[1] = SEQ_NOP();
    seqCmd[2] = SEQ_NOP();
    seqCmd[3] = SEQ_STOP();
    AD5940_SEQCmdWrite(currAddr, seqCmd, kSeqLenOneStep);
  }
} // namespace sensor::sequenceDac
