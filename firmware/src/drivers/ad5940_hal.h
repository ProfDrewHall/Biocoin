#pragma once

#include "drivers/AD5940_Helper.h"

#include <Arduino.h>


// We need to do this since the driver is a C file.
#ifdef __cplusplus
extern "C" {
#endif

#include "ad5940.h"

#ifdef __cplusplus
}
#endif

//extern voidFuncPtr interruptHandler;
//void PortInterruptHandler(void);

// Functions that are called by the driver which interact directly with the hardware
void AD5940_CsSet(void);
void AD5940_CsClr(void);
void AD5940_RstSet(void);
void AD5940_RstClr(void);
void AD5940_Delay10us(uint32_t iTime);
void AD5940_ReadWriteNBytes(unsigned char* pSendBuffer, unsigned char* pRecvBuff, unsigned long length);

void enableAFEInterrupt(void (*handler)());

void disableAFEInterrupt();
int8_t setAFEChannel(uint8_t ch);

void Start_AD5940_SPI(void);
void Stop_AD5940_SPI(void);
uint32_t AD5940_GetMCUIntFlag(void);
uint32_t AD5940_ClrMCUIntFlag(void);

// void current_monitor_callback(TimerHandle_t xTimerID);
