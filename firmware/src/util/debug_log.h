#pragma once

#include <Arduino.h>

#define DBG_LEVEL_NONE 0
#define DBG_LEVEL_ERROR 1
#define DBG_LEVEL_WARN 2
#define DBG_LEVEL_INFO 3

#define DEBUG_LEVEL DBG_LEVEL_INFO // Customize level here

#ifdef DEBUG_MODE

template <typename T>
void dbgInfo(T msg) {
#if DEBUG_LEVEL >= DBG_LEVEL_INFO
  Serial.println(msg);
  Serial.flush();
#endif
}

template <typename T>
void dbgWarn(T msg) {
#if DEBUG_LEVEL >= DBG_LEVEL_WARN
  Serial.print("[WARNING] ");
  Serial.println(msg);
  Serial.flush();
#endif
}

template <typename T>
void dbgError(T msg) {
#if DEBUG_LEVEL >= DBG_LEVEL_ERROR
  Serial.print("[ERROR] ");
  Serial.println(msg);
  Serial.flush();
#endif
}

#else

#define dbgInfo(msg)
#define dbgWarn(msg)
#define dbgError(msg)

#endif