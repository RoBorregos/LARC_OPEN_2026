#pragma once

/**
 * Override de configuración para discord-intech/FreeRTOS-Teensy4.
 *
 * La librería busca este archivo (ver FreeRTOSConfig.h del port, que hace
 * __has_include("Teensy4FreeRTOSConfig.h")) antes de caer a su
 * FreeRTOSConfig_default.h. Se usa aquí solo para habilitar
 * uxTaskGetStackHighWaterMark(), que taskHealthMonitor() (en
 * src/main_rtos_sensors.cpp) necesita y que viene deshabilitado por
 * default (INCLUDE_uxTaskGetStackHighWaterMark 0).
 */

#include "FreeRTOSConfig_default.h"

#undef INCLUDE_uxTaskGetStackHighWaterMark
#define INCLUDE_uxTaskGetStackHighWaterMark 1
