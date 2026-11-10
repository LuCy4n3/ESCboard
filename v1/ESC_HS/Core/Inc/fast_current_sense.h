#ifndef FAST_CURRENT_SENSE_H
#define FAST_CURRENT_SENSE_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef struct {
    uint16_t iu;
    uint16_t iv;
    uint16_t iw;
} FM_RawCurrents;

extern FM_RawCurrents g_fm_adc_offset;

/* Injected-path offset calibration.
 * TIM1 must already be running so its TRGO can trigger ADC2 injected ranks.
 * PWM MOE remains disabled, so the bridge is not energized while samples are
 * collected.  Runtime and calibration therefore use the exact same JDR path. */
void FM_CurrentSenseBeginInjectedCalibration(uint16_t samples);
void FM_CurrentSenseOnInjectedSample(const FM_RawCurrents *raw);
uint8_t FM_CurrentSenseCalibrationActive(void);
HAL_StatusTypeDef FM_CurrentSenseWaitInjectedCalibration(uint32_t timeout_ms);

#endif
