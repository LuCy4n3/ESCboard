/**
 * stspin_driver.h
 * ---------------
 * STSPIN32G4 register definitions and API.
 *
 * STATUS register bit layout (from official stspin32g4.h):
 *   bit 0: vccUvlo  — VCC undervoltage lockout (actual fault)
 *   bit 1: thsd     — thermal shutdown (actual fault)
 *   bit 2: vdsp     — VDS protection triggered (actual fault)
 *   bit 3: reset    — device reset or power-up occurred (NORMAL, always set on boot)
 *   bit 7: lock     — registers locked (NORMAL, always set by default)
 *
 * any_fault only looks at bits 0-2. Bits 3 and 7 are always set
 * on a healthy chip after power-on and must NOT be treated as errors.
 */

#ifndef STSPIN_DRIVER_H
#define STSPIN_DRIVER_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C address: 7-bit = 0x47, shifted for HAL = 0x8E */
#define STSPIN_ADDR             (0x47 << 1)

/* Register addresses (from official stspin32g4.h) */
#define STSPIN_REG_POWMNG       0x01
#define STSPIN_REG_NFAULT       0x02
#define STSPIN_REG_DRVCONF      0x03
#define STSPIN_REG_GATECFG      0x04
#define STSPIN_REG_CSAMP        0x05
#define STSPIN_REG_TRPCONF      0x06
#define STSPIN_REG_PWMCONF      0x07
#define STSPIN_REG_WDCONF       0x08
#define STSPIN_REG_CLEAR        0x09
#define STSPIN_REG_STBY         0x0A
#define STSPIN_REG_UNLOCK       0x0B
#define STSPIN_REG_STATUS       0x80

/* POWMNG register: VCC_VAL field [1:0] */
#define STSPIN_VCC_8V           0x00
#define STSPIN_VCC_10V          0x01
#define STSPIN_VCC_12V          0x02
#define STSPIN_VCC_15V          0x03

/* STATUS register bits — correct layout from official header */
#define STSPIN_STATUS_VCC_UVLO  (1 << 0)   /* VCC undervoltage     */
#define STSPIN_STATUS_THSD      (1 << 1)   /* Thermal shutdown     */
#define STSPIN_STATUS_VDSP      (1 << 2)   /* VDS protection       */
#define STSPIN_STATUS_RESET     (1 << 3)   /* Reset/power-on flag  */
/* bits 4-6: reserved */
#define STSPIN_STATUS_LOCK      (1 << 7)   /* Registers locked     */

/* Actual fault mask — excludes RESET (normal on boot) and LOCK (always set) */
#define STSPIN_STATUS_FAULT_MASK  (STSPIN_STATUS_VCC_UVLO | \
                                   STSPIN_STATUS_THSD     | \
                                   STSPIN_STATUS_VDSP)

typedef struct
{
    uint8_t raw;
    bool    vcc_uvlo;   /* actual fault */
    bool    thsd;       /* actual fault */
    bool    vdsp;       /* actual fault */
    bool    reset;      /* informational — set on every power-on */
    bool    lock;       /* informational — set whenever registers are locked */
    bool    any_fault;  /* true only if vcc_uvlo, thsd, or vdsp are set */
} STSPIN_Status_t;

HAL_StatusTypeDef STSPIN_Init(void);
HAL_StatusTypeDef STSPIN_GetStatus(STSPIN_Status_t *out);
HAL_StatusTypeDef STSPIN_ClearFaults(void);
HAL_StatusTypeDef STSPIN_ReadReg(uint8_t reg, uint8_t *val);
HAL_StatusTypeDef STSPIN_WriteReg(uint8_t reg, uint8_t val);

void STSPIN_AllGatesOff(void);

#endif /* STSPIN_DRIVER_H */
