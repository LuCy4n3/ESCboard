/**
 * stspin_driver.c
 * ---------------
 * STSPIN32G4 I2C register access and gate driver configuration.
 *
 * UNLOCK PROTOCOL (from official MCSDK stspin32g4.c):
 *   Lock code = 0xD
 *   To unlock: write 0x2D to register 0x0B
 *   To re-lock: write 0xDD to register 0x0B
 *
 * STATUS register note:
 *   Bit 7 (LOCK) is always set — registers are always locked by default.
 *   Bit 3 (RESET) is always set after power-on — clear with ClearFaults.
 *   any_fault only looks at bits 0-2 (real fault conditions).
 */

#include "stspin_driver.h"
#include "main.h"

#define STSPIN_UNLOCK_BYTE  0x2D
#define STSPIN_LOCK_BYTE    0xDD

extern I2C_HandleTypeDef  hi2c3;

static HAL_StatusTypeDef stspin_unlock(void)
{
    uint8_t key = STSPIN_UNLOCK_BYTE;
    return HAL_I2C_Mem_Write(&hi2c3, STSPIN_ADDR, STSPIN_REG_UNLOCK,
                             I2C_MEMADD_SIZE_8BIT, &key, 1, 100);
}

static void stspin_lock(void)
{
    uint8_t key = STSPIN_LOCK_BYTE;
    HAL_I2C_Mem_Write(&hi2c3, STSPIN_ADDR, STSPIN_REG_UNLOCK,
                      I2C_MEMADD_SIZE_8BIT, &key, 1, 100);
}

HAL_StatusTypeDef STSPIN_ReadReg(uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(&hi2c3, STSPIN_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

HAL_StatusTypeDef STSPIN_WriteReg(uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef ret = stspin_unlock();
    if (ret != HAL_OK) return ret;
    ret = HAL_I2C_Mem_Write(&hi2c3, STSPIN_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    stspin_lock();
    return ret;
}

HAL_StatusTypeDef STSPIN_ClearFaults(void)
{
    /* Official code writes 0xFF to register 0x09 (no unlock needed for CLEAR) */
    uint8_t clr = 0xFFU;
    return HAL_I2C_Mem_Write(&hi2c3, STSPIN_ADDR, STSPIN_REG_CLEAR,
                             I2C_MEMADD_SIZE_8BIT, &clr, 1, 100);
}

HAL_StatusTypeDef STSPIN_GetStatus(STSPIN_Status_t *out)
{
    uint8_t raw = 0;
    HAL_StatusTypeDef ret = STSPIN_ReadReg(STSPIN_REG_STATUS, &raw);
    if (ret != HAL_OK) return ret;

    out->raw      = raw;
    out->vcc_uvlo = (raw & STSPIN_STATUS_VCC_UVLO) != 0;
    out->thsd     = (raw & STSPIN_STATUS_THSD)     != 0;
    out->vdsp     = (raw & STSPIN_STATUS_VDSP)     != 0;
    out->reset    = (raw & STSPIN_STATUS_RESET)    != 0;  /* normal on boot */
    out->lock     = (raw & STSPIN_STATUS_LOCK)     != 0;  /* always set     */

    /* Only real faults — excludes RESET and LOCK which are always set */
    out->any_fault = (raw & STSPIN_STATUS_FAULT_MASK) != 0;
    return HAL_OK;
}

HAL_StatusTypeDef STSPIN_Init(void)
{
    HAL_StatusTypeDef ret;
    uint8_t readback = 0;

    /* Verify I2C is alive */
    ret = STSPIN_ReadReg(STSPIN_REG_POWMNG, &readback);
    if (ret != HAL_OK) return ret;

    /* Clear power-on reset flag and any latched faults */
    STSPIN_ClearFaults();
    HAL_Delay(5);

    /* Set VCC = 10V (internal Buck → gate driver supply).
     * With an 11-12 V bench supply, requesting 12 V leaves no headroom and
     * the STATUS register can latch VCC_UVLO when the bridge starts switching.
     * 10 V gives the gate driver real regulation margin while keeping strong
     * MOSFET drive.
     * Read-modify-write: preserve other bits, set VCC_VAL = 0b01. */
    uint8_t powmng = (readback & ~0x03U) | STSPIN_VCC_10V;
    ret = STSPIN_WriteReg(STSPIN_REG_POWMNG, powmng);
    if (ret != HAL_OK) return ret;

    /* Wait for Buck soft-start (min 4 ms per datasheet) */
    HAL_Delay(10);

    /* Verify VCC field was written */
    ret = STSPIN_ReadReg(STSPIN_REG_POWMNG, &readback);
    if (ret != HAL_OK) return ret;
    if ((readback & 0x03U) != STSPIN_VCC_10V) return HAL_ERROR;

    return HAL_OK;
}


void STSPIN_AllGatesOff(void)   { /* controlled by TIM1 MOE / Motor_Fault path. */ }
