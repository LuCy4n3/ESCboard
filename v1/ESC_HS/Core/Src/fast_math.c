#include "fast_math.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FM_TWO_PI             (2.0f * (float)M_PI)
#define FM_Q31_TO_FLOAT       (1.0f / 2147483648.0f)
#define FM_RAD_TO_Q31         (2147483648.0f / (float)M_PI)
#define FM_FLOAT_TO_Q31       2147483647.0f

void FM_CordicInit(void)
{
    /* The project intentionally does not use the HAL CORDIC driver.
     * Enable and reset the peripheral directly through CMSIS registers. */
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    (void)RCC->AHB1ENR;

    RCC->AHB1RSTR |= RCC_AHB1RSTR_CORDICRST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CORDICRST;
}

float FM_WrapPi(float x)
{
    /* Never allow NaN/Inf to propagate into the CORDIC Q31 conversion.
     * Converting a non-finite float to int32_t is undefined and can become a
     * UsageFault on Cortex-M4 when FP exception trapping is enabled. */
    if (!isfinite(x)) return 0.0f;
    while (x > (float)M_PI)  x -= FM_TWO_PI;
    while (x <= -(float)M_PI) x += FM_TWO_PI;
    return x;
}

void FM_CordicSinCos(float theta_rad, float *s, float *c)
{
    if (!isfinite(theta_rad)) theta_rad = 0.0f;
    float a = FM_WrapPi(theta_rad);
    int32_t q = (int32_t)(a * FM_RAD_TO_Q31);

    /* CORDIC is used only by the 40-kHz motor ISR in this application. */
    /* Direct CMSIS register configuration.
     * COSINE function = FUNC 0 (all FUNC bits clear)
     * precision 5 cycles = PRECISION field value 5
     * scale 0, one 32-bit argument, two 32-bit results.
     *
     * We intentionally use the device-register bit definitions here instead
     * of HAL CORDIC aliases. This keeps the 40-kHz ISR math independent of
     * HAL_CORDIC_MODULE_ENABLED/header ordering. */
    /* IMPORTANT: COSINE/SINE has two conceptual inputs: angle and modulus.
     * AN5325's one-write example relies on the modulus still being the reset
     * default of 1.0.  This motor controller alternates PHASE/atan2 and
     * COS/SIN on the same CORDIC peripheral, so that assumption is NOT valid:
     * the second argument can retain data from the previous PHASE operation.
     * Always supply modulus=1 explicitly.  This is the ST-recommended
     * workaround for mixed PHASE + SIN/COS use on STM32G4. */
    CORDIC->CSR = (CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_0)
                | CORDIC_CSR_NARGS
                | CORDIC_CSR_NRES;

    CORDIC->WDATA = (uint32_t)q;
    CORDIC->WDATA = 0x7FFFFFFFU; /* modulus = +1.0 in Q1.31 */
    *c = (float)(int32_t)CORDIC->RDATA * FM_Q31_TO_FLOAT;
    *s = (float)(int32_t)CORDIC->RDATA * FM_Q31_TO_FLOAT;
}

float FM_CordicAtan2(float y, float x)
{
    if (!isfinite(x) || !isfinite(y)) return 0.0f;
    float ax = fabsf(x);
    float ay = fabsf(y);
    float scale = ax + ay;

    if (scale < 1.0e-9f) return 0.0f;

    /* Keep the VECTOR modulus below 1.0, not merely each component.
     * L1 normalization guarantees sqrt(xn^2+yn^2) <= 0.95. */
    float inv_scale = 0.95f / scale;
    float xn = x * inv_scale;
    float yn = y * inv_scale;
    int32_t xq = (int32_t)(xn * FM_FLOAT_TO_Q31);
    int32_t yq = (int32_t)(yn * FM_FLOAT_TO_Q31);

    /* PHASE function = FUNC value 2 (FUNC_1 set).
     * Two Q1.31 arguments (x then y), two Q1.31 results
     * (phase/pi then modulus), precision 5 cycles, scale 0. */
    CORDIC->CSR = CORDIC_CSR_FUNC_1
                | (CORDIC_CSR_PRECISION_2 | CORDIC_CSR_PRECISION_0)
                | CORDIC_CSR_NARGS
                | CORDIC_CSR_NRES;

    CORDIC->WDATA = (uint32_t)xq;
    CORDIC->WDATA = (uint32_t)yq;

    /* PHASE: RES1 = phase/pi, RES2 = modulus. */
    int32_t phase_q31 = (int32_t)CORDIC->RDATA;
    (void)CORDIC->RDATA;
    return (float)phase_q31 * ((float)M_PI / 2147483648.0f);
}

uint8_t FM_CordicSelfTest(float *sin45_out, float *cos45_out)
{
    /* Deliberately execute PHASE first, then COS/SIN.  This reproduces the
     * exact operation ordering that corrupted the old implementation when the
     * hidden modulus argument was allowed to persist. */
    float a = FM_CordicAtan2(0.5f, 0.5f);
    float s45 = 0.0f, c45 = 0.0f;
    FM_CordicSinCos((float)M_PI * 0.25f, &s45, &c45);

    float s90 = 0.0f, c90 = 0.0f;
    (void)FM_CordicAtan2(-0.30f, 0.70f);
    FM_CordicSinCos((float)M_PI * 0.5f, &s90, &c90);

    if (sin45_out != NULL) *sin45_out = s45;
    if (cos45_out != NULL) *cos45_out = c45;

    const float SQRT_HALF = 0.70710678f;
    if (fabsf(a - (float)M_PI * 0.25f) > 0.02f) return 0U;
    if (fabsf(s45 - SQRT_HALF) > 0.02f) return 0U;
    if (fabsf(c45 - SQRT_HALF) > 0.02f) return 0U;
    if (fabsf(s90 - 1.0f) > 0.02f) return 0U;
    if (fabsf(c90) > 0.02f) return 0U;
    return 1U;
}

float FM_FastSqrt(float x)
{
    /* One sqrt per ISR is unnecessary. This helper is used only where the
     * actual magnitude is needed. sqrtf is acceptable at low-rate points. */
    return (x > 0.0f) ? sqrtf(x) : 0.0f;
}
