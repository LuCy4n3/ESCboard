#ifndef FAST_PWM_H
#define FAST_PWM_H

#include "main.h"

extern TIM_HandleTypeDef htim1;

static inline void FM_PWM_Init(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

    uint32_t mid = TIM1->ARR / 2U;
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM1->CCR3 = mid;
    TIM1->EGR = TIM_EGR_UG;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* ADC trigger */

    __HAL_TIM_MOE_DISABLE(&htim1);
}

static inline void FM_PWM_Enable(void)  { __HAL_TIM_MOE_ENABLE(&htim1); }

static inline void FM_PWM_Disable(void)
{
    __HAL_TIM_MOE_DISABLE(&htim1);
    uint32_t mid = TIM1->ARR / 2U;
    TIM1->CCR1 = mid; TIM1->CCR2 = mid; TIM1->CCR3 = mid;
}


/* True six-step / trapezoidal bridge drive.
 *
 * One phase is PWM sourced from +VBUS, one phase is clamped to the low side
 * (CCR=0 with complementary PWM enabled), and the third phase has BOTH main
 * and complementary outputs disabled so it is electrically floating.  Only
 * CC1..CC3 bits are touched; CH4 remains running as the ADC trigger.
 *
 * Sector order is positive electrical rotation:
 *   0: U+ V-  W floating
 *   1: U+ W-  V floating
 *   2: V+ W-  U floating
 *   3: V+ U-  W floating
 *   4: W+ U-  V floating
 *   5: W+ V-  U floating
 */
static inline void FM_PWM_EnterSixStepMode(void)
{
    const uint32_t phase_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                TIM_CCER_CC3E | TIM_CCER_CC3NE;

    /* A six-step commutation changes which leg is floating.  Make CCR writes
     * immediate while six-step owns TIM1 so a newly-enabled source/sink leg
     * can never run for part of a PWM cycle with the PREVIOUS sector's compare. */
    TIM1->CCER &= ~phase_mask;
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3PE;
}

static inline void FM_PWM_EnableAllPhaseOutputs(void)
{
    const uint32_t phase_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                TIM_CCER_CC3E | TIM_CCER_CC3NE;

    /* Leave six-step through a neutral 50/50/50 vector.  Because six-step has
     * immediate CCR writes enabled, write the neutral compares BEFORE restoring
     * preload and BEFORE enabling the formerly floating phase.  This prevents a
     * one-PWM-cycle topology/current kick at the six-step -> FOC boundary. */
    TIM1->CCER &= ~phase_mask;
    uint32_t mid = TIM1->ARR / 2U;
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM1->CCR3 = mid;
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->CCER |= phase_mask;
}

static inline void FM_PWM_SixStep(uint8_t sector, float source_duty)
{
    const uint32_t phase_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                TIM_CCER_CC3E | TIM_CCER_CC3NE;
    const uint32_t en1 = TIM_CCER_CC1E | TIM_CCER_CC1NE;
    const uint32_t en2 = TIM_CCER_CC2E | TIM_CCER_CC2NE;
    const uint32_t en3 = TIM_CCER_CC3E | TIM_CCER_CC3NE;

    if (source_duty < 0.0f) source_duty = 0.0f;
    if (source_duty > 0.25f) source_duty = 0.25f;

    const uint32_t arr = TIM1->ARR;
    const uint32_t pwm = (uint32_t)(source_duty * (float)arr);
    uint32_t c1 = 0U, c2 = 0U, c3 = 0U;
    uint32_t enable = 0U;

    switch (sector % 6U) {
    case 0U: /* U+ V-, W float */ c1 = pwm; c2 = 0U;  enable = en1 | en2; break;
    case 1U: /* U+ W-, V float */ c1 = pwm; c3 = 0U;  enable = en1 | en3; break;
    case 2U: /* V+ W-, U float */ c2 = pwm; c3 = 0U;  enable = en2 | en3; break;
    case 3U: /* V+ U-, W float */ c2 = pwm; c1 = 0U;  enable = en2 | en1; break;
    case 4U: /* W+ U-, V float */ c3 = pwm; c1 = 0U;  enable = en3 | en1; break;
    default: /* W+ V-, U float */ c3 = pwm; c2 = 0U;  enable = en3 | en2; break;
    }

    /* Duty updates within the SAME sector must not blank the bridge every ISR.
     * Only a real commutation (floating phase changes) briefly floats the three
     * phase outputs while the immediate CCR values and ownership are replaced. */
    uint32_t current_enable = TIM1->CCER & phase_mask;
    if (current_enable != enable)
        TIM1->CCER &= ~phase_mask;

    TIM1->CCR1 = c1;
    TIM1->CCR2 = c2;
    TIM1->CCR3 = c3;

    if (current_enable != enable)
        TIM1->CCER = (TIM1->CCER & ~phase_mask) | enable;
}

static inline void FM_PWM_Set(float da, float db, float dc)
{
    uint32_t arr = TIM1->ARR;
    TIM1->CCR1 = (uint32_t)(da * (float)arr);
    TIM1->CCR2 = (uint32_t)(db * (float)arr);
    TIM1->CCR3 = (uint32_t)(dc * (float)arr);
}

#endif
