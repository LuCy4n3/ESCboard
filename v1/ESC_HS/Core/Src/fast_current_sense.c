#include "fast_current_sense.h"
#include "fast_motor_config.h"

FM_RawCurrents g_fm_adc_offset = {2048U, 2048U, 2048U};

static uint16_t s_u[FM_OFFSET_MEDIAN_SAMPLES];
static uint16_t s_v[FM_OFFSET_MEDIAN_SAMPLES];
static uint16_t s_w[FM_OFFSET_MEDIAN_SAMPLES];

static volatile uint16_t s_target_samples;
static volatile uint16_t s_discard_count;
static volatile uint16_t s_sample_count;
static volatile uint8_t s_active;
static volatile uint8_t s_collection_done;

static void sort_u16(uint16_t *a, uint16_t n)
{
    for (uint16_t i = 1U; i < n; ++i) {
        uint16_t key = a[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            --j;
        }
        a[j + 1] = key;
    }
}

void FM_CurrentSenseBeginInjectedCalibration(uint16_t samples)
{
    if (samples == 0U || samples > FM_OFFSET_MEDIAN_SAMPLES)
        samples = FM_OFFSET_MEDIAN_SAMPLES;

    s_target_samples = samples;
    s_discard_count = 0U;
    s_sample_count = 0U;
    s_collection_done = 0U;
    s_active = 1U;
    __DMB();
}

uint8_t FM_CurrentSenseCalibrationActive(void)
{
    return s_active;
}

void FM_CurrentSenseOnInjectedSample(const FM_RawCurrents *raw)
{
    if (!s_active || raw == NULL || s_collection_done)
        return;

    if (s_discard_count < FM_OFFSET_DISCARD_SAMPLES) {
        ++s_discard_count;
        return;
    }

    uint16_t n = s_sample_count;
    if (n < s_target_samples) {
        s_u[n] = raw->iu;
        s_v[n] = raw->iv;
        s_w[n] = raw->iw;
        ++n;
        s_sample_count = n;

        if (n >= s_target_samples) {
            __DMB();
            s_collection_done = 1U;
        }
    }
}

HAL_StatusTypeDef FM_CurrentSenseWaitInjectedCalibration(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (!s_collection_done) {
        if ((uint32_t)(HAL_GetTick() - t0) >= timeout_ms) {
            s_active = 0U;
            return HAL_TIMEOUT;
        }
    }

    __DMB();
    uint16_t n = s_target_samples;
    if (n == 0U || s_sample_count < n) {
        s_active = 0U;
        return HAL_ERROR;
    }

    /* No ISR writes the arrays once collection_done is set.  Sorting is done
     * outside the ISR so calibration does not create a long high-priority
     * interrupt. */
    sort_u16(s_u, n);
    sort_u16(s_v, n);
    sort_u16(s_w, n);

    g_fm_adc_offset.iu = s_u[n / 2U];
    g_fm_adc_offset.iv = s_v[n / 2U];
    g_fm_adc_offset.iw = s_w[n / 2U];

    __DMB();
    s_active = 0U;
    return HAL_OK;
}
