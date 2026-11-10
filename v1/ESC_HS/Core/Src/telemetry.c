#include "telemetry.h"
#include "main.h"
#include "v13_capture.h"
#include "fast_motor_config.h"
#include <string.h>

/* Build 26100901: production-light telemetry plus fault-13 current capture.
 *
 * Control data is mirrored at 1 kHz from values already computed by the motor
 * ISR.  The additional current-sense words below are cheap 20-Hz DMA telemetry
 * and make an immediate ADC/op-amp rail trip diagnosable without restoring the
 * old heavy per-ISR telemetry traffic. */
#if FM_V13_20K_SHADOW_TEST
#define TLM_CHANNEL_COUNT  44U /* 26 slow + 9x capture on one packet */
#elif FM_RUN_RAW_OBSERVER_20KHZ
#define TLM_CHANNEL_COUNT  24U
#elif TELEMETRY_V13_SHADOW_CAPTURE
#define TLM_CHANNEL_COUNT  64U /* max 46 status + 17 capture fields */
#else
#define TLM_CHANNEL_COUNT  37U
#endif
#define TLM_FRAME_BYTES     7U
#define TLM_BUFFER_BYTES   (TLM_CHANNEL_COUNT * TLM_FRAME_BYTES)

static uint32_t s_last_cycles;
static uint32_t s_period_cycles;
static uint8_t s_tx_buffer[TLM_BUFFER_BYTES];
static uint32_t s_capture_tx_session = 0xFFFFFFFFU;
static uint32_t s_capture_tx_cursor = 0U;

static int32_t sat_i32_from_float(float x)
{
    if (x >= 2147483520.0f) return INT32_MAX;
    if (x <= -2147483520.0f) return INT32_MIN;
    return (int32_t)x;
}

static uint16_t append_u32(uint8_t *dst, uint16_t p, uint8_t addr, uint32_t value)
{
    dst[p++] = TELEMETRY_HEADER0;
    dst[p++] = TELEMETRY_HEADER1;
    dst[p++] = addr;
    dst[p++] = (uint8_t)(value >> 0);
    dst[p++] = (uint8_t)(value >> 8);
    dst[p++] = (uint8_t)(value >> 16);
    dst[p++] = (uint8_t)(value >> 24);
    return p;
}

static uint16_t append_i32(uint8_t *dst, uint16_t p, uint8_t addr, int32_t value)
{
    return append_u32(dst, p, addr, (uint32_t)value);
}

static uint16_t append_signed_offset(uint8_t *dst, uint16_t p, uint8_t addr,
                                     int32_t value)
{
    uint32_t wire = ((uint32_t)value) ^ 0x80000000U;
    return append_u32(dst, p, addr, wire);
}

void Telemetry_Init(void)
{
    s_last_cycles = DWT->CYCCNT;
    s_period_cycles = (SystemCoreClock / 1000U) * TELEMETRY_PERIOD_MS;
    if (s_period_cycles == 0U) s_period_cycles = 1U;

    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    while ((DMA1_Channel3->CCR & DMA_CCR_EN) != 0U) { }
    DMA1->IFCR = DMA_IFCR_CGIF3;

    DMAMUX1_Channel2->CCR =
        ((uint32_t)DMA_REQUEST_USART1_TX << DMAMUX_CxCR_DMAREQ_ID_Pos) &
        DMAMUX_CxCR_DMAREQ_ID_Msk;

    DMA1_Channel3->CPAR  = (uint32_t)&USART1->TDR;
    DMA1_Channel3->CMAR  = (uint32_t)s_tx_buffer;
    DMA1_Channel3->CNDTR = 0U;
    DMA1_Channel3->CCR   = DMA_CCR_MINC | DMA_CCR_DIR;
    SET_BIT(USART1->CR3, USART_CR3_DMAT);
}

void Telemetry_Service(const FM_Telemetry *t)
{
    if (t == NULL) return;

    uint32_t now_cycles = DWT->CYCCNT;
    if ((uint32_t)(now_cycles - s_last_cycles) < s_period_cycles) return;
    if (DMA1_Channel3->CNDTR != 0U) return;
    s_last_cycles = now_cycles;

    uint8_t *b = s_tx_buffer;
    uint16_t p = 0U;

#if FM_V13_20K_SHADOW_TEST
    /* 20kHz frozen V13 SHADOW with original 20kHz RAW observer.
     * No telemetry serialization in ISR. Capture transmits one 32-byte
     * record (=8 words) per 50ms DMA packet once frozen. */
    p = append_i32(b,p,TLM_ADDR_STATE,(int32_t)t->state);
    p = append_signed_offset(b,p,TLM_ADDR_RPM,sat_i32_from_float(t->rpm));
    p = append_signed_offset(b,p,TLM_ADDR_TARGET_RPM,sat_i32_from_float(t->target_rpm));
    p = append_i32(b,p,TLM_ADDR_FIRMWARE_BUILD_ID,(int32_t)t->firmware_build_id);
    p = append_i32(b,p,TLM_ADDR_FAULT,(int32_t)t->fault);
    p = append_i32(b,p,TLM_ADDR_STSPIN_STATUS,(int32_t)t->stspin_status_raw);
    p = append_i32(b,p,TLM_ADDR_OBSERVER20K_MODE,(int32_t)(t->state == FM_STATE_RUN?t->observer20k_mode:0U));
    p = append_i32(b,p,TLM_ADDR_OBSERVER20K_UPDATES,(int32_t)t->observer20k_updates);
    p = append_signed_offset(b,p,TLM_ADDR_RAW_FLUX_RPM,sat_i32_from_float(t->raw_flux_rpm));
    p = append_signed_offset(b,p,TLM_ADDR_IQ,sat_i32_from_float(t->iq_a*1000.0f));
    p = append_i32(b,p,TLM_ADDR_VBUS,sat_i32_from_float(t->vbus_v*1000.0f));
    p = append_i32(b,p,TLM_ADDR_ISR_OVERRUN_COUNT,(int32_t)t->isr_overrun_count);
    p = append_i32(b,p,TLM_ADDR_RUN_OBSERVER_CYCLES_LAST,(int32_t)t->observer_run_cycles_last);
    p = append_i32(b,p,TLM_ADDR_V13_FAST_MODE,(int32_t)t->v13_fast_mode);
    p = append_i32(b,p,TLM_ADDR_V13_SAMPLE_INDEX,(int32_t)t->v13_samples);
    p = append_i32(b,p,TLM_ADDR_V13_CYCLES_LAST,(int32_t)t->v13_cycles_last);
    p = append_i32(b,p,TLM_ADDR_V13_CYCLES_MAX,(int32_t)t->v13_cycles_max);
    p = append_i32(b,p,TLM_ADDR_V13_FAST_OVERRUNS,(int32_t)t->v13_fast_overruns);
    p = append_i32(b,p,TLM_ADDR_V13_GAP_RESYNCS,(int32_t)t->v13_gap_resyncs);
    p = append_i32(b,p,TLM_ADDR_V13_REJECTED_SAMPLES,(int32_t)t->v13_rejected_samples);
    p = append_i32(b,p,TLM_ADDR_V13_ALPHA,sat_i32_from_float(t->v13_alpha*1000000.0f));
    p = append_i32(b,p,TLM_ADDR_V13_BASE_ALPHA,sat_i32_from_float(t->v13_base_alpha*1000000.0f));
    p = append_i32(b,p,TLM_ADDR_V13_VOLTAGE_EVIDENCE,sat_i32_from_float(t->v13_voltage_evidence*1000000.0f));
    p = append_i32(b,p,TLM_ADDR_V13_CAPTURE_STATUS,(int32_t)g_v13_ram_capture.status);
    p = append_i32(b,p,TLM_ADDR_V13_CAPTURE_COUNT,(int32_t)g_v13_ram_capture.count);
    p = append_signed_offset(b,p,TLM_ADDR_OBSERVER_PHASE_ERROR,sat_i32_from_float(t->phase_error_rad*1000.0f));
    if (g_v13_ram_capture.status == 2U) {
        if (s_capture_tx_session != g_v13_ram_capture.session) {
            s_capture_tx_session = g_v13_ram_capture.session;
            s_capture_tx_cursor = 0U;
        }
        const V13_CaptureFrame *f = NULL;
        if (V13_CaptureRead(s_capture_tx_cursor, &f)) {
            p = append_u32(b, p, TLM_ADDR_CAPTURE_SEQ, s_capture_tx_cursor);
            uint32_t words[8];
            memcpy(words, f, sizeof(words));
            for (unsigned i=0U; i<8U; ++i)
                p = append_u32(b,p,(uint8_t)(TLM_ADDR_CAPTURE_WORD0+i),words[i]);
            ++s_capture_tx_cursor;
        }
    }
#elif FM_RUN_RAW_OBSERVER_20KHZ
    /* Exactly 22 channels, 20Hz. No V13/large RAM-capture export. */
    p = append_i32(b,p,TLM_ADDR_STATE,(int32_t)t->state);
    p = append_signed_offset(b,p,TLM_ADDR_RPM,sat_i32_from_float(t->rpm));
    p = append_signed_offset(b,p,TLM_ADDR_TARGET_RPM,sat_i32_from_float(t->target_rpm));
    p = append_i32(b,p,TLM_ADDR_FIRMWARE_BUILD_ID,(int32_t)t->firmware_build_id);
    p = append_i32(b,p,TLM_ADDR_FAULT,(int32_t)t->fault);
    p = append_i32(b,p,TLM_ADDR_STSPIN_STATUS,(int32_t)t->stspin_status_raw);
    p = append_i32(b,p,TLM_ADDR_OBSERVER20K_MODE,(int32_t)(t->state == FM_STATE_RUN ? t->observer20k_mode : 0U));
    p = append_i32(b,p,TLM_ADDR_OBSERVER20K_UPDATES,(int32_t)t->observer20k_updates);
    p = append_i32(b,p,TLM_ADDR_OBSERVER20K_PREDICTS,(int32_t)t->observer20k_predicts);
    p = append_signed_offset(b,p,TLM_ADDR_RAW_FLUX_RPM,sat_i32_from_float(t->raw_flux_rpm));
    p = append_signed_offset(b,p,TLM_ADDR_OBSERVER_PHASE_ERROR,sat_i32_from_float(t->phase_error_rad*1000.0f));
    p = append_i32(b,p,TLM_ADDR_OBSERVER_PHASE_VALID,(int32_t)t->observer_phase_valid);
    p = append_i32(b,p,TLM_ADDR_OBSERVER_LOCKED,(int32_t)t->observer_locked);
    p = append_signed_offset(b,p,TLM_ADDR_REJOIN_ANGLE_ERROR,sat_i32_from_float(t->rejoin_angle_error_rad*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_IQ,sat_i32_from_float(t->iq_a*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_IQ_REF,sat_i32_from_float(t->iq_ref_a*1000.0f));
    p = append_i32(b,p,TLM_ADDR_VBUS,sat_i32_from_float(t->vbus_v*1000.0f));
    p = append_i32(b,p,TLM_ADDR_ISR_CYCLES_LAST,(int32_t)t->isr_cycles_last);
    p = append_i32(b,p,TLM_ADDR_ISR_CYCLES_MAX,(int32_t)t->isr_cycles_max);
    p = append_i32(b,p,TLM_ADDR_ISR_OVERRUN_COUNT,(int32_t)t->isr_overrun_count);
    p = append_i32(b,p,TLM_ADDR_RUN_OBSERVER_CYCLES_LAST,(int32_t)t->observer_run_cycles_last);
    p = append_i32(b,p,TLM_ADDR_RUN_OBSERVER_CYCLES_MAX,(int32_t)t->observer_run_cycles_max);
#elif TELEMETRY_V13_SHADOW_CAPTURE
    /* Reduced periodic diagnostic words at 20Hz. All non-filtered math done
     * outside 40kHz control; channel IDs remain backward compatible. */
    p = append_i32(b,p,TLM_ADDR_STATE, (int32_t)t->state);
    p = append_signed_offset(b,p,TLM_ADDR_RPM,sat_i32_from_float(t->rpm));
    p = append_signed_offset(b,p,TLM_ADDR_TARGET_RPM,sat_i32_from_float(t->target_rpm));
    p = append_i32(b,p,TLM_ADDR_FAULT,(int32_t)t->fault);
    p = append_i32(b,p,TLM_ADDR_FIRMWARE_BUILD_ID,(int32_t)t->firmware_build_id);
    p = append_i32(b,p,TLM_ADDR_V13_SAMPLE_INDEX,(int32_t)t->v13_samples);
    p = append_i32(b,p,TLM_ADDR_V13_DECIMATION,(int32_t)t->v13_shadow_decimation);
    p = append_i32(b,p,TLM_ADDR_V13_MISSED_SAMPLES,(int32_t)t->v13_missed_samples);
    p = append_i32(b,p,TLM_ADDR_V13_GAP_RESYNCS,(int32_t)t->v13_gap_resyncs);
    p = append_i32(b,p,TLM_ADDR_V13_SESSION_RESETS,(int32_t)t->v13_reset_count);
    p = append_i32(b,p,TLM_ADDR_V13_CONTINUITY_INVALID,(int32_t)t->v13_continuity_invalid);
    p = append_i32(b,p,TLM_ADDR_V13_ALPHA,sat_i32_from_float(t->v13_alpha * 1000000.0f));
    p = append_i32(b,p,TLM_ADDR_V13_BASE_ALPHA,sat_i32_from_float(t->v13_base_alpha * 1000000.0f));
    p = append_i32(b,p,TLM_ADDR_V13_VOLTAGE_EVIDENCE,sat_i32_from_float(t->v13_voltage_evidence*1000000.0f));
    /* No-voltage variant disabled in this performance test (still in source). */
    p = append_i32(b,p,TLM_ADDR_V13_CPU_CYCLES_LAST,(int32_t)t->v13_cpu_cycles_last);
    p = append_i32(b,p,TLM_ADDR_ISR_OVERRUN_COUNT,(int32_t)t->isr_overrun_count);
    p = append_i32(b,p,TLM_ADDR_V13_CAPTURE_STATUS,(int32_t)g_v13_ram_capture.status);
    p = append_i32(b,p,TLM_ADDR_V13_CAPTURE_COUNT,(int32_t)g_v13_ram_capture.count);
    /* Paired 4kHz-filter snapshot, decimated only for 20Hz Excel trends.
     * Accurate phase lag/noise decisions require the separate RAM capture. */
    p = append_signed_offset(b,p,TLM_ADDR_IU,sat_i32_from_float(t->iu_a*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_IV,sat_i32_from_float(t->iv_a*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_IW,sat_i32_from_float(t->iw_a*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_V13_IU,sat_i32_from_float(t->v13_iu_a*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_V13_IV,sat_i32_from_float(t->v13_iv_a*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_V13_IW,sat_i32_from_float(t->v13_iw_a*1000.0f));
    /* Projection-only and research comparator omitted from slow UART.
     * Existing RAM capture still retains projection samples. */
    p = append_i32(b,p,TLM_ADDR_V13_40K_GAIN,
                   sat_i32_from_float(t->v13_40k_gain*1000000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_V13_40K_ALPHA,
                             sat_i32_from_float(t->v13_40k_alpha_current*1000.0f));
    p = append_signed_offset(b,p,TLM_ADDR_V13_40K_BETA,
                             sat_i32_from_float(t->v13_40k_beta_current*1000.0f));
    p = append_i32(b,p,TLM_ADDR_V13_40K_CYCLES,(int32_t)t->v13_40k_cycles_last);
    p = append_i32(b,p,TLM_ADDR_V13_40K_CYCLES_MAX,(int32_t)t->v13_40k_cycles_max);
    p = append_i32(b,p,TLM_ADDR_V13_40K_SAMPLES,(int32_t)t->v13_40k_samples);
    p = append_i32(b,p,TLM_ADDR_V13_40K_GUARD_EVENTS,(int32_t)t->v13_40k_guard_events);
    p = append_i32(b,p,TLM_ADDR_V13_40K_GAIN_AGE,(int32_t)t->v13_40k_gain_age_ticks);
    p = append_signed_offset(b,p,TLM_ADDR_RAW_FLUX_RPM,sat_i32_from_float(t->raw_flux_rpm));
    p = append_signed_offset(b,p,TLM_ADDR_OBSERVER_PHASE_ERROR,sat_i32_from_float(t->phase_error_rad*1000.0f));
    p = append_i32(b,p,TLM_ADDR_VBUS,sat_i32_from_float(t->vbus_v*1000.0f));
    p = append_i32(b,p,TLM_ADDR_ISR_CYCLES_LAST,(int32_t)t->isr_cycles_last);
    p = append_i32(b,p,TLM_ADDR_ISR_CYCLES_MAX,(int32_t)t->isr_cycles_max);
    /* Once frozen, export exactly one 64-byte record per 20Hz DMA packet.
     * Existing AA55 4-byte framing. No UART serialization in ADC ISR.
     * Capture parser groups D2..DB chunks by D1 index and writes CSV. */
    if (g_v13_ram_capture.status == 2U) {
        if (s_capture_tx_session != g_v13_ram_capture.session) {
            s_capture_tx_session = g_v13_ram_capture.session;
            s_capture_tx_cursor = 0U;
        }
        const V13_CaptureFrame *f = NULL;
        if (V13_CaptureRead(s_capture_tx_cursor, &f)) {
            p = append_i32(b, p, TLM_ADDR_CAPTURE_SEQ, (int32_t)s_capture_tx_cursor);
            uint32_t words[16];
            /* memcpy avoids potentially unaligned 32bit loads and aliasing UB. */
            memcpy(words, f, sizeof(words));
            for (unsigned i=0U;i<16U;i++)
                p = append_u32(b,p,(uint8_t)(TLM_ADDR_CAPTURE_WORD0+i),words[i]);
            ++s_capture_tx_cursor;
        }
    }
#else
    p = append_i32(b, p, TLM_ADDR_STATE,                    (int32_t)t->state);
    p = append_signed_offset(b, p, TLM_ADDR_RPM,             sat_i32_from_float(t->rpm));
    p = append_signed_offset(b, p, TLM_ADDR_TARGET_RPM,      sat_i32_from_float(t->target_rpm));
    p = append_signed_offset(b, p, TLM_ADDR_IQ,              sat_i32_from_float(t->iq_a * 1000.0f));
    p = append_signed_offset(b, p, TLM_ADDR_IQ_REF,          sat_i32_from_float(t->iq_ref_a * 1000.0f));
    p = append_i32(b, p, TLM_ADDR_BEMF,                     sat_i32_from_float(t->bemf_v * 1000.0f));
    p = append_signed_offset(b, p, TLM_ADDR_OBSERVER_PHASE_ERROR,
                                                              sat_i32_from_float(t->phase_error_rad * 1000.0f));
    p = append_i32(b, p, TLM_ADDR_FAULT,                    (int32_t)t->fault);
    p = append_signed_offset(b, p, TLM_ADDR_PHASE_CURRENT_SUM,
                                                              sat_i32_from_float(t->phase_current_sum_a * 1000.0f));
    p = append_i32(b, p, TLM_ADDR_PHASE_CURRENT_PEAK,       sat_i32_from_float(t->phase_current_peak_a * 1000.0f));
    p = append_i32(b, p, TLM_ADDR_VBUS,                     sat_i32_from_float(t->vbus_v * 1000.0f));
    p = append_i32(b, p, TLM_ADDR_STSPIN_STATUS,            (int32_t)t->stspin_status_raw);
    p = append_i32(b, p, TLM_ADDR_FIRMWARE_BUILD_ID,        (int32_t)t->firmware_build_id);
    p = append_i32(b, p, TLM_ADDR_ISR_CYCLES_LAST,          (int32_t)t->isr_cycles_last);
    p = append_i32(b, p, TLM_ADDR_ISR_CYCLES_MAX,           (int32_t)t->isr_cycles_max);
    p = append_i32(b, p, TLM_ADDR_ISR_OVERRUN_COUNT,        (int32_t)t->isr_overrun_count);
    p = append_i32(b, p, TLM_ADDR_OBSERVER_PHASE_VALID,     (int32_t)t->observer_phase_valid);
    p = append_signed_offset(b, p, TLM_ADDR_RAW_FLUX_RPM,   sat_i32_from_float(t->raw_flux_rpm));
    p = append_i32(b, p, TLM_ADDR_SIXSTEP_BEMF_SYNC,        (int32_t)t->sixstep_bemf_sync);
    p = append_i32(b, p, TLM_ADDR_NEG_RUN_FALSE_LOCK_ACTIVE,
                                                              (int32_t)t->neg_run_false_lock_active);

    /* Fault-13 / current-sense diagnostics.  raw_jdr* are frozen to the exact
     * faulting injected ADC triplet by FastMotor_ADC2InjectedIRQ().  offset_*
     * are the startup zero-current medians used by the current conversion. */
    p = append_i32(b, p, TLM_ADDR_RAW_JDR1,                  (int32_t)t->raw_iu_counts);
    p = append_i32(b, p, TLM_ADDR_RAW_JDR2,                  (int32_t)t->raw_iv_counts);
    p = append_i32(b, p, TLM_ADDR_RAW_JDR3,                  (int32_t)t->raw_iw_counts);
    p = append_i32(b, p, TLM_ADDR_OFFSET_U,                  (int32_t)t->offset_iu_counts);
    p = append_i32(b, p, TLM_ADDR_OFFSET_V,                  (int32_t)t->offset_iv_counts);
    p = append_i32(b, p, TLM_ADDR_OFFSET_W,                  (int32_t)t->offset_iw_counts);
    p = append_signed_offset(b, p, TLM_ADDR_IU,              sat_i32_from_float(t->iu_a * 1000.0f));
    p = append_signed_offset(b, p, TLM_ADDR_IV,              sat_i32_from_float(t->iv_a * 1000.0f));
    p = append_signed_offset(b, p, TLM_ADDR_IW,              sat_i32_from_float(t->iw_a * 1000.0f));
    p = append_i32(b, p, TLM_ADDR_OVERCURRENT_COUNT,         (int32_t)t->overcurrent_count);
    p = append_i32(b, p, TLM_ADDR_CURRENT_SUM_BAD_COUNT,     (int32_t)t->current_sum_bad_count);
    p = append_i32(b, p, TLM_ADDR_VBUS_ADC_COUNTS,           (int32_t)t->vbus_adc_counts);
    p = append_i32(b, p, TLM_ADDR_VBUS_ADC_VALID,            (int32_t)t->vbus_adc_valid);
    p = append_i32(b, p, TLM_ADDR_TIM1_MOE,                  (int32_t)t->tim1_moe);
    p = append_i32(b, p, TLM_ADDR_TIM1_CCR1,                 (int32_t)t->tim1_ccr1);
    p = append_i32(b, p, TLM_ADDR_TIM1_CCR2,                 (int32_t)t->tim1_ccr2);
    p = append_i32(b, p, TLM_ADDR_TIM1_CCR3,                 (int32_t)t->tim1_ccr3);

#endif

    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    while ((DMA1_Channel3->CCR & DMA_CCR_EN) != 0U) { }
    DMA1->IFCR = DMA_IFCR_CGIF3;
    DMA1_Channel3->CMAR  = (uint32_t)s_tx_buffer;
    DMA1_Channel3->CNDTR = p;
    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
}
