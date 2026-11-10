#include "fast_motor.h"
#include "fast_motor_config.h"
#include "fast_current_sense.h"
#include "v13_lean.h"
#include "v13_capture.h"
#include "fast_math.h"
#include "v13_alpha40_lut.h"
#include "fast_pwm.h"
#include "stspin_driver.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* Hot-path attributes: O3 only, deliberately no fast-math so observer/FOC
 * numerical semantics stay the same as the proven 26091701 baseline. */
#if defined(__GNUC__)
#define FM_HOT_ISR __attribute__((hot,optimize("O3")))
#else
#define FM_HOT_ISR
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FM_TWO_PI       (2.0f * (float)M_PI)
#define FM_INV_SQRT3    0.57735026919f
#define FM_SQRT3_BY_2   0.86602540378f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

/* Forward declaration: the sling/ZVV seed path can reset RUN re-capture state
 * before the helper's full definition later in this translation unit. */
static void observer_recatch_reset(void);
static void clear_run_observer_history_for_sixstep(void);

typedef struct {
    float i_alpha_prev;
    float i_beta_prev;
    float di_alpha;
    float di_beta;
    float e_alpha;
    float e_beta;

    /* PM-flux observer state.  This is the rotor-flux vector itself, not the
     * BEMF vector, so atan2(flux_beta, flux_alpha) is the electrical rotor
     * angle directly.  It is seeded from the known forced trajectory before
     * sensorless takeover. */
    float flux_alpha;
    float flux_beta;
    float flux_theta_prev;
    float flux_omega_e;
    uint8_t flux_initialized;
    uint8_t flux_theta_prev_valid;
    uint32_t flux_speed_sample_count;

    float theta;
    float omega_e;
    float bemf_mag;
    float phase_err;
    /* Raw voltage-model rotor-angle candidate before the pi-polarity branch.
     * This is captured from the same BEMF vector that feeds the PLL so startup
     * can qualify angle coherence without trusting the seeded PLL state. */
    float theta_meas_raw;
    uint8_t theta_meas_valid;
    float voltage_to_rotor_lag_rad;
    uint32_t lock_count;
    uint8_t locked;
    uint8_t polarity_resolved;
    uint8_t polarity_pi;
} FM_Observer;

typedef struct {
    float theta;
    float omega_e;
    float omega_filt;
    float carrier_phase; /* retained for source/debug compatibility; LUT index owns the carrier */
    uint8_t carrier_index;
    float demod_lpf;
    float error_a;
    float err_norm;
    float saliency_scale_a;
    uint8_t motion_coherent;

    float scan_score[FM_HFI_SCAN_ANGLE_COUNT];
    float scan_accum;
    uint32_t scan_sample;
    uint8_t scan_index;

    uint32_t converge_count;

    /* Legacy calibration accumulators kept only so the existing telemetry
     * packet layout remains source-compatible.  Runtime startup no longer
     * commands known electrical angles or performs an open-loop alignment. */
    float cal0_sin_sum;
    float cal0_cos_sum;
    float cal1_sin_sum;
    float cal1_cos_sum;
    uint32_t cal0_samples;
    uint32_t cal1_samples;
    uint32_t cal_verify_count;
    float cal_raw0_rad;
    float cal_raw1_rad;
    float cal_offset_rad;
    float cal_resultant;
    float cal_model_error_rad;
    uint8_t cal_model_valid;

    /* HFI axis/polarity qualification.  Rotor-angle feedback remains HFI
     * closed-loop.  A small +Iq pulse followed by an Iq=0 coast distinguishes
     * persistent rotor displacement from current-induced estimator bias. */
    float polarity_last_theta;
    float polarity_unwrapped_delta;
    uint32_t polarity_sample_count;
    uint8_t polarity_probe_started;
    uint8_t polarity_decision;
    uint8_t polarity_axis_retry; /* 0 first probe, 1 stronger probe retry */
} FM_Hfi;

typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
} FM_Notch1;

typedef struct {
    FM_Telemetry t;
    float v13_inv_arr; /* one-time precomputed CCR normalization */
    FM_Observer obs;
    FM_Hfi hfi;

    float vbus;
    float hfi_to_foc_offset;
    float hfi_angle_sign;
    float int_d;
    float int_q;
    float speed_int;
    float active_target_rpm;
    float launch_id_start_a;
    float forced_theta;
    float forced_omega_e;
    float forced_voltage_v;
    uint32_t forced_observer_good_count;
    float forced_accel_rpm_s;
    float forced_observer_blend;

    /* Early BEMF-angle acquisition and bumpless current-loop handover.
     * The angle qualifier uses a multi-sample residual window against the
     * HFI-seeded rotating reference.  The PI seed uses EVERY coherent sample
     * in the same window (Iq_ref, measured Id/Iq, applied Vd/Vq and integrator
     * state), never a single last sample. */
    uint8_t handover_angle_acquired;
    uint8_t handover_polarity_pi;
    uint32_t handover_angle_samples;
    uint32_t handover_pi_votes;
    float handover_residual_mean;
    float handover_residual_m2;
    float handover_angle_offset_rad;
    uint32_t handover_control_samples;
    float handover_id_meas_avg;
    float handover_iq_meas_avg;
    float handover_iq_ref_avg;
    float handover_vd_avg;
    float handover_vq_avg;
    float handover_int_d_avg;
    float handover_int_q_avg;
    float bemf_boost_iq_cmd_a;
    float bemf_boost_id_cmd_a;
    uint32_t bemf_boost_entry_sample;
    uint32_t foc_capture_load_boost_count;
    uint32_t foc_capture_stall_count;
    float foc_capture_peak_bemf_v;
    uint8_t first_foc_sample_captured;

    float control_feedback_blend;
    float control_theta;
    float control_speed_feedback_rpm;
    float if_iq_ref_a;
    uint32_t feedback_good_count;
    uint32_t hfi_track_bad_count;
    uint32_t hfi_reverse_motion_count;
    uint8_t hfi_pi_flip_done;
    float if_raw_id_sum;
    float if_raw_iq_sum;
    uint32_t if_raw_samples;
    uint8_t if_pi_seeded;
    uint32_t forced_catch_start_ms;
    uint8_t forced_catch_active;
    float handover_id_start_a;
    float handover_iq_start_a;
    float handover_observer_blend_start;
    uint32_t observer_unlock_count;
    uint32_t run_low_speed_confirm_count;
    uint32_t run_entry_grace_count;
    uint32_t neg_run_release_count; /* 1-kHz negative post-catch release ticks */
    float neg_run_release_k;        /* cached 0..1 release authority */
    float neg_run_target_ramp;      /* cached RPM/s so 40-kHz ISR avoids release math */
    uint32_t neg_run_false_lock_count; /* 40-kHz proof of the ~600-rpm phantom RUN condition */
    uint8_t neg_run_false_lock_active;
    uint8_t neg_run_admission_ready;
    uint32_t neg_run_quality_bits;
    float neg_run_iq_limit_a;

    /* Latched by the UART/main-context speed setter. RUN consumes this only
     * after checking whether its sensorless speed feedback is still real. */
    volatile uint8_t speed_command_pending;

    /* Legacy coast/catch telemetry storage retained for protocol compatibility. */
    uint32_t coast_good_count;
    float handover_iq_entry_a;
    float catch_iq_limit_a;
    uint8_t catch_seeded;
    float last_good_observer_theta;
    float last_good_observer_omega_e;
    uint32_t observer_holdover_count;
    uint8_t observer_holdover_active;
    uint8_t last_good_observer_valid;

    /* RUN-state bumpless observer re-capture.  A raw BEMF-angle window is
     * qualified against the propagated holdover trajectory.  Once coherent,
     * authority is blended back to the observer instead of demanding that one
     * instantaneous observer sample land within a small angle gate. */
    uint32_t observer_recatch_samples;
    float observer_recatch_residual_mean;
    float observer_recatch_residual_m2;
    uint8_t observer_recatch_blending;
    uint32_t observer_recatch_blend_count;

    uint32_t state6_entry_good_count;
    float state6_entry_blend;
    uint8_t full_release_qualified;

    uint32_t overcurrent_count;
    uint32_t hard_overcurrent_count;
    uint32_t current_sum_bad_count;
    uint8_t vbus_has_measurement;
    uint8_t vbus_uv_count;
    uint8_t vbus_ov_count;
    uint32_t last_vbus_update_ms;

    /* HFI current path: i_lf now means carrier-notched fundamental current,
     * not a slow low-pass current. */
    FM_AlphaBeta i_lf;
    FM_Notch1 notch_alpha;
    FM_Notch1 notch_beta;
    FM_AlphaBeta last_v_fund;

    /* HFI-seeded slingshot + zero-voltage-vector estimator state. */
    uint8_t sling_phase;
    uint8_t sling_attempt;
    uint8_t sling_axis_quadrature;
    uint8_t sling_reverse_retry;
    uint32_t sling_sample;
    float sling_theta;
    float sling_hfi_seed_theta;
    float sling_i_alpha_prev;
    float sling_i_beta_prev;
    float sling_di_alpha;
    float sling_di_beta;
    float sling_e_alpha;
    float sling_e_beta;
    float sling_prev_bemf_theta;
    float sling_unwrapped_theta;
    float sling_first_bemf_theta;
    float sling_last_bemf_theta;
    float sling_bemf_sum;
    float sling_peak_bemf;
    uint32_t sling_valid_samples;
    uint32_t sling_first_valid_sample;
    uint32_t sling_last_valid_sample;
    uint8_t sling_have_prev_theta;
    uint32_t bemf_boost_bad_count;

    /* True six-step test state.  The sector phase is fractional progress to
     * the next 60-degree commutation boundary. */
    uint8_t trap_sector;
    float trap_sector_phase;
    /* Startup-only six-step commutation phase.  This is NOT a rotor estimate
     * and is never reported as measured speed/angle.  It exists only until the
     * first physically valid floating-phase BEMF zero crossing takes over. */
    float trap_bootstrap_phase;
    float trap_command_rpm;
    float trap_line_voltage_v;
    float trap_iq_avg;
    float trap_id_avg;

    /* Minimal six-step -> FOC angle calibration.  The proven six-step stage
     * runs the flux observer in the background and learns one fixed sector->
     * rotor offset from only a handful of mid-sector samples. */
    uint8_t trap_cal_observer_started;
    uint8_t trap_cal_ready;
    uint32_t trap_cal_samples;
    float trap_cal_sin_sum;
    float trap_cal_cos_sum;
    float trap_cal_offset_rad;

    /* Six-step -> sinusoidal FOC settling/qualification state.  The observer
     * is deliberately prevented from acquiring angle during the topology
     * transient.  D/Q current continuity is checked in the still-forced frame
     * before raw BEMF samples are admitted to the angle window. */
    float trap_foc_prev_id_a;
    float trap_foc_prev_iq_a;
    uint32_t trap_foc_stable_count;
    uint8_t trap_foc_prev_current_valid;
    uint8_t trap_foc_observer_started;

    /* Closed-loop six-step and controlled FOC-capture state. */
    uint8_t trap_bemf_sync_active;
    uint8_t trap_had_bemf_lock;
    uint32_t trap_bemf_sync_good_count;
    uint32_t trap_bemf_sync_bad_count;
    float trap_measured_rpm;
    float trap_sync_speed_err_rpm;
    float trap_sync_phase_err_rad;
    float trap_speed_int_v;
    uint8_t trap_sector_candidate;
    uint8_t trap_sector_candidate_count;

    /* Event-driven six-step zero-cross state.  The floating phase is physical
     * (one bridge leg is Hi-Z); its BEMF is reconstructed from the measured
     * currents and the voltage that was actually applied. */
    uint8_t trap_zc_locked;
    uint8_t trap_zc_armed;
    uint8_t trap_zc_pending;
    uint8_t trap_zc_good_events;
    uint8_t trap_zc_prev_valid;
    float trap_zc_prev_efloat_v;
    float trap_zc_rpm;
    uint32_t trap_sector_ticks;
    uint32_t trap_last_sector_ticks;
    uint32_t trap_zc_countdown_ticks; /* legacy diagnostic: ticks remaining to scheduled commutation */
    uint32_t trap_zc_due_sector_ticks; /* absolute tick from last commutation; ZC should occur near half this */
    uint8_t trap_zc_holdover_sectors;

    /* Dedicated 40-kHz floating-phase model for TRUE six-step ZC timing.
     * It never computes rotor angle.  It estimates phase BEMF directly from
     * e = v_phase - R*i_phase - L*di_phase/dt, then selects only the one
     * hardware leg that is Hi-Z in the active commutation sector. */
    float trap_i_u_prev;
    float trap_i_v_prev;
    float trap_i_w_prev;
    float trap_di_u;
    float trap_di_v;
    float trap_di_w;
    float trap_phase_bemf_u_v;
    float trap_phase_bemf_v_v;
    float trap_phase_bemf_w_v;
    float trap_floating_bemf_v;
    float trap_floating_current_a;
    uint8_t trap_phase_model_valid;

    uint32_t trap_commutation_count;
    FM_AlphaBeta trap_obs_v_accum;
    uint8_t trap_obs_decim_count;

    /* Cached six-step observer products.  The PM-flux observer is updated at
     * 20 kHz, so recomputing two CORDIC angles again on the interleaved 40-kHz
     * current sample only wastes ISR budget and cannot add new information. */
    float trap_theta_cached;
    float trap_bemf_theta_cached;
    float trap_bemf_vector_mag_cached;
    float trap_flux_rpm_cached;
    uint8_t trap_theta_cached_valid;
    uint8_t trap_bemf_theta_valid_cached;
    uint8_t trap_flux_angle_available_cached;
    uint8_t trap_flux_available_cached;

    /* Prevent repeated FOC probes from chopping up a good six-step rotation. */
    uint32_t trap_foc_retry_cooldown_samples;

    /* Last FOC-probe entry snapshot.  Kept across the topology transition so
     * the 40-Hz logger can see the exact state-4 conditions that admitted it. */
    float foc_entry_rpm;
    float foc_entry_command_rpm;
    float foc_entry_slip_rpm;
    float foc_entry_bemf_v;
    float foc_entry_angle_error_rad;
    float foc_entry_speed_error_rpm;
    float foc_entry_phase_current_a;

    /* Count legal observer-owned commutations after flux/BEMF lock.  FOC is not
     * allowed to probe until the rotor has completed a real continuous sector
     * sequence; this rejects in-place twitch / one-sector false catches. */
    uint32_t trap_lock_commutation_start;

    /* State-6 observer is intentionally 20 kHz while current control remains
     * 40 kHz.  Average the actually-applied voltage over each observer interval. */
    FM_AlphaBeta handover_obs_v_accum;
    uint8_t handover_obs_decim_count;

    uint32_t foc_capture_good_count;

    /* DWT timing state for the 40 kHz control callback. */
    uint32_t isr_prev_entry_cycles;
    uint8_t isr_prev_entry_valid;
    uint32_t isr_cycles_last;
    uint32_t isr_cycles_max;
    uint32_t isr_overrun_count;
    uint32_t isr_interval_cycles_last;
    uint32_t isr_interval_cycles_max;
    uint32_t isr_late_count;

    /* RUN high-speed observer-loss diagnostics. */
    uint8_t observer_phase_valid;
    float rejoin_angle_error_rad;

    uint32_t state_start_ms;
    uint32_t state_start_isr_count;
    uint32_t isr_count;
} FM_App;

static FM_App g;
/* Non-blocking RUN-only full 20kHz V13 capture FIFO.
 * SPSC: high-priority ADC ISR is the sole producer; main foreground is the
 * sole consumer. Publication is head-last; tail release is consumer-last.
 * Neither side waits/spins. Full FIFO drops diagnostics, never motor samples.
 * 64 slots x 28 bytes = 1792 bytes; no heap/RTOS needed. */
#define FM_V13_BG_RING_CAPACITY 64U
#define FM_V13_BG_RING_MASK (FM_V13_BG_RING_CAPACITY - 1U)
typedef struct {
    float current[3];
    float bus;
    uint32_t tick;
    uint16_t ccr[3];
    uint8_t voltage_valid;
    uint8_t observer_flags;
    uint16_t padding;
    /* Sample the observer snapshot in ISR alongside the ADC triplet.
     * Cached state PRECEDES this tick's current observer update. */
    float observer_theta;
    float control_theta;
    float phase_error;
    float flux_omega_e;
    float feedback_rpm;
    float id_a;
    float iq_a;
} FM_V13_RingFrame;
static FM_V13_RingFrame s_v13_ring[FM_V13_BG_RING_CAPACITY];
static volatile uint32_t s_v13_ring_head, s_v13_ring_tail, s_v13_ring_dropped;
static uint32_t s_v13_bg_session = 0xFFFFFFFFU;
static uint32_t s_v13_bg_last_tick;
static uint32_t s_v13_bg_gap_count;
static V13_State s_v13_state;
static V13_State s_v13_noevent_state;
/* Separate 40-kHz RESEARCH EMA, driven by the most recent 4-kHz V13 gain.
 * This is not the original V13 mathematics, and NEVER feeds the observer/PI.
 * Only one atomic aligned 32-bit gain publication crosses ISR boundary. */
static volatile float s_v13_40k_cached_gain = 0.0026f;
static volatile uint32_t s_v13_40k_gain_sample_tick;
static float s_v13_40k_a, s_v13_40k_b;
static uint32_t s_v13_40k_session = 0xFFFFFFFFU;
static uint32_t s_v13_40k_updates, s_v13_40k_cycles_max;
static uint8_t s_v13_40k_initialized;
static uint8_t s_v13_40k_guard_off;
static uint32_t s_v13_40k_overbudget_consecutive;
static uint32_t s_v13_40k_guard_events;
static inline void fm_v13_cached40k_update(float iu, float iv, float iw)
{
#if FM_V13_40K_CACHED_OUTPUT && FM_V13_SHADOW_ENABLED
    if (g.t.state != FM_STATE_RUN || s_v13_40k_guard_off) return;
    if (s_v13_40k_session != g.state_start_isr_count) {
        s_v13_40k_session = g.state_start_isr_count;
        s_v13_40k_initialized = 0U;
        s_v13_40k_updates = 0U;
    }
    const uint8_t timed = ((g.isr_count & (FM_V13_40K_TIMING_DIV-1U)) == 0U);
    const uint32_t start = timed ? DWT->CYCCNT : 0U;
    /* Preserve the same three-phase Clarke projection as original V13. */
    const float a = (2.0f * iu - iv - iw) * 0.3333333333333333f;
    const float b = (iv - iw) * 0.5773502691896258f;
    if (!s_v13_40k_initialized) {
        s_v13_40k_a = a;
        s_v13_40k_b = b;
        s_v13_40k_initialized = 1U;
    }
    const float gain = s_v13_40k_cached_gain;
    s_v13_40k_a += gain * (a - s_v13_40k_a);
    s_v13_40k_b += gain * (b - s_v13_40k_b);
    ++s_v13_40k_updates;
    if (timed) {
        uint32_t cycles = DWT->CYCCNT - start;
        if (cycles > s_v13_40k_cycles_max) s_v13_40k_cycles_max = cycles;
        /* Publish diagnostics once per 128 ticks: avoid ISR telemetry traffic. */
        g.t.v13_40k_cycles_last = cycles;
        g.t.v13_40k_cycles_max = s_v13_40k_cycles_max;
        g.t.v13_40k_alpha_current = s_v13_40k_a;
        g.t.v13_40k_beta_current = s_v13_40k_b;
        g.t.v13_40k_gain = gain;
        g.t.v13_40k_gain_age_ticks = s_v13_40k_gain_sample_tick ?
            (uint32_t)(g.isr_count - s_v13_40k_gain_sample_tick) : 0xFFFFFFFFU;
        g.t.v13_40k_samples = s_v13_40k_updates;
        g.t.v13_40k_guard_events = s_v13_40k_guard_events;
    }
#else
    (void)iu; (void)iv; (void)iw;
#endif
}

static uint32_t s_v13_session_resets;
static volatile uint8_t s_v13_gap_invalid;
static volatile uint8_t s_v13_capture_rearm;
V13_CaptureBuffer g_v13_ram_capture;
_Static_assert(sizeof(V13_CaptureFrame) == 32U, "V13 capture record layout changed");

static float rpm_from_omega(float omega_e);
static int16_t capture_current(float amps)
{
    float u = amps * 250.0f; /* 4mA per signed step */
    if (u > 32767.0f) u = 32767.0f;
    if (u < -32768.0f) u = -32768.0f;
    return (int16_t)u;
}
/* Quantization is performed in the foreground, NEVER in the ADC ISR. */
static int16_t capture_s16(float x)
{
    if (!isfinite(x)) return 0;
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)x;
}
static int16_t capture_angle_q15(float radians)
{
    if (!isfinite(radians)) return 0;
    /* Controller and theta_meas_raw use wrapped [-pi, pi]. */
    const float scale = 32767.0f / 3.14159265358979323846f;
    return capture_s16(radians * scale);
}
static uint16_t capture_u16(float unit)
{
    if (unit < 0.0f) unit = 0.0f;
    if (unit > 1.0f) unit = 1.0f;
    return (uint16_t)(unit * 65535.0f + 0.5f);
}
uint8_t V13_CaptureRead(uint32_t index, const V13_CaptureFrame **out)
{
    if (out == NULL || g_v13_ram_capture.status != 2U ||
        index >= g_v13_ram_capture.count) return 0U;
    *out = &g_v13_ram_capture.frames[index];
    return 1U;
}
void FastMotor_V13CaptureArm(void)
{
    /* Service in foreground; no 40kHz ISR state touched. */
    s_v13_capture_rearm = 1U;
}

/* Dedicated 20-kHz original V13 state. Never shared with foreground fallback. */
static V13_State s_v13_fast_state;
static uint32_t s_v13_fast_updates;
static uint32_t s_v13_fast_run_updates;
static uint32_t s_v13_fast_run_start_tick;
static uint32_t s_v13_fast_cycles_max;
static uint32_t s_v13_fast_overruns;
static uint8_t s_v13_fast_disabled;
static uint8_t s_v13_fast_this_isr;
static FM_AlphaBeta s_v13_fast_ab;
/* The 20kHz filter samples on EVEN 40kHz ticks, the observer consumes the
 * paired (filtered current, applied voltage) sample on the following ODD
 * tick. Never apply a current sample with a voltage from another PWM window. */
static FM_AlphaBeta s_v13_fast_raw_ab;
static FM_AlphaBeta s_v13_fast_v_sample;
static uint8_t s_v13_fast_pair_valid;
static uint32_t s_v13_run_observer_cycles_last;
static uint32_t s_v13_run_observer_cycles_max;
/* RUN-only raw-observer decimation. The current loop is never decimated. */
static uint32_t s_run20k_session = 0xFFFFFFFFU;
static FM_AlphaBeta s_run20k_v_accum;
static uint8_t s_run20k_accum_count;
static uint8_t s_run20k_correction_due;
static uint32_t s_run20k_updates;
static uint32_t s_run20k_predicts;
static uint32_t s_run20k_last_observer_tick;

static volatile uint32_t s_v13_sample_count;
static uint32_t s_v13_missed_samples;
static uint32_t s_v13_cycles_max;
static uint32_t s_v13_cpu_cycles_max;
/* Accumulate measured 40-kHz ISR wall time so foreground DWT timing can
 * subtract periods spent preempted by the motor-control ISR. 32-bit wrap is
 * intentional: differences across one filter call are valid modulo 2^32. */
static volatile uint32_t s_v13_isr_cycles_total;
/* Filter publication is one atomic 32-bit per coordinate and an age tag;
 * the ISR never waits, spins, or calls floating-point filtering functions. */
static volatile float s_v13_observer_alpha, s_v13_observer_beta;
static volatile uint32_t s_v13_observer_seq, s_v13_observer_tick;

static void fm_v13_foreground_service(void)
{
#if FM_V13_SHADOW_ENABLED
    /* Bounded foreground processing; never blocks ADC. Safety polling remains
     * ahead of us in FastMotor_Service, commands ahead in main.c. */
    if (g.t.state != FM_STATE_RUN) {
        s_v13_ring_tail = s_v13_ring_head;
        s_v13_bg_session = 0xFFFFFFFFU;
        g.t.v13_fast_mode = 0U;
        return;
    }
    if (s_v13_bg_session != g.state_start_isr_count) {
        s_v13_bg_session = g.state_start_isr_count;
        s_v13_ring_tail = s_v13_ring_head;
        s_v13_bg_last_tick = 0U;
        s_v13_gap_invalid = 0U;
        g.t.v13_age_ticks_max = 0U;
        V13_ResetSession(&s_v13_state);
#if FM_V13_RESEARCH_NOEVENT
        V13_ResetSession(&s_v13_noevent_state);
#endif
        ++s_v13_session_resets;
        g_v13_ram_capture.status = 0U;
        g_v13_ram_capture.count = 0U;
        g_v13_ram_capture.session++;
        g_v13_ram_capture.reset_count = s_v13_session_resets;
        g_v13_ram_capture.discontinuities = s_v13_bg_gap_count;
        s_v13_capture_rearm = 1U;
    }
    if (s_v13_capture_rearm) {
        /* Never reset V13 state just to capture another segment. */
        s_v13_capture_rearm = 0U;
        g_v13_ram_capture.status = 1U;
        g_v13_ram_capture.count = 0U;
        ++g_v13_ram_capture.session;
        g_v13_ram_capture.discontinuities = s_v13_bg_gap_count;
    }
    g.t.v13_fast_mode = 5U; /* continuous frozen V13 4kHz + cached-gain EMA 40kHz */
    g.t.v13_shadow_decimation = FM_V13_SHADOW_DECIMATION;
    g.t.v13_observer_blend = 0.0f;
    for (unsigned count = 0U; count < 8U; ++count) {
        uint32_t tail = s_v13_ring_tail;
        if (tail == s_v13_ring_head) break;
        __DMB();
        FM_V13_RingFrame frame = s_v13_ring[tail & FM_V13_BG_RING_MASK];
        __DMB();
        s_v13_ring_tail = tail + 1U;
        uint8_t gap = 0U;
        if (s_v13_bg_last_tick &&
            (uint32_t)(frame.tick - s_v13_bg_last_tick) != FM_V13_SHADOW_DECIMATION) {
            ++s_v13_bg_gap_count;
            s_v13_gap_invalid = 1U;
            gap = 1U;
            /* Keep the state; DO NOT silently reset on every lost frame.
             * Once a gap occurs, reference-continuity status stays invalid
             * for this RUN session. Reacquire in a fresh RUN. */
        }
        s_v13_bg_last_tick = frame.tick;
        /* Queue + processing latency, in 40kHz ticks (25us nominal).
         * Never delay the motor to reduce this metric. */
        uint32_t sample_age_ticks;
        float duty[3] = {
            (float)frame.ccr[0] * g.v13_inv_arr,
            (float)frame.ccr[1] * g.v13_inv_arr,
            (float)frame.ccr[2] * g.v13_inv_arr
        };
        float frozen[3], without_event[3];
        uint32_t preempt_before = s_v13_isr_cycles_total;
        uint32_t begin = DWT->CYCCNT;
        V13_Update(&s_v13_state, frame.current, duty, frame.bus,
                   frame.voltage_valid, frozen);
#if FM_V13_RESEARCH_NOEVENT
        V13_UpdateNoVoltageResearch(&s_v13_noevent_state, frame.current,
                                     duty, frame.bus, frame.voltage_valid,
                                     without_event);
#else
        /* Unavailable comparator is explicitly zero, never present as V13. */
        without_event[0] = without_event[1] = without_event[2] = 0.0f;
#endif
        /* Precompute / quantize the 40-kHz adaptive-gain conversion outside ISR. */
#if FM_V13_40K_CACHED_OUTPUT
        float alpha4k = s_v13_state.diag.alpha;
        float pos = (alpha4k - 0.008f) * 265.072765073f;
        unsigned gain_ix = (unsigned)pos;
        if (gain_ix > 254U) gain_ix = 254U;
        float frac = pos - (float)gain_ix;
        s_v13_40k_cached_gain = V13_Alpha40Lut[gain_ix] +
            frac * (V13_Alpha40Lut[gain_ix + 1U] - V13_Alpha40Lut[gain_ix]);
        s_v13_40k_gain_sample_tick = frame.tick;
#endif
        uint32_t wall_cycles = DWT->CYCCNT - begin;
        uint32_t preempt_cycles = s_v13_isr_cycles_total - preempt_before;
        uint32_t est_cpu_cycles = wall_cycles > preempt_cycles ?
                                       wall_cycles - preempt_cycles : 0U;
        sample_age_ticks = (uint32_t)(g.isr_count - frame.tick);
        if (sample_age_ticks > g.t.v13_age_ticks_max)
            g.t.v13_age_ticks_max = sample_age_ticks;
        if (wall_cycles > s_v13_cycles_max) s_v13_cycles_max = wall_cycles;
        if (est_cpu_cycles > s_v13_cpu_cycles_max)
            s_v13_cpu_cycles_max = est_cpu_cycles;
        ++s_v13_sample_count;
        if (g_v13_ram_capture.status == 1U) {
            uint32_t ix = g_v13_ram_capture.count;
            if (ix < V13_CAPTURE_LENGTH) {
                V13_CaptureFrame *d = &g_v13_ram_capture.frames[ix];
                d->sample_index = frame.tick;
                float mean = (frame.current[0]+frame.current[1]+frame.current[2]) * (1.0f/3.0f);
                for (unsigned k=0U;k<3U;k++) {
                    d->raw[k] = capture_current(frame.current[k]);
                    d->frozen[k] = capture_current(frozen[k]);
                    d->no_event[k] = capture_current(without_event[k]);
                    d->projected[k] = capture_current(frame.current[k] - mean);
                }
                d->alpha = capture_u16(s_v13_state.diag.alpha);
                d->base_alpha = capture_u16(s_v13_state.diag.base_alpha);
                d->voltage_event = capture_u16(s_v13_state.diag.voltage_evidence);
                d->event_latch = capture_u16(s_v13_state.diag.event_latch);
                d->cpu_cycles = est_cpu_cycles > 65535U ? 65535U : (uint16_t)est_cpu_cycles;
                for (unsigned k=0U;k<3U;k++) d->duty[k] = capture_u16(duty[k]);
                float bus_mv = frame.bus * 1000.0f;
                if (bus_mv < 0.0f) bus_mv = 0.0f;
                if (bus_mv > 65535.0f) bus_mv = 65535.0f;
                d->bus_mv = (uint16_t)bus_mv;
                d->observer_angle_q15 = capture_angle_q15(frame.observer_theta);
                d->control_angle_q15 = capture_angle_q15(frame.control_theta);
                d->observer_phase_error_q15 = capture_angle_q15(frame.phase_error);
                d->flux_rpm = capture_s16(rpm_from_omega(frame.flux_omega_e));
                d->feedback_rpm = capture_s16(frame.feedback_rpm);
                d->id_10ma = capture_s16(frame.id_a * 100.0f);
                d->iq_10ma = capture_s16(frame.iq_a * 100.0f);
                /* Upper 12 bits encode filter completion age in 25us ticks.
                 * Lower 4 bits are last-observer-state validity flags. */
                const uint16_t packed_age = (uint16_t)(sample_age_ticks > 4095U ? 4095U : sample_age_ticks);
                d->observer_flags = (uint16_t)frame.observer_flags | (uint16_t)(packed_age << 4);
                d->flags = (frame.voltage_valid ? 1U : 0U) |
                          (gap ? 2U : 0U) |
                          ((!s_v13_state.diag.input_valid ||
                             (FM_V13_RESEARCH_NOEVENT && !s_v13_noevent_state.diag.input_valid)) ? 4U : 0U);
                __DMB();
                g_v13_ram_capture.count = ix + 1U;
                if (ix + 1U == V13_CAPTURE_LENGTH) {
                    __DMB();
                    g_v13_ram_capture.status = 2U; /* snapshot FROZEN */
                }
            }
        }
        /* Very compact low-rate diagnostics; DMA outside control interrupt. */
        if ((s_v13_sample_count % 40U) == 0U) {
            g.t.iu_a = frame.current[0];
            g.t.iv_a = frame.current[1];
            g.t.iw_a = frame.current[2];
            g.t.v13_iu_a = frozen[0];
            g.t.v13_iv_a = frozen[1];
            g.t.v13_iw_a = frozen[2];
            g.t.v13_noevent_iu_a = without_event[0];
            g.t.v13_noevent_iv_a = without_event[1];
            g.t.v13_noevent_iw_a = without_event[2];
            const float mean = (frame.current[0]+frame.current[1]+frame.current[2]) * (1.0f/3.0f);
            g.t.v13_projection_iu_a = frame.current[0] - mean;
            g.t.v13_projection_iv_a = frame.current[1] - mean;
            g.t.v13_projection_iw_a = frame.current[2] - mean;
            g.t.v13_alpha = s_v13_state.diag.alpha;
            g.t.v13_base_alpha = s_v13_state.diag.base_alpha;
            g.t.v13_voltage_evidence = s_v13_state.diag.voltage_evidence;
            g.t.v13_research_alpha = FM_V13_RESEARCH_NOEVENT ?
                s_v13_noevent_state.diag.alpha : 0.0f;
            g.t.v13_age_ticks_last = sample_age_ticks;
            g.t.v13_cycles_last = wall_cycles;
            g.t.v13_cpu_cycles_last = est_cpu_cycles;
            g.t.v13_samples = s_v13_sample_count;
            g.t.v13_rejected_samples = s_v13_state.rejected_samples;
            g.t.v13_missed_samples = s_v13_ring_dropped;
            g.t.v13_gap_resyncs = s_v13_bg_gap_count; /* now counts gaps, NOT resets */
            g.t.v13_reset_count = s_v13_session_resets;
            g.t.v13_continuity_invalid = s_v13_gap_invalid;
            g.t.v13_capture_state = g_v13_ram_capture.status;
            g.t.v13_capture_count = g_v13_ram_capture.count;
        }
    }
#else
    return;
#endif
}

static FM_AlphaBeta fm_v13_low_rpm_observer_input(FM_AlphaBeta raw)
{
#if FM_V13_RUN_OBSERVER_ASSIST && FM_V13_SHADOW_ENABLED
    const float abs_rpm = fabsf(g.control_speed_feedback_rpm);
#if FM_V13_FAST_20KHZ_ENABLE
    if (!s_v13_fast_disabled) {
        if (s_v13_fast_run_updates >= FM_V13_FAST_WARMUP_UPDATES &&
            abs_rpm < FM_V13_FAST_BLEND_RPM_END) {
            float weight = FM_V13_FAST_BLEND_MAX;
            if (abs_rpm > FM_V13_FAST_BLEND_RPM_BEGIN)
                weight *= (FM_V13_FAST_BLEND_RPM_END - abs_rpm) /
                          (FM_V13_FAST_BLEND_RPM_END - FM_V13_FAST_BLEND_RPM_BEGIN);
            /* If the filter has not updated within 3 ISR periods, observer
             * always receives raw current. The filter never drives the PI. */
            if (s_v13_fast_pair_valid && s_v13_fast_state.initialized &&
                (g.isr_count - s_v13_observer_tick) <= 3U) {
                raw.alpha += weight * (s_v13_fast_ab.alpha - raw.alpha);
                raw.beta += weight * (s_v13_fast_ab.beta - raw.beta);
                g.t.v13_observer_blend = weight;
            }
        }
        return raw;
    }
#endif
    if (abs_rpm >= FM_V13_RUN_OBSERVER_MAX_RPM || !s_v13_sample_count)
        return raw;
    uint32_t a = s_v13_observer_seq;
    if (a & 1U) return raw;
    __DMB();
    float alpha = s_v13_observer_alpha;
    float beta = s_v13_observer_beta;
    uint32_t tick = s_v13_observer_tick;
    __DMB();
    if (a != s_v13_observer_seq ||
        (uint32_t)(g.isr_count - tick) > FM_V13_MAX_SAMPLE_AGE_TICKS ||
        !isfinite(alpha) || !isfinite(beta)) return raw;
    float w = FM_V13_RUN_OBSERVER_MAX_BLEND;
    if (abs_rpm > FM_V13_RUN_OBSERVER_FULL_RPM) {
        w *= (FM_V13_RUN_OBSERVER_MAX_RPM - abs_rpm) /
             (FM_V13_RUN_OBSERVER_MAX_RPM - FM_V13_RUN_OBSERVER_FULL_RPM);
    }
    g.t.v13_observer_blend = w;
    raw.alpha += w * (alpha - raw.alpha);
    raw.beta += w * (beta - raw.beta);
#else
    (void)g;
#endif
    return raw;
}
/* Full original V13 executes once per two 40kHz ADC samples in RUN only.
 * Short timer: two DWT reads per filter sample; no IRQ-subtraction bookkeeper,
 * no printing, no UART and no CORDIC/FMAC handshakes. Cortex-M4F FPU runs all
 * arithmetic; CORDIC assists trigonometry, not this stateful fuzzy filter. */
static inline void fm_v13_fast_run_update(float iu, float iv, float iw)
{
#if FM_V13_FAST_20KHZ_ENABLE && FM_V13_SHADOW_ENABLED
    if (s_v13_fast_disabled || (g.isr_count % FM_V13_FAST_DIV) != 0U)
        return;
    if (s_v13_fast_run_start_tick != g.state_start_isr_count) {
        /* Do not carry stale current history into reverse RUN acquisition;
         * no expf or other expensive coefficient initialization in ISR. */
        V13_ResetSession(&s_v13_fast_state);
        s_v13_fast_run_start_tick = g.state_start_isr_count;
        s_v13_fast_run_updates = 0U;
        s_v13_fast_pair_valid = 0U;
    }
    const float input[3] = {iu, iv, iw};
    const float duty[3] = {
        (float)TIM1->CCR1 * g.v13_inv_arr,
        (float)TIM1->CCR2 * g.v13_inv_arr,
        (float)TIM1->CCR3 * g.v13_inv_arr
    };
    const uint32_t outputs = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                             TIM_CCER_CC2E | TIM_CCER_CC2NE |
                             TIM_CCER_CC3E | TIM_CCER_CC3NE;
    uint8_t valid = (((TIM1->CCER & outputs) == outputs) &&
                     ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)) ? 1U : 0U;
    float filtered[3];
    uint32_t t0 = DWT->CYCCNT;
    V13_Update(&s_v13_fast_state, input, duty, g.vbus, valid, filtered);
    uint32_t filter_cycles = DWT->CYCCNT - t0;
    s_v13_fast_this_isr = 1U;
    ++s_v13_fast_updates;
    ++s_v13_fast_run_updates;
    s_v13_observer_tick = g.isr_count;
    s_v13_fast_ab.alpha = s_v13_fast_state.out[0];
    s_v13_fast_ab.beta = s_v13_fast_state.out[1];
    s_v13_fast_raw_ab.alpha = (2.0f*iu - iv - iw) * (1.0f/3.0f);
    s_v13_fast_raw_ab.beta = (iv - iw) * FM_INV_SQRT3;
    s_v13_fast_v_sample = g.last_v_fund;
    s_v13_fast_pair_valid = s_v13_fast_state.diag.input_valid;
    if (filter_cycles > s_v13_fast_cycles_max)
        s_v13_fast_cycles_max = filter_cycles;
    /* Diagnostics at 1kHz only, not each 20kHz update. */
    if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U) {
        g.t.iu_a = iu; g.t.iv_a = iv; g.t.iw_a = iw;
        g.t.v13_iu_a = filtered[0];
        g.t.v13_iv_a = filtered[1];
        g.t.v13_iw_a = filtered[2];
        g.t.v13_alpha = s_v13_fast_state.diag.alpha;
        g.t.v13_base_alpha = s_v13_fast_state.diag.base_alpha;
        g.t.v13_confidence = s_v13_fast_state.diag.confidence;
        g.t.v13_voltage_evidence = s_v13_fast_state.diag.voltage_evidence;
        g.t.v13_step_latch = s_v13_fast_state.diag.step_latch;
        g.t.v13_cycles_last = filter_cycles;
        g.t.v13_cycles_max = s_v13_fast_cycles_max;
        g.t.v13_samples = s_v13_fast_updates;
        g.t.v13_rejected_samples = s_v13_fast_state.rejected_samples;
        g.t.v13_voltage_valid = valid;
        g.t.v13_shadow_decimation = FM_V13_FAST_DIV;
        g.t.v13_fast_mode = 1U;
    }
#endif
}

/* 26101703: full V13, shadow-only, opposite the 20kHz observer update.
 * The observer consumes original raw currents. No filter output is used by
 * current PI, overcurrent logic, rotor estimator, or any safety gate. */
static uint32_t s_shadow20_session = 0xFFFFFFFFU;
static uint32_t s_shadow20_updates, s_shadow20_cycles_last;
static uint32_t s_shadow20_cycles_max, s_shadow20_overruns;
static uint8_t s_shadow20_disabled, s_shadow20_this_isr;
static uint32_t s_shadow20_prev_tick;
static uint32_t s_shadow20_gaps;

static inline int16_t fm_shadow20_qcurrent(float amps)
{
    float q = amps * 250.0f; /* 4mA/count, +/-131A */
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int16_t)q;
}
static inline uint16_t fm_shadow20_u16(float x)
{
    if (!(x >= 0.0f)) return 0U;
    if (x >= 65535.0f) return 65535U;
    return (uint16_t)x;
}

static inline void fm_shadow20_update(float iu, float iv, float iw)
{
#if FM_V13_20K_SHADOW_TEST
    if (g.t.state != FM_STATE_RUN) return;
    /* RUN observer's first/predictor callback is when its voltage-pair
     * accumulator is empty. Avoid executing full V13 in observer callbacks. */
    if (s_run20k_session == g.state_start_isr_count &&
        s_run20k_accum_count != 0U) return;
    if (s_shadow20_session != g.state_start_isr_count) {
        s_shadow20_session = g.state_start_isr_count;
        V13_ResetSession(&s_v13_fast_state); /* no expf in interrupt */
        s_shadow20_updates = 0U;
        s_shadow20_cycles_max = 0U;
        s_shadow20_overruns = 0U;
        s_shadow20_gaps = 0U;
        s_shadow20_prev_tick = 0U;
        s_shadow20_disabled = 0U;
        /* RAM capture remains idle until explicitly armed. */
        g_v13_ram_capture.status = 0U;
        g_v13_ram_capture.count = 0U;
        g.t.v13_fast_mode = 1U;
    }
    if (s_shadow20_disabled) return;
    if (s_shadow20_prev_tick && (g.isr_count - s_shadow20_prev_tick) != 2U)
        ++s_shadow20_gaps;
    s_shadow20_prev_tick = g.isr_count;
    const float currents[3] = {iu, iv, iw};
    const uint16_t ccr[3] = {(uint16_t)TIM1->CCR1,
                              (uint16_t)TIM1->CCR2,
                              (uint16_t)TIM1->CCR3};
    const float duty[3] = {ccr[0] * g.v13_inv_arr,
                            ccr[1] * g.v13_inv_arr,
                            ccr[2] * g.v13_inv_arr};
    const uint32_t active_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
        TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    const uint8_t voltage_valid =
        ((TIM1->CCER & active_mask) == active_mask &&
         (TIM1->BDTR & TIM_BDTR_MOE)) ? 1U : 0U;
    float filtered[3];
    const uint32_t start = DWT->CYCCNT;
    V13_Update(&s_v13_fast_state, currents, duty, g.vbus,
               voltage_valid, filtered);
    const uint32_t cycles = DWT->CYCCNT - start;
    s_shadow20_cycles_last = cycles;
    if (cycles > s_shadow20_cycles_max) s_shadow20_cycles_max = cycles;
    ++s_shadow20_updates;
    s_shadow20_this_isr = 1U;

    if (s_v13_capture_rearm) {
        s_v13_capture_rearm = 0U;
        g_v13_ram_capture.status = 1U;
        g_v13_ram_capture.count = 0U;
        ++g_v13_ram_capture.session;
        g_v13_ram_capture.discontinuities = s_shadow20_gaps;
    }
    if (g_v13_ram_capture.status == 1U) {
        uint32_t ix = g_v13_ram_capture.count;
        if (ix < V13_CAPTURE_LENGTH) {
            V13_CaptureFrame *c = &g_v13_ram_capture.frames[ix];
            c->isr_tick = g.isr_count;
            for (unsigned j=0; j<3U; ++j) {
                c->raw[j] = fm_shadow20_qcurrent(currents[j]);
                c->frozen[j] = fm_shadow20_qcurrent(filtered[j]);
                c->duty[j] = fm_shadow20_u16(duty[j]*65535.0f);
            }
            c->bus_mv = fm_shadow20_u16(g.vbus * 1000.0f);
            c->alpha = fm_shadow20_u16(s_v13_fast_state.diag.alpha * 65535.0f);
            c->voltage_event = fm_shadow20_u16(s_v13_fast_state.diag.voltage_evidence * 65535.0f);
            c->cpu_cycles = fm_shadow20_u16((float)cycles);
            c->flags = (uint16_t)((voltage_valid ? 1U : 0U) |
                     (!s_v13_fast_state.diag.input_valid ? 2U : 0U) |
                     (g.t.observer20k_mode == 1U ? 4U : 0U));
            g_v13_ram_capture.count = ix + 1U;
            if (ix + 1U == V13_CAPTURE_LENGTH)
                g_v13_ram_capture.status = 2U;
        }
    }
    /* One slow telemetry update per 40 filter updates (500Hz), never UART
     * serialization from the control ISR. Host DMA stream is only 20Hz. */
    if ((s_shadow20_updates % 40U) == 0U) {
        g.t.v13_fast_mode = 1U;
        g.t.v13_shadow_decimation = 2U;
        g.t.v13_samples = s_shadow20_updates;
        g.t.v13_cycles_last = s_shadow20_cycles_last;
        g.t.v13_cycles_max = s_shadow20_cycles_max;
        g.t.v13_fast_overruns = s_shadow20_overruns;
        g.t.v13_gap_resyncs = s_shadow20_gaps;
        g.t.v13_alpha = s_v13_fast_state.diag.alpha;
        g.t.v13_base_alpha = s_v13_fast_state.diag.base_alpha;
        g.t.v13_voltage_evidence = s_v13_fast_state.diag.voltage_evidence;
        g.t.v13_rejected_samples = s_v13_fast_state.rejected_samples;
        g.t.v13_capture_state = g_v13_ram_capture.status;
        g.t.v13_capture_count = g_v13_ram_capture.count;
        g.t.iu_a = iu; g.t.iv_a = iv; g.t.iw_a = iw;
        g.t.v13_iu_a = filtered[0];
        g.t.v13_iv_a = filtered[1];
        g.t.v13_iw_a = filtered[2];
        g.t.v13_observer_blend = 0.0f;
    }
#else
    (void)iu; (void)iv; (void)iw;
#endif
}

static uint32_t s_isr_budget_cycles = 4250U; /* refreshed from SystemCoreClock at init */


/* Keep execution-time measurement, but remove the old per-ISR entry-interval
 * bookkeeping.  That diagnostic did several extra loads/branches on every
 * 25-us callback and was not used by control.  The interrupt entry timestamp
 * is already captured by ADC1_2_IRQHandler, so one CYCCNT read at exit is
 * enough to prove whether the control callback missed its own budget. */
static inline void fm_isr_timing_begin(uint32_t entry_cycles)
{
    (void)entry_cycles;
}

static inline void fm_isr_timing_end(uint32_t entry_cycles)
{
    uint32_t elapsed = DWT->CYCCNT - entry_cycles;
    g.isr_cycles_last = elapsed;
    if (elapsed > g.isr_cycles_max)
        g.isr_cycles_max = elapsed;
    if (elapsed > s_isr_budget_cycles)
        ++g.isr_overrun_count;
#if FM_V13_40K_CACHED_OUTPUT
    /* Self-disengage *experimental* extra ISR work on 2 consecutive
     * deadline overruns. Does not affect observer, current PI or protections. */
    if (g.t.state == FM_STATE_RUN && !s_v13_40k_guard_off) {
        if (elapsed > s_isr_budget_cycles) {
            if (++s_v13_40k_overbudget_consecutive >= 2U) {
                s_v13_40k_guard_off = 1U;
                ++s_v13_40k_guard_events;
                g.t.v13_40k_guard_events = s_v13_40k_guard_events;
            }
        } else {
            s_v13_40k_overbudget_consecutive = 0U;
        }
    }
#endif
#if FM_V13_FAST_20KHZ_ENABLE
    if (g.t.state == FM_STATE_RUN && !s_v13_fast_disabled) {
        if (elapsed > s_isr_budget_cycles) {
            ++s_v13_fast_overruns;
            if (s_v13_fast_overruns >= FM_V13_FAST_MAX_OVERRUNS) {
                /* Preserve the working high-speed FOC path. This is a
                 * performance fallback, not an electrical fault bypass. */
                s_v13_fast_disabled = 1U;
                g.t.v13_fast_mode = 2U;
                g.t.v13_shadow_decimation = FM_V13_SHADOW_DECIMATION;
            }
        }
        g.t.v13_fast_overruns = s_v13_fast_overruns;
    }
    s_v13_fast_this_isr = 0U;
#endif
#if FM_V13_20K_SHADOW_TEST
    /* Count only V13-bearing callback overruns, not acquisition or observer
     * callback overruns. Never interfere with current protection or observer. */
    if (s_shadow20_this_isr && elapsed > s_isr_budget_cycles) {
        ++s_shadow20_overruns;
        if (s_shadow20_overruns >= FM_V13_20K_MAX_OVERRUNS) {
            s_shadow20_disabled = 1U;
            g.t.v13_fast_mode = 2U; /* guard tripped; RAW control untouched */
            g.t.v13_fast_overruns = s_shadow20_overruns;
        }
    }
    s_shadow20_this_isr = 0U;
#endif
    s_v13_isr_cycles_total += elapsed; /* ~one addition per ISR, never waits */
}

static float one_pole_alpha_dt(float fc, float dt_s)
{
    float x = FM_TWO_PI * fc * dt_s;
    return x / (1.0f + x);
}

static float one_pole_alpha(float fc)
{
    return one_pole_alpha_dt(fc, FM_CONTROL_TS_S);
}

static float fast_atan_0_to_1(float x)
{
    return x / (1.0f + 0.28086f * x * x);
}

static float rpm_from_omega(float omega_e)
{
    return omega_e / FM_POLE_PAIRS * (60.0f / FM_TWO_PI);
}

static float omega_from_rpm(float rpm)
{
    return rpm * FM_TWO_PI / 60.0f * FM_POLE_PAIRS;
}

static float blend_angle(float a, float b, float k)
{
    k = FM_Clamp(k, 0.0f, 1.0f);
    return FM_WrapPi(a + k * FM_WrapPi(b - a));
}

static float hfi_foc_theta(void)
{
    return FM_WrapPi(g.hfi_angle_sign * g.hfi.theta + g.hfi_to_foc_offset);
}

/* The same fixed sign used to map HFI angle into the FOC frame must also be
 * applied to HFI angular velocity.  Otherwise a commissioned -1 angle sign
 * would make the speed loop use the opposite direction. */
static float hfi_foc_omega_e(void)
{
    return g.hfi_angle_sign * g.hfi.omega_filt;
}

static float hfi_foc_rpm(void)
{
    return rpm_from_omega(hfi_foc_omega_e());
}

/* Do not treat a large transient voltage-model residual as proof of real
 * mechanical motion.  During the previous launch the observer briefly claimed
 * ~1.8 V BEMF and ~525 rpm while HFI was only around 73 rpm and the angles were
 * ~1.4 rad apart.  Relax HFI omega damping only when independent observer
 * evidence is mutually coherent with the HFI track. */
static uint8_t observer_motion_coherent(void)
{
    float obs_rpm = rpm_from_omega(g.obs.omega_e);
    float hfi_rpm = hfi_foc_rpm();
    float angle_err = fabsf(FM_WrapPi(g.obs.theta - hfi_foc_theta()));

    if (fabsf(hfi_rpm) < FM_OBSERVER_PLL_ENABLE_MIN_HFI_RPM)
        return 0U;
    if (g.obs.bemf_mag < FM_HFI_MOTION_BEMF_V)
        return 0U;
    if (fabsf(obs_rpm) < FM_HFI_MOTION_MIN_OBS_RPM)
        return 0U;
    if (angle_err > FM_HFI_MOTION_MAX_ANGLE_ERR_RAD)
        return 0U;
    if (fabsf(obs_rpm - hfi_rpm) > FM_HFI_MOTION_MAX_RPM_DIFF)
        return 0U;

    /* If HFI speed is already meaningful, also require the same direction. */
    if (fabsf(hfi_rpm) > 30.0f && (obs_rpm * hfi_rpm) < 0.0f)
        return 0U;

    return 1U;
}

/* PF0 / ADC1_IN10 is the real DC-bus divider input on this board.  The
 * original project used a nominal 20k/1k divider (21:1), so this fast-only
 * controller restores the same measurement instead of reporting a fixed
 * 13 V constant.  ADC1 is independent from ADC2 injected current sampling. */
static uint8_t read_vbus_once(float *voltage_v, uint16_t *counts_out)
{
    uint16_t counts = 0U;
    uint8_t ok = 0U;

    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, FM_VBUS_ADC_TIMEOUT_MS) == HAL_OK) {
            counts = (uint16_t)(HAL_ADC_GetValue(&hadc1) & 0x0FFFU);
            float v = (float)counts * FM_VBUS_VOLTS_PER_COUNT;
            if (isfinite(v) && v >= FM_VBUS_VALID_MIN_V && v <= FM_VBUS_VALID_MAX_V) {
                if (voltage_v != NULL) *voltage_v = v;
                ok = 1U;
            }
        }
        (void)HAL_ADC_Stop(&hadc1);
    }

    if (counts_out != NULL) *counts_out = counts;
    return ok;
}

static void update_vbus_measurement(void)
{
    float v = 0.0f;
    uint16_t counts = 0U;
    uint8_t ok = read_vbus_once(&v, &counts);

    g.t.vbus_adc_counts = counts;
    g.t.vbus_adc_valid = ok;

    if (ok) {
        if (!g.vbus_has_measurement) {
            g.vbus = v;
            g.vbus_has_measurement = 1U;
        } else {
            g.vbus += FM_VBUS_FILTER_ALPHA * (v - g.vbus);
        }

        if (g.vbus < FM_VBUS_UV_TRIP_V) {
            if (g.vbus_uv_count < 255U) ++g.vbus_uv_count;
        } else {
            g.vbus_uv_count = 0U;
        }

        /* Existing bridge MOSFETs are only 60-V parts.  This is deliberately
         * conservative and prevents a software command from normalizing a
         * dangerously high bus as a valid operating point.  The threshold is
         * a hardware-profile value, not the future 60-V ESC target. */
        if (g.vbus > FM_VBUS_OV_TRIP_V) {
            if (g.vbus_ov_count < 255U) ++g.vbus_ov_count;
        } else {
            g.vbus_ov_count = 0U;
        }
    } else {
        /* Keep the last valid bus value (or the nominal boot fallback). */
        g.vbus_uv_count = 0U;
        g.vbus_ov_count = 0U;
    }

    g.t.vbus_v = g.vbus;
}

static void set_state(FM_State s)
{
    g.t.state = s;
    g.state_start_ms = HAL_GetTick();          /* foreground/debug timestamp */
    g.state_start_isr_count = g.isr_count;     /* hard-real-time state timing */
}

static FM_AlphaBeta clarke(float iu, float iv)
{
    FM_AlphaBeta x;
    x.alpha = iu;
    x.beta = (iu + 2.0f * iv) * FM_INV_SQRT3;
    return x;
}

static inline FM_DQ park_sc(FM_AlphaBeta x, float s, float c)
{
    FM_DQ y;
    y.d =  c * x.alpha + s * x.beta;
    y.q = -s * x.alpha + c * x.beta;
    return y;
}

static inline FM_AlphaBeta invpark_sc(FM_DQ x, float s, float c)
{
    FM_AlphaBeta y;
    y.alpha = c * x.d - s * x.q;
    y.beta  = s * x.d + c * x.q;
    return y;
}

static FM_DQ park(FM_AlphaBeta x, float theta)
{
    float s, c;
    FM_CordicSinCos(theta, &s, &c);
    return park_sc(x, s, c);
}

static FM_AlphaBeta invpark(FM_DQ x, float theta)
{
    float s, c;
    FM_CordicSinCos(theta, &s, &c);
    return invpark_sc(x, s, c);
}

static FM_AlphaBeta add_ab(FM_AlphaBeta a, FM_AlphaBeta b)
{
    FM_AlphaBeta r = {a.alpha + b.alpha, a.beta + b.beta};
    return r;
}

static FM_AlphaBeta sub_ab(FM_AlphaBeta a, FM_AlphaBeta b)
{
    FM_AlphaBeta r = {a.alpha - b.alpha, a.beta - b.beta};
    return r;
}

/* The 4 kHz notch output g.i_lf contains the fundamental motor current while
 * strongly rejecting the HFI carrier.  Subtracting it from the raw current
 * therefore gives the carrier-dominated residual that the HFI demodulator
 * actually needs.  This prevents torque-producing DC/low-frequency Iq from
 * biasing the angle PLL as soon as the motor is released from alignment. */
static FM_AlphaBeta hfi_carrier_current(FM_AlphaBeta i_raw)
{
    return sub_ab(i_raw, g.i_lf);
}

static float hfi_scan_angle(uint8_t index)
{
    /* Saliency is an axis, so scan one electrical pi-period rather than 2*pi. */
    return (float)index * (float)M_PI / (float)FM_HFI_SCAN_ANGLE_COUNT;
}

static FM_AlphaBeta limit_ab(FM_AlphaBeta v, float vmax)
{
    float m2 = v.alpha * v.alpha + v.beta * v.beta;
    float vmax2 = vmax * vmax;
    if (m2 > vmax2) {
        float k = vmax / sqrtf(m2);
        v.alpha *= k;
        v.beta *= k;
    }
    return v;
}

static FM_HOT_ISR void pwm_apply_ab(FM_AlphaBeta v)
{
    float va = v.alpha;
    float vb = -0.5f * v.alpha + FM_SQRT3_BY_2 * v.beta;
    float vc = -0.5f * v.alpha - FM_SQRT3_BY_2 * v.beta;

    float vmax = (va > vb) ? va : vb;
    if (vc > vmax) vmax = vc;
    float vmin = (va < vb) ? va : vb;
    if (vc < vmin) vmin = vc;
    float common = -0.5f * (vmax + vmin);

    float inv_vbus = 1.0f / g.vbus;
    float da = 0.5f + (va + common) * inv_vbus;
    float db = 0.5f + (vb + common) * inv_vbus;
    float dc = 0.5f + (vc + common) * inv_vbus;

    da = FM_Clamp(da, FM_MIN_DUTY, FM_MAX_DUTY);
    db = FM_Clamp(db, FM_MIN_DUTY, FM_MAX_DUTY);
    dc = FM_Clamp(dc, FM_MIN_DUTY, FM_MAX_DUTY);
    FM_PWM_Set(da, db, dc);
}

/* Six-step sector centers expressed as the equivalent rotor-reference angle.
 * The actual energized current vectors are +90 electrical degrees from these
 * centers.  The sequence increases by +60 degrees per commutation. */
static float trap_rotor_ref_for_sector(uint8_t sector)
{
    static const float ref[6] = {
        -2.09439510239f, -1.04719755120f, 0.0f,
         1.04719755120f,  2.09439510239f, 3.14159265359f
    };
    return ref[sector % 6U];
}

/* Continuous rotor-reference implied by the six-step sector and fractional
 * progress through that 60-degree sector.  The old handover used the SECTOR
 * CENTER even when trap_sector_phase was near zero, which can put forced FOC
 * roughly +30 electrical degrees ahead immediately. */
static float trap_continuous_rotor_ref(uint8_t sector, float sector_phase)
{
    sector_phase = FM_Clamp(sector_phase, 0.0f, 1.0f);
    return FM_WrapPi(trap_rotor_ref_for_sector(sector) +
                     (sector_phase - 0.5f) * ((float)M_PI / 3.0f));
}

/* Average alpha/beta voltage vector of the active six-step pair.  Common-mode
 * voltage is irrelevant to Clarke alpha/beta, so the floating phase is placed
 * at the source/sink midpoint.  line_v is the source-to-sink average voltage.
 * This lets the observer run during the already-proven six-step stage. */
static FM_AlphaBeta trap_sixstep_voltage_ab(uint8_t sector, float line_v)
{
    const float h = 0.5f * line_v;
    const float q = 0.28867513459f * line_v;
    const float r = 0.57735026919f * line_v;
    FM_AlphaBeta v = {0.0f, 0.0f};
    switch (sector % 6U) {
    case 0U: v.alpha =  h; v.beta = -q; break;
    case 1U: v.alpha =  h; v.beta =  q; break;
    case 2U: v.alpha = 0.0f; v.beta =  r; break;
    case 3U: v.alpha = -h; v.beta =  q; break;
    case 4U: v.alpha = -h; v.beta = -q; break;
    default: v.alpha = 0.0f; v.beta = -r; break;
    }
    return v;
}

static float trap_rotor_ref_for_sector_dir(uint8_t sector, float dir)
{
    float theta = trap_rotor_ref_for_sector(sector);
    /* Each hardware six-step current vector is +90 electrical degrees from the
     * positive-torque rotor reference above.  Negative torque therefore needs
     * the rotor reference shifted by pi (same vector becomes -90 deg from flux). */
    if (dir < 0.0f)
        theta += (float)M_PI;
    return FM_WrapPi(theta);
}

static uint8_t trap_sector_from_rotor_theta(float rotor_theta, float dir)
{
    uint8_t best = 0U;
    float best_err = fabsf(FM_WrapPi(rotor_theta -
                                     trap_rotor_ref_for_sector_dir(0U, dir)));
    for (uint8_t k = 1U; k < 6U; ++k) {
        float e = fabsf(FM_WrapPi(rotor_theta -
                                  trap_rotor_ref_for_sector_dir(k, dir)));
        if (e < best_err) {
            best_err = e;
            best = k;
        }
    }
    return best;
}

static float trap_phase_from_rotor_theta_dir(uint8_t sector, float rotor_theta,
                                             float dir)
{
    float local = FM_WrapPi(rotor_theta -
                            trap_rotor_ref_for_sector_dir(sector, dir));
    return FM_Clamp(0.5f + local / ((float)M_PI / 3.0f), 0.0f, 1.0f);
}

/* Continuous rotor reference for the bootstrap sector clock.  This is only a
 * startup trajectory before the PM-flux/BEMF observer has proved real motion. */
static float trap_bootstrap_rotor_ref(uint8_t sector, float sector_phase, float dir)
{
    sector_phase = FM_Clamp(sector_phase, 0.0f, 1.0f);
    return FM_WrapPi(trap_rotor_ref_for_sector_dir(sector, dir) +
                     (sector_phase - 0.5f) * ((float)M_PI / 3.0f));
}

/* Advance the six-step torque vector modestly at speed.  Rotor angle still owns
 * the commutation; this only changes where the legal 60-degree boundary is
 * crossed to compensate current/inductance lag. */
static float trap_torque_advance_for_rpm(float rpm)
{
    float mag = fabsf(rpm);
    float den = FM_TRAP_FOC_HANDOVER_RPM - FM_TRAP_BEMF_SYNC_ENABLE_RPM;
    float k = (den > 1.0f) ?
              (mag - FM_TRAP_BEMF_SYNC_ENABLE_RPM) / den : 1.0f;
    k = FM_Clamp(k, 0.0f, 1.0f);
    return FM_TRAP_TORQUE_ADVANCE_LOW_RAD +
           k * (FM_TRAP_TORQUE_ADVANCE_HIGH_RAD -
                FM_TRAP_TORQUE_ADVANCE_LOW_RAD);
}

static uint8_t trap_next_sector(uint8_t sector, float dir)
{
    return (dir < 0.0f) ? (uint8_t)((sector + 5U) % 6U)
                        : (uint8_t)((sector + 1U) % 6U);
}

static uint8_t trap_prev_sector(uint8_t sector, float dir)
{
    return (dir < 0.0f) ? (uint8_t)((sector + 1U) % 6U)
                        : (uint8_t)((sector + 5U) % 6U);
}

static void trap_zc_reset_for_sector(void);

/* Rotor-event commutator.  There is no synthetic electrical-angle accumulator.
 * The actual PM-flux/BEMF angle chooses the next legal six-step sector.  Two
 * consecutive observer updates are required before an adjacent transition so
 * raw flux noise cannot chatter the bridge. */
static uint8_t trap_commutate_from_rotor_theta(float rotor_theta, float dir)
{
    uint8_t desired = trap_sector_from_rotor_theta(rotor_theta, dir);
    uint8_t next = trap_next_sector(g.trap_sector, dir);
    uint8_t prev = trap_prev_sector(g.trap_sector, dir);

    if (desired == g.trap_sector) {
        g.trap_sector_candidate = desired;
        g.trap_sector_candidate_count = 0U;
        return 1U;
    }

    if (desired == next) {
        if (g.trap_sector_candidate != desired) {
            g.trap_sector_candidate = desired;
            g.trap_sector_candidate_count = 1U;
        } else if (g.trap_sector_candidate_count < 255U) {
            ++g.trap_sector_candidate_count;
        }

        if (g.trap_sector_candidate_count >= FM_TRAP_SECTOR_CONFIRM_UPDATES) {
            uint32_t elapsed = g.trap_sector_ticks;
            g.trap_sector = desired;
            g.trap_sector_candidate_count = 0U;
            ++g.trap_commutation_count;
            if (elapsed > 0U)
                g.trap_last_sector_ticks = elapsed;
            trap_zc_reset_for_sector();
        }
        return 1U;
    }

    /* A one-sector move in the opposite direction is physical evidence that the
     * rotor is not following the requested torque direction.  Do not fake a
     * forward sector to hide it.  Larger jumps are rejected as estimator glitches. */
    if (desired == prev)
        return 0U;

    return 0U;
}

static uint8_t trap_floating_phase_index(uint8_t sector)
{
    /* Hardware table: W floats in sectors 0/3, V in 1/4, U in 2/5. */
    switch (sector % 6U) {
    case 0U:
    case 3U: return 2U; /* W */
    case 1U:
    case 4U: return 1U; /* V */
    default: return 0U; /* U */
    }
}

static float trap_update_floating_phase_model(float iu, float iv, float iw,
                                               FM_AlphaBeta v_applied,
                                               uint8_t sector)
{
    /* Approximate the three phase-to-neutral voltages from the average applied
     * alpha/beta six-step voltage, then estimate phase BEMF directly:
     *
     *   e_u ~= v_u - R*i_u - L*di_u/dt
     *   e_v ~= v_v - R*i_v - L*di_v/dt
     *   e_w ~= v_w - R*i_w - L*di_w/dt
     *
     * The current values are the THREE actual injected ADC current channels,
     * including the phase that is physically Hi-Z.  All three estimates remain
     * warm across commutations, but only the sector's floating phase is allowed
     * to drive the zero-cross detector. */
    float vu = v_applied.alpha;
    float vv = -0.5f * v_applied.alpha + FM_SQRT3_BY_2 * v_applied.beta;
    float vw = -0.5f * v_applied.alpha - FM_SQRT3_BY_2 * v_applied.beta;

    if (!g.trap_phase_model_valid) {
        g.trap_i_u_prev = iu;
        g.trap_i_v_prev = iv;
        g.trap_i_w_prev = iw;
        g.trap_di_u = 0.0f;
        g.trap_di_v = 0.0f;
        g.trap_di_w = 0.0f;
        g.trap_phase_bemf_u_v = 0.0f;
        g.trap_phase_bemf_v_v = 0.0f;
        g.trap_phase_bemf_w_v = 0.0f;
        g.trap_phase_model_valid = 1U;
    }

    float diu_raw = (iu - g.trap_i_u_prev) / FM_CONTROL_TS_S;
    float div_raw = (iv - g.trap_i_v_prev) / FM_CONTROL_TS_S;
    float diw_raw = (iw - g.trap_i_w_prev) / FM_CONTROL_TS_S;
    g.trap_i_u_prev = iu;
    g.trap_i_v_prev = iv;
    g.trap_i_w_prev = iw;
    g.trap_di_u += FM_TRAP_ZC_DIDT_ALPHA * (diu_raw - g.trap_di_u);
    g.trap_di_v += FM_TRAP_ZC_DIDT_ALPHA * (div_raw - g.trap_di_v);
    g.trap_di_w += FM_TRAP_ZC_DIDT_ALPHA * (diw_raw - g.trap_di_w);

    float eu = vu - FM_PHASE_R_OHM * iu - FM_PHASE_L_H * g.trap_di_u;
    float ev = vv - FM_PHASE_R_OHM * iv - FM_PHASE_L_H * g.trap_di_v;
    float ew = vw - FM_PHASE_R_OHM * iw - FM_PHASE_L_H * g.trap_di_w;
    g.trap_phase_bemf_u_v += FM_TRAP_ZC_BEMF_ALPHA *
                             (eu - g.trap_phase_bemf_u_v);
    g.trap_phase_bemf_v_v += FM_TRAP_ZC_BEMF_ALPHA *
                             (ev - g.trap_phase_bemf_v_v);
    g.trap_phase_bemf_w_v += FM_TRAP_ZC_BEMF_ALPHA *
                             (ew - g.trap_phase_bemf_w_v);

    uint8_t fp = trap_floating_phase_index(sector);
    if (fp == 0U) {
        g.trap_floating_bemf_v = g.trap_phase_bemf_u_v;
        g.trap_floating_current_a = iu;
    } else if (fp == 1U) {
        g.trap_floating_bemf_v = g.trap_phase_bemf_v_v;
        g.trap_floating_current_a = iv;
    } else {
        g.trap_floating_bemf_v = g.trap_phase_bemf_w_v;
        g.trap_floating_current_a = iw;
    }
    return g.trap_floating_bemf_v;
}

static float trap_expected_zc_edge_sign(uint8_t sector, float dir)
{
    /* For + rotation sectors 0/2/4 cross +->- and 1/3/5 cross -->+.
     * Reverse rotation flips the time direction and therefore the edge. */
    float parity = (sector & 1U) ? 1.0f : -1.0f;
    return parity * ((dir < 0.0f) ? -1.0f : 1.0f);
}

static float trap_rpm_from_sector_ticks(uint32_t sector_ticks, float dir)
{
    if (sector_ticks == 0U) return 0.0f;
    /* One sector is 1/6 electrical revolution. */
    float rpm = (10.0f * FM_PWM_HZ) /
                (FM_POLE_PAIRS * (float)sector_ticks);
    return ((dir < 0.0f) ? -rpm : rpm);
}

static uint8_t trap_direct_bemf_rotor_theta(float dir, float *theta_out,
                                             float *mag_out)
{
    float e2 = g.obs.e_alpha * g.obs.e_alpha + g.obs.e_beta * g.obs.e_beta;
    if (!isfinite(e2) || e2 < FM_TRAP_ZC_MIN_BEMF_VECTOR_V *
                              FM_TRAP_ZC_MIN_BEMF_VECTOR_V)
        return 0U;

    float theta_e = FM_CordicAtan2(g.obs.e_beta, g.obs.e_alpha);
    /* PMSM BEMF: e = omega * J * psi.  Therefore e leads rotor flux by
     * +90 deg for positive rotation and lags by 90 deg for negative rotation. */
    float theta = FM_WrapPi(theta_e - ((dir < 0.0f) ? -1.0f : 1.0f) *
                                      ((float)M_PI * 0.5f));
    if (theta_out != NULL) *theta_out = theta;
    if (mag_out != NULL) *mag_out = sqrtf(e2);
    return 1U;
}

static void trap_zc_reset_for_sector(void)
{
    g.trap_zc_armed = 0U;
    g.trap_zc_pending = 0U;
    g.trap_zc_prev_valid = 0U;
    g.trap_zc_prev_efloat_v = 0.0f;
    g.trap_zc_countdown_ticks = 0U;
    g.trap_zc_due_sector_ticks = 0U;
    g.trap_sector_ticks = 0U;
}

static uint8_t trap_flux_measurement_available(float *rpm_out)
{
    float rpm = rpm_from_omega(g.obs.flux_omega_e);
    uint8_t ok = (g.trap_cal_observer_started && g.obs.theta_meas_valid &&
                  g.obs.flux_speed_sample_count >= FM_TRAP_FLUX_SPEED_MIN_SAMPLES &&
                  isfinite(rpm));
    if (rpm_out != NULL) *rpm_out = rpm;
    return ok;
}

static uint8_t trap_flux_measurement_valid(float *rpm_out)
{
    const float min2 = FM_TRAP_BEMF_SYNC_MIN_V * FM_TRAP_BEMF_SYNC_MIN_V;
    float e2 = g.obs.e_alpha * g.obs.e_alpha + g.obs.e_beta * g.obs.e_beta;
    float rpm = 0.0f;
    uint8_t available = trap_flux_measurement_available(&rpm);
    uint8_t ok = (available && isfinite(e2) && e2 >= min2 &&
                  fabsf(rpm) >= FM_TRAP_BEMF_SYNC_ENABLE_RPM);
    if (rpm_out != NULL) *rpm_out = rpm;
    return ok;
}

static float trap_base_line_voltage(float rpm)
{
    float den = FM_TRAP_TARGET_RPM - FM_TRAP_START_RPM;
    float k = (den > 1.0f) ? (rpm - FM_TRAP_START_RPM) / den : 1.0f;
    k = FM_Clamp(k, 0.0f, 1.0f);
    return FM_TRAP_START_LINE_V + k * (FM_TRAP_TARGET_LINE_V - FM_TRAP_START_LINE_V);
}

static float trap_soft_current_limit(float rpm)
{
    float mag = fabsf(rpm);
    float den = FM_TRAP_TARGET_RPM - FM_TRAP_BEMF_SYNC_ENABLE_RPM;
    float k = (den > 1.0f) ? (mag - FM_TRAP_BEMF_SYNC_ENABLE_RPM) / den : 1.0f;
    k = FM_Clamp(k, 0.0f, 1.0f);
    return FM_TRAP_LAUNCH_CURRENT_A +
           k * (FM_TRAP_HANDOVER_CURRENT_A - FM_TRAP_LAUNCH_CURRENT_A);
}


static float hybrid_sign(float x)
{
    return (x < 0.0f) ? -1.0f : 1.0f;
}

/* Raw PM-flux angle placed on the same pi branch selected at the six-step ->
 * FOC transition.  The calibration offset itself is never modified by pi, so
 * it remains a reusable hardware sector->flux calibration for later low-speed
 * six-step fallback and direction changes. */
static float hybrid_aligned_raw_theta(void)
{
    float theta = g.obs.theta_meas_raw;
    if (g.handover_polarity_pi)
        theta = FM_WrapPi(theta + (float)M_PI);
#if FM_RUN_RAW_OBSERVER_20KHZ
    if (g.t.state == FM_STATE_RUN && s_run20k_accum_count == 1U &&
        s_run20k_session == g.state_start_isr_count &&
        g.obs.theta_meas_valid && isfinite(g.obs.flux_omega_e)) {
        /* On the skipped update, the last flux angle is 25us old. */
        theta = FM_WrapPi(theta + g.obs.flux_omega_e * FM_CONTROL_TS_S);
    }
#endif

#if FM_V13_FAST_20KHZ_ENABLE && FM_V13_STAGGER_OBSERVER_20KHZ
    /* Health check must compare angles at the SAME instant. The observer is
     * sampled one tick earlier on a filter-only (even) ISR. Do not mutate
     * flux_theta_prev: it is needed for 50us dtheta on next observer update. */
    if (g.t.state == FM_STATE_RUN && !s_v13_fast_disabled &&
        s_v13_fast_this_isr && s_v13_fast_pair_valid &&
        isfinite(g.obs.flux_omega_e))
        theta = FM_WrapPi(theta + g.obs.flux_omega_e * FM_CONTROL_TS_S);
#endif
    return theta;
}

/* Continuous control-angle PLL.  During only the short six-step->FOC
 * acquisition window it is anchored near the 1500-rpm six-step trajectory.
 * Once phase lock is declared, the anchor is removed: PLL omega becomes the
 * real sensorless speed estimate and target RPM is no longer an angle source.
 * Explicit finite guards keep bad observer math away from CORDIC conversion. */
static FM_HOT_ISR void hybrid_control_pll_step(float authority, float min_bemf_v, float dt_s,
                                    float *phase_err_out, uint8_t *valid_out)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f) dt_s = FM_CONTROL_TS_S;
    if (!isfinite(authority)) authority = 0.0f;
    authority = FM_Clamp(authority, 0.0f, 1.0f);

    if (!isfinite(g.control_theta))
        g.control_theta = isfinite(g.forced_theta) ? FM_WrapPi(g.forced_theta) : 0.0f;

    /* There are deliberately TWO PLL regimes:
     *
     *  1) OBSERVER_HANDOVER: a short I-f acquisition bridge around the proven
     *     six-step crossover speed.  The observer may trim that speed only by
     *     a small bounded amount while theta_control is phase-locked.
     *
     *  2) BEMF_BOOST / RUN: no virtual/base speed exists anymore.  The PLL's
     *     own omega state is the measured/control speed and is corrected only
     *     by the raw PM-flux phase detector.  The 10,000-rpm request is handled
     *     later by the speed PI; it NEVER advances electrical angle directly.
     */
    const uint8_t acquisition = (g.t.state == FM_STATE_OBSERVER_HANDOVER);

    float omega_e = g.forced_omega_e;
    if (!isfinite(omega_e)) {
        float seed_rpm = isfinite(g.trap_command_rpm) ? g.trap_command_rpm :
                         hybrid_sign(g.t.target_rpm) * FM_HYBRID_FOC_ENTER_RPM;
        omega_e = omega_from_rpm(seed_rpm);
    }

    float trim_e = 0.0f;
    float base_e = 0.0f;
    float trim_limit_e = 0.0f;

    if (acquisition) {
        float base_rpm = g.trap_command_rpm;
        if (!isfinite(base_rpm) || fabsf(base_rpm) < FM_HYBRID_FOC_EXIT_RPM)
            base_rpm = hybrid_sign(g.t.target_rpm) * FM_HYBRID_FOC_ENTER_RPM;

        base_e = omega_from_rpm(base_rpm);
        trim_e = g.handover_observer_blend_start;
        if (!isfinite(trim_e)) trim_e = 0.0f;

        float trim_limit_rpm = FM_HYBRID_PLL_MAX_TRIM_FRACTION * fabsf(base_rpm);
        if (trim_limit_rpm < FM_HYBRID_PLL_MIN_TRIM_RPM)
            trim_limit_rpm = FM_HYBRID_PLL_MIN_TRIM_RPM;
        trim_limit_e = omega_from_rpm(trim_limit_rpm);
        trim_e = FM_Clamp(trim_e, -trim_limit_e, trim_limit_e);
        omega_e = base_e + trim_e;
    } else {
        /* Once acquisition is complete this old field is no longer a speed
         * source.  Clear it so telemetry/debugging cannot mistake a stale trim
         * for active virtual-speed authority. */
        g.handover_observer_blend_start = 0.0f;
    }

    omega_e = FM_Clamp(omega_e,
                       -omega_from_rpm(FM_HYBRID_PLL_MAX_MECH_RPM),
                        omega_from_rpm(FM_HYBRID_PLL_MAX_MECH_RPM));

    float theta_pred = FM_WrapPi(g.control_theta + omega_e * dt_s);
    g.control_theta = theta_pred;
    float correction_dt = dt_s;
#if FM_RUN_RAW_OBSERVER_20KHZ
    /* The flux/PLL innovation arrives once per 50us, but prediction is 25us.
     * Integrate Ki/Kp corrections over the elapsed innovation interval. */
    if (g.t.state == FM_STATE_RUN && s_run20k_correction_due)
        correction_dt = 2.0f * FM_CONTROL_TS_S;
#endif

#if FM_V13_FAST_20KHZ_ENABLE && FM_V13_STAGGER_OBSERVER_20KHZ
    if (g.t.state == FM_STATE_RUN && !s_v13_fast_disabled &&
        s_v13_fast_pair_valid && !s_v13_fast_this_isr)
        correction_dt = 2.0f * FM_CONTROL_TS_S; /* 20kHz correction, 40kHz prediction */
#endif

    float err = 0.0f;
    uint8_t valid = 0U;
    float e2 = g.obs.e_alpha * g.obs.e_alpha +
               g.obs.e_beta  * g.obs.e_beta;
    float emin2 = min_bemf_v * min_bemf_v;

    if (g.obs.theta_meas_valid && isfinite(e2) && e2 >= emin2) {
        float theta_meas = hybrid_aligned_raw_theta();
        if (isfinite(theta_meas))
            err = FM_WrapPi(theta_meas - theta_pred);

        if (isfinite(err) && fabsf(err) <= FM_HYBRID_PLL_MAX_TRACK_ERR_RAD) {
            float epll = FM_Clamp(err,
                                  -FM_HYBRID_PLL_PHASE_CLAMP_RAD,
                                   FM_HYBRID_PLL_PHASE_CLAMP_RAD);

            float dw = authority * FM_HYBRID_PLL_KI_RAD_S2_PER_RAD *
                       epll * correction_dt;
            float max_dw = omega_from_rpm(FM_HYBRID_PLL_MAX_ACCEL_RPM_S) *
                           correction_dt;
            if (isfinite(dw)) {
                dw = FM_Clamp(dw, -max_dw, max_dw);

                if (acquisition) {
                    trim_e = FM_Clamp(trim_e + dw,
                                      -trim_limit_e, trim_limit_e);
                    omega_e = base_e + trim_e;
                    g.handover_observer_blend_start = trim_e;
                } else {
                    /* REAL observer-locked speed state.  There is no 1500->10k
                     * forced ramp here; physical acceleration must appear as
                     * phase motion before omega can increase. */
                    omega_e += dw;
                }

                omega_e = FM_Clamp(omega_e,
                                   -omega_from_rpm(FM_HYBRID_PLL_MAX_MECH_RPM),
                                    omega_from_rpm(FM_HYBRID_PLL_MAX_MECH_RPM));
            }

            float phase_rate = authority * FM_HYBRID_PLL_KP_RAD_S_PER_RAD * epll;
            if (isfinite(phase_rate))
                g.control_theta = FM_WrapPi(theta_pred +
                                            phase_rate * correction_dt);
            valid = 1U;
        }
    }

    /* During the short FOC-capture state, preserve the signed handover branch.
     * Raw flux with the opposite sign makes capture incoherent and triggers the
     * existing 5-ms fallback; it must not pull the control PLL through zero and
     * create the positive-RPM flip seen during failed negative catches. */
    if (g.t.state == FM_STATE_BEMF_BOOST) {
        float capture_dir = hybrid_sign(g.active_target_rpm);
        if (capture_dir < 0.0f && omega_e > 0.0f)
            omega_e = 0.0f;
        else if (capture_dir > 0.0f && omega_e < 0.0f)
            omega_e = 0.0f;
    }

    if (!isfinite(omega_e))
        omega_e = omega_from_rpm(hybrid_sign(g.t.target_rpm) *
                                 FM_HYBRID_FOC_ENTER_RPM);
    if (!isfinite(g.control_theta))
        g.control_theta = 0.0f;

    g.forced_omega_e = omega_e;
    g.obs.phase_err = err;
    g.obs.voltage_to_rotor_lag_rad = err;
    g.obs.theta = g.control_theta;
    g.obs.omega_e = g.forced_omega_e;

    if (phase_err_out) *phase_err_out = err;
    if (valid_out) *valid_out = valid;
}

/* Independent acquisition-speed check.  Keep this noinline so the extra
 * scratch locals do not enlarge the already-hot 40 kHz ADC ISR stack frame in
 * the Debug/-O0 build. */
__attribute__((noinline))
static uint8_t hybrid_flux_speed_coherent(float control_rpm)
{
    float flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
    if (!isfinite(control_rpm) || !isfinite(flux_rpm))
        return 0U;
    if (fabsf(flux_rpm) < FM_HYBRID_SYNC_MIN_FLUX_RPM)
        return 0U;
    if (flux_rpm * control_rpm <= 0.0f)
        return 0U;
    if (fabsf(flux_rpm - control_rpm) > FM_HYBRID_SYNC_MAX_FLUX_RPM_ERR)
        return 0U;
    return 1U;
}

/* Re-enter the proven six-step operating mode from an already-moving FOC
 * trajectory.  This is the low-speed/stall/reversal escape path: no HFI scan is
 * required because the continuous control angle gives the present rotor sector.
 * The calibrated sector->flux offset is retained for the next FOC entry. */
static void hybrid_enter_sixstep_from_control(void)
{
    /* This function is intentionally a LOW-SPEED mode transition only.  Never
     * infer zero speed from an invalid observer.  Prefer validated control RPM;
     * if that is unavailable, accept the independent flux-speed estimate only
     * when it itself proves the rotor is already inside the six-step region. */
    float current_rpm = g.control_speed_feedback_rpm;
    float flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
    uint8_t flux_low_valid = (g.obs.theta_meas_valid && isfinite(flux_rpm) &&
                              isfinite(g.obs.bemf_mag) &&
                              g.obs.bemf_mag >= FM_HYBRID_SYNC_MIN_BEMF_V &&
                              fabsf(flux_rpm) <= FM_HYBRID_FOC_EXIT_RPM);

    if (g.control_feedback_blend < 0.5f) {
        if (!flux_low_valid)
            return; /* no positive proof that a high-speed rotor is slow */
        current_rpm = flux_rpm;
        g.control_speed_feedback_rpm = flux_rpm;
        g.control_theta = hybrid_aligned_raw_theta();
        g.forced_omega_e = omega_from_rpm(flux_rpm);
    }

    if (!isfinite(current_rpm) ||
        fabsf(current_rpm) > FM_HYBRID_FOC_ENTER_RPM)
        return;

    g.handover_observer_blend_start = 0.0f;
    float requested_rpm = g.t.target_rpm;
    clear_run_observer_history_for_sixstep();
    /* Six-step must initially follow the PHYSICAL rotor direction.  If the
     * command asks for reversal, state 4 will first decelerate this real
     * direction to near zero and only then run HFI for the opposite launch. */
    float dir = (fabsf(current_rpm) >= FM_HYBRID_STOP_RPM) ?
                hybrid_sign(current_rpm) :
                ((fabsf(requested_rpm) >= FM_HYBRID_STOP_RPM) ?
                 hybrid_sign(requested_rpm) : 1.0f);
    float mag = FM_Clamp(fabsf(current_rpm),
                         FM_TRAP_START_RPM, FM_TRAP_TARGET_RPM);

    float theta_sector_frame = g.control_theta - g.trap_cal_offset_rad;
    if (g.handover_polarity_pi)
        theta_sector_frame -= (float)M_PI;
    theta_sector_frame = FM_WrapPi(theta_sector_frame);

    g.trap_sector = trap_sector_from_rotor_theta(theta_sector_frame, dir);
    g.trap_sector_phase = trap_phase_from_rotor_theta_dir(g.trap_sector,
                                                           theta_sector_frame, dir);
    g.trap_bootstrap_phase = g.trap_sector_phase;
    g.trap_command_rpm = dir * mag;
    g.trap_line_voltage_v = trap_base_line_voltage(mag);
    g.trap_iq_avg = 0.0f;
    g.trap_id_avg = 0.0f;
    /* This transition comes from an already-qualified FOC rotor angle/speed, so
     * six-step can stay closed-loop on the same flux trajectory immediately.
     * There is no reason to throw away a trusted observer and bootstrap again. */
    g.trap_bemf_sync_active = 1U;
    g.trap_had_bemf_lock = 1U;
    g.trap_bemf_sync_good_count =
        FM_TRAP_BEMF_SYNC_GOOD_SAMPLES +
        FM_TRAP_FLUX_LOCK_EXTRA_GOOD_SAMPLES;
    g.trap_bemf_sync_bad_count = 0U;
    g.trap_measured_rpm = current_rpm;
    g.trap_sync_speed_err_rpm = 0.0f;
    g.trap_sync_phase_err_rad = 0.0f;
    g.trap_speed_int_v = 0.0f;
    g.trap_sector_candidate = g.trap_sector;
    g.trap_sector_candidate_count = 0U;
    g.trap_zc_locked = 0U;
    g.trap_zc_good_events = 0U;
    g.trap_zc_rpm = 0.0f;
    g.trap_last_sector_ticks = 0U;
    g.trap_zc_holdover_sectors = 0U;
    g.trap_phase_model_valid = 0U;
    g.trap_floating_bemf_v = 0.0f;
    g.trap_floating_current_a = 0.0f;
    trap_zc_reset_for_sector();
    g.trap_commutation_count = 0U;
    g.trap_cal_observer_started = 1U;
    g.trap_cal_ready = 1U;
    g.trap_cal_offset_rad = 0.0f;
    g.trap_obs_v_accum.alpha = 0.0f;
    g.trap_obs_v_accum.beta = 0.0f;
    g.trap_obs_decim_count = 0U;
    g.trap_theta_cached = theta_sector_frame;
    g.trap_bemf_theta_cached = theta_sector_frame;
    g.trap_bemf_vector_mag_cached = 0.0f;
    g.trap_flux_rpm_cached = current_rpm;
    g.trap_theta_cached_valid = 1U;
    g.trap_bemf_theta_valid_cached = 1U;
    g.trap_flux_angle_available_cached = 1U;
    g.trap_flux_available_cached = flux_low_valid;
    g.handover_obs_v_accum.alpha = 0.0f;
    g.handover_obs_v_accum.beta = 0.0f;
    g.handover_obs_decim_count = 0U;

    g.sling_sample = 0U;
    g.sling_phase = (uint8_t)(g.trap_sector + 1U);
    g.sling_theta = g.control_theta;
    g.sling_hfi_seed_theta = g.control_theta;
    g.forced_theta = g.control_theta;
    g.forced_omega_e = omega_from_rpm(current_rpm);
    g.forced_voltage_v = g.trap_line_voltage_v;
    g.active_target_rpm = g.trap_command_rpm;
    g.handover_polarity_pi = 0U;
    g.feedback_good_count = 0U;
    g.observer_unlock_count = 0U;
    g.run_low_speed_confirm_count = 0U;
    g.forced_observer_blend = 0.0f;
    g.control_feedback_blend = 0.0f;
    g.int_d = 0.0f;
    g.int_q = 0.0f;
    g.speed_int = 0.0f;
    g.t.active_estimator = FM_ESTIMATOR_HFI_SEED;
    g.t.launch_phase = 1U;

    FM_PWM_EnterSixStepMode();
    float duty = (g.vbus > 1.0f) ? (g.trap_line_voltage_v / g.vbus) : 0.0f;
    FM_PWM_SixStep(g.trap_sector, duty);
    g.last_v_fund = trap_sixstep_voltage_ab(g.trap_sector,
                                             g.trap_line_voltage_v);
    set_state(FM_STATE_SLINGSHOT);
}

/* State-6 failure recovery for startup.  Unlike hybrid_enter_sixstep_from_control(),
 * this path is intentionally allowed at the handover speed: it does NOT stop the
 * motor, run HFI, or require proof that speed is already low.  The continuous FOC
 * trajectory gives a recent electrical field angle, so use that only to choose a
 * legal six-step sector, then let the ordinary bootstrap/ZC acquisition run again.
 * This prevents a failed FOC acquisition from becoming a zero-torque 1333-rpm
 * "ghost-speed" state. */
static void trap_resume_sixstep_after_failed_foc(FM_AlphaBeta i_raw)
{
    g.trap_foc_retry_cooldown_samples =
        FM_TRAP_EARLY_FOC_RETRY_COOLDOWN_SAMPLES;

    /* Choose recovery direction from the freshest REAL trajectory, not from a
     * stale command sign.  This matters most on + -> - reversals where a failed
     * negative FOC catch previously could resume six-step with positive RUN
     * holdover history still latched. */
    float signed_resume_rpm = 0.0f;
    if (g.last_good_observer_valid &&
        isfinite(g.last_good_observer_omega_e) &&
        fabsf(rpm_from_omega(g.last_good_observer_omega_e)) >= FM_HYBRID_STOP_RPM)
        signed_resume_rpm = rpm_from_omega(g.last_good_observer_omega_e);
    else if (isfinite(g.control_speed_feedback_rpm) &&
             fabsf(g.control_speed_feedback_rpm) >= FM_HYBRID_STOP_RPM)
        signed_resume_rpm = g.control_speed_feedback_rpm;
    else if (isfinite(g.trap_measured_rpm) &&
             fabsf(g.trap_measured_rpm) >= FM_HYBRID_STOP_RPM)
        signed_resume_rpm = g.trap_measured_rpm;
    else {
        float forced_rpm_now = rpm_from_omega(g.forced_omega_e);
        if (isfinite(forced_rpm_now) && fabsf(forced_rpm_now) >= FM_HYBRID_STOP_RPM)
            signed_resume_rpm = forced_rpm_now;
    }
    if (fabsf(signed_resume_rpm) < FM_HYBRID_STOP_RPM)
        signed_resume_rpm = g.trap_command_rpm;
    float dir = hybrid_sign(signed_resume_rpm);

    /* Preserve the speed already established before the failed FOC attempt.
     * Build 26082956 could return to six-step near ~300 rpm even though the
     * handover had just occurred around 1500 rpm.  Use the largest recent
     * physically relevant trajectory/ZC/setpoint speed so recovery resumes near
     * the handover speed instead of re-running a low-speed launch. */
    float resume_mag = fabsf(signed_resume_rpm);
    float command_mag = fabsf(g.trap_command_rpm);
    float if_mag = fabsf(rpm_from_omega(g.forced_omega_e));
    float measured_mag = fabsf(g.trap_measured_rpm);
    if (isfinite(command_mag) && command_mag > resume_mag) resume_mag = command_mag;
    if (isfinite(if_mag) && if_mag > resume_mag) resume_mag = if_mag;
    if (isfinite(measured_mag) && measured_mag > resume_mag) resume_mag = measured_mag;
    resume_mag = FM_Clamp(resume_mag,
                          FM_TRAP_FOC_PREP_RPM,
                          FM_TRAP_FIELD_CEILING_RPM);

    float theta = g.control_theta;
    if (!isfinite(theta))
        theta = g.forced_theta;
    if (!isfinite(theta))
        theta = g.sling_hfi_seed_theta;
    theta = FM_WrapPi(theta);

    g.trap_sector = trap_sector_from_rotor_theta(theta, dir);
    g.trap_sector_phase = trap_phase_from_rotor_theta_dir(g.trap_sector,
                                                           theta, dir);
    g.trap_bootstrap_phase = g.trap_sector_phase;

    /* Keep the launch setpoint and available line voltage.  Do not restart the
     * 120-rpm ramp and do not remove torque merely because FOC qualification
     * failed. */
    float cmd_mag = resume_mag;
    g.trap_command_rpm = dir * cmd_mag;
    float base_v = trap_base_line_voltage(cmd_mag);
    if (!isfinite(g.trap_line_voltage_v) || g.trap_line_voltage_v < base_v)
        g.trap_line_voltage_v = base_v;
    g.trap_line_voltage_v = FM_Clamp(g.trap_line_voltage_v,
                                     FM_TRAP_MIN_LINE_V,
                                     FM_TRAP_MAX_LINE_V);

    g.trap_bemf_sync_active = 0U;
    g.trap_had_bemf_lock = 0U;
    g.trap_bemf_sync_good_count = 0U;
    g.trap_bemf_sync_bad_count = 0U;
    g.trap_measured_rpm = 0.0f;
    g.trap_zc_locked = 0U;
    g.trap_zc_good_events = 0U;
    g.trap_zc_rpm = 0.0f;
    g.trap_last_sector_ticks = 0U;
    g.trap_zc_holdover_sectors = 0U;
    g.trap_phase_model_valid = 0U;
    g.trap_floating_bemf_v = 0.0f;
    g.trap_floating_current_a = 0.0f;
    trap_zc_reset_for_sector();
    g.trap_speed_int_v = 0.0f;

    /* Re-seed only the PM-flux integrator from the most recent continuous field
     * angle.  This seed is never reported as measured speed and cannot directly
     * declare ZC lock.  First discard every RUN holdover/recatch latch so this
     * six-step acquisition cannot inherit the previous FOC direction. */
    clear_run_observer_history_for_sixstep();
    memset(&g.obs, 0, sizeof(g.obs));
    {
        float ss, cc;
        FM_CordicSinCos(theta, &ss, &cc);
        float resume_rpm = dir * resume_mag;
        float resume_omega_e = omega_from_rpm(resume_rpm);
        g.obs.theta = theta;
        /* This is only a prediction prior.  theta_meas_valid remains false and
         * the normal sample-count qualification must rebuild physical trust.
         * Starting the observer from 0 rpm after a failed ~1500-rpm handover was
         * causing needless sign flips on the next attempt. */
        g.obs.omega_e = resume_omega_e;
        g.obs.flux_omega_e = resume_omega_e;
        g.obs.flux_alpha = FM_MOTOR_FLUX_LINKAGE_WB * cc;
        g.obs.flux_beta = FM_MOTOR_FLUX_LINKAGE_WB * ss;
        g.obs.flux_theta_prev = theta;
        g.obs.flux_initialized = 1U;
        g.obs.flux_theta_prev_valid = 0U;
        g.obs.theta_meas_valid = 0U;
        g.obs.flux_speed_sample_count = 0U;
        g.obs.i_alpha_prev = i_raw.alpha;
        g.obs.i_beta_prev = i_raw.beta;
    }
    g.trap_obs_v_accum.alpha = 0.0f;
    g.trap_obs_v_accum.beta = 0.0f;
    g.trap_obs_decim_count = 0U;
    g.trap_theta_cached = theta;
    g.trap_bemf_theta_cached = theta;
    g.trap_bemf_vector_mag_cached = 0.0f;
    g.trap_flux_rpm_cached = dir * resume_mag;
    g.trap_theta_cached_valid = 1U;
    g.trap_bemf_theta_valid_cached = 0U;
    g.trap_flux_angle_available_cached = 0U;
    g.trap_flux_available_cached = 0U;
    g.handover_obs_v_accum.alpha = 0.0f;
    g.handover_obs_v_accum.beta = 0.0f;
    g.handover_obs_decim_count = 0U;

    g.control_theta = theta;
    g.forced_theta = theta;
    g.forced_omega_e = omega_from_rpm(dir * resume_mag);
    g.control_speed_feedback_rpm = dir * resume_mag;
    g.control_feedback_blend = 0.0f;
    g.forced_observer_blend = 0.0f;
    g.feedback_good_count = 0U;
    g.observer_phase_valid = 0U;
    g.sling_sample = 0U;
    g.active_target_rpm = g.trap_command_rpm;
    g.t.active_estimator = FM_ESTIMATOR_HFI_SEED;
    g.t.launch_phase = 1U;

    FM_PWM_EnterSixStepMode();
    float duty = (g.vbus > 1.0f) ? (g.trap_line_voltage_v / g.vbus) : 0.0f;
    FM_PWM_SixStep(g.trap_sector, duty);
    g.last_v_fund = trap_sixstep_voltage_ab(g.trap_sector,
                                             g.trap_line_voltage_v);
    set_state(FM_STATE_SLINGSHOT);
}

static void hfi_carrier(float *s, float *c)
{
    /* Fs/Fhfi = 40 kHz / 4 kHz = exactly 10 samples.  A LUT is both faster
     * and more deterministic than spending one CORDIC transaction plus a
     * floating-point phase wrap on every PWM sample. */
    static const float sin_lut[10] = {
         0.0f,  0.5877852523f,  0.9510565163f,  0.9510565163f,  0.5877852523f,
         0.0f, -0.5877852523f, -0.9510565163f, -0.9510565163f, -0.5877852523f
    };
    static const float cos_lut[10] = {
         1.0f,  0.8090169944f,  0.3090169944f, -0.3090169944f, -0.8090169944f,
        -1.0f, -0.8090169944f, -0.3090169944f,  0.3090169944f,  0.8090169944f
    };
    uint8_t k = g.hfi.carrier_index;
    *s = sin_lut[k];
    *c = cos_lut[k];
    ++k;
    if (k >= 10U) k = 0U;
    g.hfi.carrier_index = k;
}

static float hfi_effective_voltage(void)
{
    float v = FM_HFI_V;
    if (isfinite(g.vbus) && g.vbus > 1.0f) {
        float duty_floor_v = FM_HFI_MIN_EFFECTIVE_DUTY * g.vbus;
        if (duty_floor_v > v) v = duty_floor_v;
    }
    return FM_Clamp(v, FM_HFI_V, FM_HFI_MAX_EFFECTIVE_V);
}

static FM_AlphaBeta hfi_track_step(FM_AlphaBeta i_raw, float amplitude_scale)
{
    float sh, ch;
    hfi_carrier(&sh, &ch);

    FM_AlphaBeta i_hf = hfi_carrier_current(i_raw);
    float st, ct;
    FM_CordicSinCos(g.hfi.theta, &st, &ct);
    FM_DQ iq_est = park_sc(i_hf, st, ct);
    float demod = iq_est.q * ch;
    float ad = one_pole_alpha(FM_HFI_DEMOD_LPF_HZ);
    g.hfi.demod_lpf += ad * (demod - g.hfi.demod_lpf);

    /* Keep the useful demodulated carrier sample, but do not spend a sqrt in
     * the 40-kHz HFI path just for telemetry. */
    g.t.hfi_carrier_iq_a = iq_est.q;

    /* For pulsating d-axis injection, -LPF(iq*cos(w_h*t)) is locally
     * proportional to (theta_actual - theta_est) when Lq > Ld. */
    g.hfi.error_a = FM_HFI_ERROR_SIGN * (-g.hfi.demod_lpf);

    float scale = (g.hfi.saliency_scale_a > FM_HFI_MIN_SALIENCY_A) ?
                  g.hfi.saliency_scale_a : FM_HFI_MIN_SALIENCY_A;
    float err_norm = FM_Clamp(g.hfi.error_a / scale, -1.0f, 1.0f);
    g.hfi.err_norm = err_norm;
    g.t.hfi_err_norm = err_norm;

    /* A pure type-II PLL can keep a false non-zero omega after a temporary
     * demodulation bias disappears.  That is exactly what the last log showed:
     * HFI error went ~0 while the estimated field kept walking at ~21 rpm.
     * Add controlled omega leakage.  Before real BEMF exists the leak is
     * deliberately strong; once physical motion is independently visible in
     * the voltage-model observer it relaxes so HFI can track acceleration. */
    uint8_t motion_confirmed = observer_motion_coherent();
    g.hfi.motion_coherent = motion_confirmed;
    g.t.observer_motion_coherent = motion_confirmed;

    uint8_t hfi_motion_state = (g.t.state == FM_STATE_OBSERVER_HANDOVER);
    float omega_leak = (motion_confirmed || hfi_motion_state)
                       ? FM_HFI_OMEGA_LEAK_MOVING_S
                       : FM_HFI_OMEGA_LEAK_STILL_S;
    g.hfi.omega_e += (FM_HFI_PLL_KI_RAD_S2 * err_norm
                     - omega_leak * g.hfi.omega_e) * FM_CONTROL_TS_S;

    float hfi_max_rpm = hfi_motion_state ? FM_HFI_MAX_RPM
                                         : FM_HFI_STATIONARY_MAX_RPM;
    g.hfi.omega_e = FM_Clamp(g.hfi.omega_e,
                             -omega_from_rpm(hfi_max_rpm),
                              omega_from_rpm(hfi_max_rpm));
    g.hfi.theta = FM_WrapPi(g.hfi.theta +
                            (g.hfi.omega_e + FM_HFI_PLL_KP_RAD_S * err_norm) * FM_CONTROL_TS_S);

    float ao = one_pole_alpha(FM_HFI_OMEGA_LPF_HZ);
    g.hfi.omega_filt += ao * (g.hfi.omega_e - g.hfi.omega_filt);

    FM_DQ vhf_dq = {hfi_effective_voltage() * amplitude_scale * sh, 0.0f};
    return invpark_sc(vhf_dq, st, ct);
}

static float notch_4khz_step(FM_Notch1 *n, float x)
{
    const float c = FM_HFI_NOTCH_COS_W0;
    const float r = FM_HFI_NOTCH_R;

    /* H(z)=K*(1-2*cos(w0)z^-1+z^-2) /
     *          (1-2*r*cos(w0)z^-1+r^2*z^-2)
     * K normalizes DC gain to 1.  At Fs=40 kHz this places the transmission
     * zero exactly at the 4 kHz HFI carrier while keeping low-frequency phase
     * lag small enough for a fast current loop. */
    float y = FM_HFI_NOTCH_GAIN *
              (x - 2.0f * c * n->x1 + n->x2)
              + 2.0f * r * c * n->y1
              - FM_HFI_NOTCH_R2 * n->y2;

    n->x2 = n->x1;
    n->x1 = x;
    n->y2 = n->y1;
    n->y1 = y;
    return y;
}

static void update_hfi_current_feedback(FM_AlphaBeta i_raw)
{
    g.i_lf.alpha = notch_4khz_step(&g.notch_alpha, i_raw.alpha);
    g.i_lf.beta  = notch_4khz_step(&g.notch_beta,  i_raw.beta);
}

static void reset_notch_to_steady_input(FM_Notch1 *n, float x)
{
    /* With x1=x2=y1=y2=x the unity-DC-gain notch starts exactly at the
     * present current rather than ringing from the just-removed HFI carrier. */
    n->x1 = x;
    n->x2 = x;
    n->y1 = x;
    n->y2 = x;
}

static FM_HOT_ISR void observer_step(FM_AlphaBeta i, FM_AlphaBeta v_applied, uint8_t seed_mode,
                          float seed_theta, float seed_omega_e,
                          float seed_pll_enable_rpm, float pll_blend,
                          float bemf_valid_min_v, float dt_s)
{
    (void)seed_pll_enable_rpm;
    (void)pll_blend;

    FM_Observer *o = &g.obs;

    /* observer_step is called only at 40/20/10 kHz in this firmware.  The old
     * generic form paid five floating-point divisions per observer update
     * (three LPF coefficients + two dI/dt divisions).  Use the exact
     * precomputed coefficients for those three periods; fall back to the
     * generic formula only for an unexpected dt.  This changes scheduling
     * cost, not the observer equations. */
    float ad, ae, aw, inv_dt;
    if (dt_s < (1.5f * FM_CONTROL_TS_S)) {
        ad = 0.485193601f;
        ae = 0.281969800f;
        aw = 0.111635212f;
        inv_dt = 40000.0f;
    } else if (dt_s < (3.0f * FM_CONTROL_TS_S)) {
        ad = 0.653374214f;
        ae = 0.439900846f;
        aw = 0.200848643f;
        inv_dt = 20000.0f;
    } else if (dt_s < (5.0f * FM_CONTROL_TS_S)) {
        ad = 0.790352491f;
        ae = 0.611015470f;
        aw = 0.334511171f;
        inv_dt = 10000.0f;
    } else {
        ad = one_pole_alpha_dt(FM_DIDT_LPF_HZ, dt_s);
        ae = one_pole_alpha_dt(FM_BEMF_LPF_HZ, dt_s);
        aw = one_pole_alpha_dt(FM_FLUX_SPEED_LPF_HZ, dt_s);
        inv_dt = 1.0f / dt_s;
    }

    /* Keep the old derivative BEMF calculation as an independent magnitude /
     * health diagnostic, but DO NOT use its phase as rotor angle.  The old
     * build did atan2(-e_alpha,e_beta) + pi/2; that +90-degree term came from a
     * virtual-rotor experiment and produced a ~75-degree hardware handover. */
    float delta_ia = i.alpha - o->i_alpha_prev;
    float delta_ib = i.beta  - o->i_beta_prev;
    float dia = delta_ia * inv_dt;
    float dib = delta_ib * inv_dt;
    o->i_alpha_prev = i.alpha;
    o->i_beta_prev  = i.beta;

    o->di_alpha += ad * (dia - o->di_alpha);
    o->di_beta  += ad * (dib - o->di_beta);

    /* v_applied is the voltage command that was present during the PWM interval
     * which produced THIS current sample.  Do not use the command just computed
     * later in this ISR; that one-sample timing error rotates the voltage model. */
    float va = v_applied.alpha;
    float vb = v_applied.beta;
    /* Keep truthful x5 current scaling for FOC/protection, but reproduce the
     * observer dynamics of the best x10-firmware/x5-hardware run explicitly.
     * That run effectively halved the R*i and L*di/dt subtraction terms. */
    float ea = va - FM_OBSERVER_PHASE_R_OHM * i.alpha - FM_OBSERVER_PHASE_L_H * o->di_alpha;
    float eb = vb - FM_OBSERVER_PHASE_R_OHM * i.beta  - FM_OBSERVER_PHASE_L_H * o->di_beta;
    o->e_alpha += ae * (ea - o->e_alpha);
    o->e_beta  += ae * (eb - o->e_beta);

    float e2 = o->e_alpha * o->e_alpha + o->e_beta * o->e_beta;
    if (bemf_valid_min_v < 0.0f) bemf_valid_min_v = 0.0f;
    float valid2 = bemf_valid_min_v * bemf_valid_min_v;

    /* A2212/5T 2450KV: 14 rotor poles -> 7 pole pairs.  The KV-derived PM flux
     * linkage is used only to seed and radially stabilize the observer; angle
     * comes from the integrated voltage/current model.  This is an MXLEMMING-
     * style flux update written directly for this controller:
     *
     *   psi_pm[k+1] = psi_pm[k] + (v - R i) Ts - L (i[k]-i[k-1])
     *
     * It avoids differentiating current to obtain the rotor angle. */
    if (!o->flux_initialized) {
        float th = seed_mode ? seed_theta : o->theta;
        float s, c;
        FM_CordicSinCos(th, &s, &c);
        o->flux_alpha = FM_MOTOR_FLUX_LINKAGE_WB * c;
        o->flux_beta  = FM_MOTOR_FLUX_LINKAGE_WB * s;
        o->flux_theta_prev = th;
        o->flux_omega_e = seed_mode ? seed_omega_e : o->omega_e;
        o->flux_initialized = 1U;
        o->flux_theta_prev_valid = 0U;
    }

    o->flux_alpha += (va - FM_OBSERVER_PHASE_R_OHM * i.alpha) * dt_s
                     - FM_OBSERVER_PHASE_L_H * delta_ia;
    o->flux_beta  += (vb - FM_OBSERVER_PHASE_R_OHM * i.beta) * dt_s
                     - FM_OBSERVER_PHASE_L_H * delta_ib;

    /* Radial correction prevents DC voltage-model error from making the
     * integrated flux vector walk away.  It changes magnitude, not angle. */
    const float lambda2 = FM_MOTOR_FLUX_LINKAGE_WB * FM_MOTOR_FLUX_LINKAGE_WB;
    float flux2 = o->flux_alpha * o->flux_alpha + o->flux_beta * o->flux_beta;
    if (isfinite(flux2) && flux2 > 1.0e-14f) {
        float radial_err = (lambda2 - flux2) / (lambda2 + flux2 + 1.0e-14f);
        float k = 1.0f + FM_FLUX_RADIAL_CORRECTION * radial_err;
        o->flux_alpha *= k;
        o->flux_beta  *= k;
        flux2 = o->flux_alpha * o->flux_alpha + o->flux_beta * o->flux_beta;
    }

    /* Hard component ceiling is a numerical guard only. */
    const float flux_lim = FM_FLUX_COMPONENT_LIMIT_RATIO * FM_MOTOR_FLUX_LINKAGE_WB;
    o->flux_alpha = FM_Clamp(o->flux_alpha, -flux_lim, flux_lim);
    o->flux_beta  = FM_Clamp(o->flux_beta,  -flux_lim, flux_lim);
    flux2 = o->flux_alpha * o->flux_alpha + o->flux_beta * o->flux_beta;

    const float min_flux = FM_FLUX_MIN_MAG_RATIO * FM_MOTOR_FLUX_LINKAGE_WB;
    const float min_flux2 = min_flux * min_flux;
    o->theta_meas_valid = 0U;

    if (isfinite(flux2) && flux2 > min_flux2 && e2 > valid2) {
        float theta_flux = FM_CordicAtan2(o->flux_beta, o->flux_alpha);
        o->theta_meas_raw = theta_flux;
        o->theta_meas_valid = 1U;

        /* Independent speed from the flux-angle trajectory.  Reject impossible
         * one-sample phase jumps rather than integrating them into a fake
         * negative RPM, which is exactly how the old observer failed. */
        if (o->flux_theta_prev_valid) {
            float dtheta = FM_WrapPi(theta_flux - o->flux_theta_prev);
            if (fabsf(dtheta) <= FM_FLUX_MAX_SAMPLE_DELTA_RAD) {
                float omega_inst = dtheta * inv_dt;
                if (FM_OBSERVER_FORWARD_ONLY && o->polarity_resolved && omega_inst < 0.0f)
                    omega_inst = 0.0f;
                omega_inst = FM_Clamp(omega_inst,
                                      FM_OBSERVER_FORWARD_ONLY && o->polarity_resolved ? 0.0f : -omega_from_rpm(FM_MAX_RPM),
                                      omega_from_rpm(FM_MAX_RPM));
                o->flux_omega_e += aw * (omega_inst - o->flux_omega_e);
                if (o->flux_speed_sample_count < 0xFFFFFFFFU)
                    ++o->flux_speed_sample_count;
            } else {
                o->flux_speed_sample_count = 0U;
            }
        }
        o->flux_theta_prev = theta_flux;
        o->flux_theta_prev_valid = 1U;

        if (seed_mode) {
            /* Forced angle owns control during acquisition.  The flux observer
             * remains independent and telemetry exposes its physical lag. */
            o->theta = seed_theta;
            o->omega_e = seed_omega_e;
            o->phase_err = FM_WrapPi(theta_flux - seed_theta);
            o->voltage_to_rotor_lag_rad = o->phase_err;
        } else {
            /* High-bandwidth type-II angle PLL.
             *
             * The raw PM-flux angle is ONLY a phase-detector measurement.  It
             * never replaces o->theta directly.  o->theta is first predicted
             * from the estimated electrical speed, then the wrapped flux-angle
             * residual corrects phase and speed.  This is important on the
             * A2212: one bad voltage-model sample used to move the observer by
             * 90..130 electrical degrees even though the rotor can only move a
             * few degrees in one 25-us ISR at the present test speed.
             *
             * A small phase-detector clamp is part of the PLL itself, not a
             * multi-sample qualification gate: a large raw-angle glitch can
             * only pull the PLL a few degrees per sample.  The un-clamped
             * residual remains in phase_err so RUN health logic still sees the
             * disturbance immediately and can use propagated holdover. */
            float theta_pred = FM_WrapPi(o->theta + o->omega_e * dt_s);
            float innovation_raw = FM_WrapPi(theta_flux - theta_pred);
            float innovation_pll = FM_Clamp(innovation_raw,
                                            -FM_PLL_PHASE_DETECTOR_CLAMP_RAD,
                                             FM_PLL_PHASE_DETECTOR_CLAMP_RAD);
            o->phase_err = innovation_raw;
            o->voltage_to_rotor_lag_rad = innovation_raw;

            /* Type-II PLL:
             *   omega_dot = Ki * phase_error
             *   theta_dot = omega + Kp * phase_error
             *
             * Kp/Ki are intentionally much faster than the legacy values.
             * Prediction carries high electrical speed; bandwidth is used to
             * correct model/phase error, not to low-pass the rotor motion. */
            float dw = FM_PLL_KI_RAD_S2_PER_RAD * innovation_pll * dt_s;
            float max_dw = omega_from_rpm(FM_PLL_MAX_MECH_ACCEL_RPM_S) * dt_s;
            dw = FM_Clamp(dw, -max_dw, max_dw);
            o->omega_e += dw;

            if (FM_OBSERVER_FORWARD_ONLY && o->polarity_resolved && o->omega_e < 0.0f)
                o->omega_e = 0.0f;
            o->omega_e = FM_Clamp(o->omega_e,
                                  FM_OBSERVER_FORWARD_ONLY && o->polarity_resolved ? 0.0f : -omega_from_rpm(FM_PLL_MAX_MECH_RPM),
                                  omega_from_rpm(FM_PLL_MAX_MECH_RPM));

            float phase_rate_correction = FM_PLL_KP_RAD_S_PER_RAD * innovation_pll;
            o->theta = FM_WrapPi(theta_pred + phase_rate_correction * dt_s);
        }
    } else if (seed_mode) {
        o->theta = seed_theta;
        o->omega_e = seed_omega_e;
        o->phase_err = 0.0f;
    } else {
        o->theta = FM_WrapPi(o->theta + o->omega_e * dt_s);
    }

    float rpm = rpm_from_omega(o->omega_e);
    float lock2 = FM_BEMF_LOCK_MIN_V * FM_BEMF_LOCK_MIN_V;
    if (o->theta_meas_valid && e2 > lock2 &&
        fabsf(rpm) > FM_LOCK_MIN_RPM &&
        fabsf(o->phase_err) < FM_LOCK_MAX_PHASE_ERR_RAD) {
        if (o->lock_count < FM_LOCK_SAMPLES) ++o->lock_count;
    } else {
        o->lock_count = 0U;
    }
    o->locked = (o->lock_count >= FM_LOCK_SAMPLES) ? 1U : 0U;

    if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U)
        o->bemf_mag = sqrtf(e2);
}

/* Reset the independent BEMF-angle window and the controller-history window
 * together.  A rejected angular window therefore cannot leave stale current-PI
 * history behind for a later takeover. */
static void handover_history_reset(void)
{
    g.handover_angle_acquired = 0U;
    g.handover_polarity_pi = 0U;
    g.handover_angle_samples = 0U;
    g.handover_pi_votes = 0U;
    g.handover_residual_mean = 0.0f;
    g.handover_residual_m2 = 0.0f;
    g.handover_angle_offset_rad = 0.0f;
    g.handover_control_samples = 0U;
    g.handover_id_meas_avg = 0.0f;
    g.handover_iq_meas_avg = 0.0f;
    g.handover_iq_ref_avg = 0.0f;
    g.handover_vd_avg = 0.0f;
    g.handover_vq_avg = 0.0f;
    g.handover_int_d_avg = 0.0f;
    g.handover_int_q_avg = 0.0f;
    g.if_raw_id_sum = 0.0f;
    g.if_raw_iq_sum = 0.0f;
    g.if_raw_samples = 0U;
    g.if_pi_seeded = 0U;
}

/* Qualify the PM-FLUX angle, not the seed-biased PLL angle.  Each raw
 * sample is first de-rotated by the monotonic HFI-seeded command angle.  The pi
 * branch that is closest to that trajectory is selected per sample, then a
 * Welford window measures whether the residual is actually coherent.
 *
 * This deliberately does not require the instantaneous observer phase_err to
 * be small.  The 2026-08-26 log showed 0.6..2 V BEMF with useful angle samples,
 * while the instantaneous phase_err gate kept resetting the old counter. */
static uint8_t handover_angle_window_step(float reference_theta)
{
    if (g.handover_angle_acquired)
        return 1U;

    /* Acquisition must be a genuinely consecutive PM-flux trajectory.
     * Do not accumulate a few good samples scattered between weak/noisy ones. */
    if (!g.obs.theta_meas_valid) {
        if (g.handover_angle_samples > 0U)
            handover_history_reset();
        return 0U;
    }

    float e2 = g.obs.e_alpha * g.obs.e_alpha + g.obs.e_beta * g.obs.e_beta;
    float min2 = FM_SLING_ACQUIRE_BEMF_V * FM_SLING_ACQUIRE_BEMF_V;
    if (!isfinite(e2) || e2 < min2) {
        if (g.handover_angle_samples > 0U)
            handover_history_reset();
        return 0U;
    }

    /* Flux angle has physical magnet polarity, unlike the old BEMF-vector
     * phase which was ambiguous modulo pi.  Do not hide a wrong branch by
     * adding pi here. */
    float residual = FM_WrapPi(g.obs.theta_meas_raw - reference_theta);
    float flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
    float forced_rpm = rpm_from_omega(g.forced_omega_e);

    if (!isfinite(residual) || fabsf(residual) > FM_SLING_ACQUIRE_MAX_RESIDUAL_RAD ||
        !isfinite(flux_rpm) || fabsf(flux_rpm) < FM_TRAP_FOC_FLUX_MIN_RPM ||
        flux_rpm * forced_rpm <= 0.0f ||
        fabsf(flux_rpm - forced_rpm) > FM_TRAP_FOC_FLUX_MAX_RPM_ERR) {
        handover_history_reset();
        return 0U;
    }

    /* A single discontinuous raw-angle sample must not poison an otherwise
     * useful window.  Restart the window from the current sample instead. */
    if (g.handover_angle_samples > 0U &&
        fabsf(residual - g.handover_residual_mean) > FM_SLING_ACQUIRE_MAX_INNOVATION_RAD) {
        handover_history_reset();
    }

    ++g.handover_angle_samples;

    float n = (float)g.handover_angle_samples;
    float d = residual - g.handover_residual_mean;
    g.handover_residual_mean += d / n;
    float d2 = residual - g.handover_residual_mean;
    g.handover_residual_m2 += d * d2;
    g.forced_observer_good_count = g.handover_angle_samples;

    if (g.handover_angle_samples < FM_SLING_ACQUIRE_SAMPLES)
        return 0U;

    float variance = (g.handover_angle_samples > 1U) ?
                     g.handover_residual_m2 / (float)(g.handover_angle_samples - 1U) : 0.0f;
    if (!isfinite(variance) || variance > FM_SLING_ACQUIRE_MAX_VARIANCE_RAD2) {
        handover_history_reset();
        return 0U;
    }

    g.handover_polarity_pi = 0U;
    g.handover_angle_offset_rad = g.handover_residual_mean;
    g.handover_angle_acquired = 1U;

    g.obs.polarity_pi = 0U;
    g.obs.polarity_resolved = 1U;
    g.obs.theta = FM_WrapPi(reference_theta + g.handover_angle_offset_rad);
    /* Do not seed takeover speed from a still-settling LPF.  Six-step already
     * supplies a known forward commutation rate; start the free observer there
     * and let the faster flux-speed estimate correct it after handover. */
    g.obs.omega_e = g.forced_omega_e;
    g.obs.phase_err = 0.0f;
    g.obs.voltage_to_rotor_lag_rad = g.handover_angle_offset_rad;
    return 1U;
}

/* Running means of ALL controller samples that belong to the accepted angle
 * window / blend.  These values represent the applied torque state much better
 * than the final ISR sample alone. */
static void handover_control_history_add(float iq_ref)
{
    if (g.handover_angle_samples == 0U && !g.handover_angle_acquired)
        return;

    ++g.handover_control_samples;
    float n = (float)g.handover_control_samples;
#define FM_MEAN_UPDATE(avg, x) ((avg) += ((x) - (avg)) / n)
    FM_MEAN_UPDATE(g.handover_id_meas_avg, g.t.id_a);
    FM_MEAN_UPDATE(g.handover_iq_meas_avg, g.t.iq_a);
    FM_MEAN_UPDATE(g.handover_iq_ref_avg, iq_ref);
    FM_MEAN_UPDATE(g.handover_vd_avg, g.t.vd_cmd_v);
    FM_MEAN_UPDATE(g.handover_vq_avg, g.t.vq_cmd_v);
    FM_MEAN_UPDATE(g.handover_int_d_avg, g.int_d);
    FM_MEAN_UPDATE(g.handover_int_q_avg, g.int_q);
#undef FM_MEAN_UPDATE

    g.if_raw_id_sum += g.t.id_a;
    g.if_raw_iq_sum += g.t.iq_a;
    ++g.if_raw_samples;
}

/* Preload the current PI from the multi-sample handover history.  The seed is
 * reconstructed from averaged applied Vd/Vq and averaged current error, then
 * softly combined with the averaged live integrator state.  current_control()
 * integrates once before producing its first output, hence Kp+Ki below. */
static float seed_current_pi_from_handover_history(void)
{
    float vmax = FM_VOLTAGE_LIMIT_RATIO * g.vbus;

    /* If no history exists, preserve the already-live integrators instead of
     * inventing a zero-voltage seed.  This path should not normally execute. */
    if (g.handover_control_samples == 0U) {
        float iq_entry = FM_Clamp(g.t.iq_ref_a, 0.0f, FM_SLING_KICK_IQ_A);
        g.handover_iq_start_a = iq_entry;
        g.bemf_boost_iq_cmd_a = iq_entry;
        g.bemf_boost_entry_sample = 0U;
        g.if_iq_ref_a = iq_entry;
        return iq_entry;
    }

    float iq_ref_avg = g.handover_iq_ref_avg;
    float iq_meas_avg = g.handover_iq_meas_avg;
    float id_meas_avg = g.handover_id_meas_avg;

    /* Never command more torque at the instant of handover than the coherent
     * launch window was already commanding.  If measured Iq lagged the ref,
     * split the difference so the new loop does not suddenly chase the entire
     * old error in one sample. */
    float iq_mid = 0.5f * (iq_ref_avg + iq_meas_avg);
    float iq_entry = fminf(iq_ref_avg, iq_mid);
    iq_entry = FM_Clamp(iq_entry, 0.0f, FM_SLING_KICK_IQ_A);

    float ed = -id_meas_avg;
    float eq = iq_entry - iq_meas_avg;
    float id_recon = g.handover_vd_avg -
                     (FM_LAUNCH_CURRENT_KP_V_PER_A + FM_LAUNCH_CURRENT_KI_STEP_V_PER_A) * ed;
    float iq_recon = g.handover_vq_avg -
                     (FM_LAUNCH_CURRENT_KP_V_PER_A + FM_LAUNCH_CURRENT_KI_STEP_V_PER_A) * eq;

    float hist_k = FM_HANDOVER_PI_HISTORY_BLEND;
    g.int_d = FM_Clamp(hist_k * g.handover_int_d_avg + (1.0f - hist_k) * id_recon,
                       -vmax, vmax);
    g.int_q = FM_Clamp(hist_k * g.handover_int_q_avg + (1.0f - hist_k) * iq_recon,
                       -vmax, vmax);

    g.handover_iq_start_a = iq_entry;
    g.bemf_boost_iq_cmd_a = iq_entry;
    g.bemf_boost_entry_sample = 0U;
    g.if_iq_ref_a = iq_entry;
    g.if_pi_seeded = 1U;
    return iq_entry;
}

/* Mechanical-equivalent command for the forced rotating FOC frame.
 * This is intentionally cheap enough for the 40 kHz ISR: one multiply/add and
 * clamps only.  It is a COMMAND, never rotor feedback. */
static float sling_forced_rpm(uint32_t sample)
{
    if (sample >= FM_SLING_ROTATE_RAMP_SAMPLES)
        return FM_SLING_ROTATE_TARGET_RPM;

    const float inv = 1.0f / (float)FM_SLING_ROTATE_RAMP_SAMPLES;
    float k = (float)sample * inv;
    k = FM_Clamp(k, 0.0f, 1.0f);
    return FM_SLING_ROTATE_START_RPM +
           k * (FM_SLING_ROTATE_TARGET_RPM - FM_SLING_ROTATE_START_RPM);
}

static void sling_zvv_reset(FM_AlphaBeta i_raw)
{
    g.sling_i_alpha_prev = i_raw.alpha;
    g.sling_i_beta_prev = i_raw.beta;
    g.sling_di_alpha = 0.0f;
    g.sling_di_beta = 0.0f;
    g.sling_e_alpha = 0.0f;
    g.sling_e_beta = 0.0f;
    g.sling_prev_bemf_theta = 0.0f;
    g.sling_unwrapped_theta = 0.0f;
    g.sling_first_bemf_theta = 0.0f;
    g.sling_last_bemf_theta = 0.0f;
    g.sling_bemf_sum = 0.0f;
    g.sling_peak_bemf = 0.0f;
    g.sling_valid_samples = 0U;
    g.sling_first_valid_sample = 0U;
    g.sling_last_valid_sample = 0U;
    g.sling_have_prev_theta = 0U;
    g.t.sling_bemf_v = 0.0f;
    g.t.sling_rpm = 0.0f;
    g.t.sling_angle_rad = g.sling_theta;
    g.t.sling_valid_samples = 0U;
}

/* During a commanded zero-voltage vector, v_alpha=v_beta=0.  With only the
 * existing phase-current sensors available on this board, the instantaneous
 * BEMF vector follows e = -R*i - L*di/dt.  A short ZVV window therefore gives
 * us a real rotating electrical vector without adding phase-voltage ADCs. */
static void sling_zvv_step(FM_AlphaBeta i_raw)
{
    const float ad = one_pole_alpha(FM_SLING_ZVV_DIDT_LPF_HZ);
    const float ae = one_pole_alpha(FM_SLING_ZVV_BEMF_LPF_HZ);

    float dia = (i_raw.alpha - g.sling_i_alpha_prev) / FM_CONTROL_TS_S;
    float dib = (i_raw.beta  - g.sling_i_beta_prev)  / FM_CONTROL_TS_S;
    g.sling_i_alpha_prev = i_raw.alpha;
    g.sling_i_beta_prev = i_raw.beta;

    g.sling_di_alpha += ad * (dia - g.sling_di_alpha);
    g.sling_di_beta  += ad * (dib - g.sling_di_beta);

    float ea = -FM_PHASE_R_OHM * i_raw.alpha - FM_PHASE_L_H * g.sling_di_alpha;
    float eb = -FM_PHASE_R_OHM * i_raw.beta  - FM_PHASE_L_H * g.sling_di_beta;
    g.sling_e_alpha += ae * (ea - g.sling_e_alpha);
    g.sling_e_beta  += ae * (eb - g.sling_e_beta);

    float mag = sqrtf(g.sling_e_alpha * g.sling_e_alpha +
                      g.sling_e_beta  * g.sling_e_beta);
    if (mag > g.sling_peak_bemf)
        g.sling_peak_bemf = mag;
    g.t.sling_bemf_v = mag;

    if (!isfinite(mag) || mag < FM_SLING_MIN_BEMF_V)
        return;

    /* The constant phase mapping is the same one used by the normal BEMF
     * observer.  It does not affect direction, but it lets the final ZVV angle
     * seed the normal observer in the same electrical-angle convention. */
    float theta = FM_CordicAtan2(-g.sling_e_alpha, g.sling_e_beta);
    theta = FM_WrapPi(theta + FM_OBSERVER_BEMF_ANGLE_OFFSET_RAD);

    if (!g.sling_have_prev_theta) {
        g.sling_have_prev_theta = 1U;
        g.sling_prev_bemf_theta = theta;
        g.sling_first_bemf_theta = theta;
        g.sling_last_bemf_theta = theta;
        g.sling_first_valid_sample = g.sling_sample;
        g.sling_last_valid_sample = g.sling_sample;
        g.sling_bemf_sum = mag;
        g.sling_valid_samples = 1U;
    } else {
        float dtheta = FM_WrapPi(theta - g.sling_prev_bemf_theta);

        /* At startup a real 25-us electrical step is tiny.  Reject an isolated
         * large phase jump as derivative/noise corruption instead of turning it
         * into a fictitious thousands-of-rpm estimate. */
        if (fabsf(dtheta) <= 0.20f) {
            g.sling_unwrapped_theta += dtheta;
            g.sling_prev_bemf_theta = theta;
            g.sling_last_bemf_theta = theta;
            g.sling_last_valid_sample = g.sling_sample;
            g.sling_bemf_sum += mag;
            ++g.sling_valid_samples;
        }
    }

    g.t.sling_angle_rad = g.sling_last_bemf_theta;
    g.t.sling_valid_samples = g.sling_valid_samples;

    if (g.sling_valid_samples >= 2U &&
        g.sling_last_valid_sample > g.sling_first_valid_sample) {
        float dt = (float)(g.sling_last_valid_sample - g.sling_first_valid_sample) *
                   FM_CONTROL_TS_S;
        if (dt > 0.0f) {
            float omega_e = g.sling_unwrapped_theta / dt;
            g.t.sling_rpm = rpm_from_omega(omega_e);
        }
    }
}

static uint8_t sling_zvv_result(float *rpm_out, float *bemf_out, float *theta_out)
{
    if (g.sling_valid_samples < FM_SLING_ZVV_MIN_VALID_SAMPLES ||
        g.sling_last_valid_sample <= g.sling_first_valid_sample)
        return 0U;

    float dt = (float)(g.sling_last_valid_sample - g.sling_first_valid_sample) *
               FM_CONTROL_TS_S;
    if (dt <= 0.0f)
        return 0U;

    float omega_e = g.sling_unwrapped_theta / dt;
    float rpm = rpm_from_omega(omega_e);
    float bemf = g.sling_bemf_sum / (float)g.sling_valid_samples;

    /* Compensate the dedicated ZVV BEMF low-pass before using its final angle
     * as the normal observer seed. */
    float ratio = fabsf(omega_e) / (FM_TWO_PI * FM_SLING_ZVV_BEMF_LPF_HZ);
    ratio = FM_Clamp(ratio, 0.0f, 1.0f);
    float theta = FM_WrapPi(g.sling_last_bemf_theta + fast_atan_0_to_1(ratio));

    g.t.sling_rpm = rpm;
    g.t.sling_bemf_v = bemf;
    g.t.sling_angle_rad = theta;
    g.t.sling_valid_samples = g.sling_valid_samples;

    if (rpm_out != NULL) *rpm_out = rpm;
    if (bemf_out != NULL) *bemf_out = bemf;
    if (theta_out != NULL) *theta_out = theta;

    return (isfinite(rpm) && isfinite(bemf) && isfinite(theta) &&
            bemf >= FM_SLING_MIN_BEMF_V &&
            fabsf(rpm) >= FM_SLING_MIN_DIRECTION_RPM) ? 1U : 0U;
}

static void seed_observer_from_sling(FM_AlphaBeta i_raw,
                                     float theta, float rpm, float bemf_v)
{
    memset(&g.obs, 0, sizeof(g.obs));
    g.obs.theta = theta;
    g.obs.omega_e = omega_from_rpm(rpm);
    g.obs.i_alpha_prev = i_raw.alpha;
    g.obs.i_beta_prev = i_raw.beta;
    g.obs.di_alpha = g.sling_di_alpha;
    g.obs.di_beta = g.sling_di_beta;
    g.obs.e_alpha = g.sling_e_alpha;
    g.obs.e_beta = g.sling_e_beta;
    g.obs.bemf_mag = bemf_v;
    g.obs.polarity_resolved = 1U;
    g.obs.polarity_pi = 0U;
    g.obs.phase_err = 0.0f;

    g.control_theta = theta;
    g.control_speed_feedback_rpm = rpm;
    g.control_feedback_blend = 1.0f;
    g.last_good_observer_theta = theta;
    g.last_good_observer_omega_e = g.obs.omega_e;
    g.last_good_observer_valid = 1U;
    g.observer_holdover_active = 0U;
    g.observer_holdover_count = 0U;
    observer_recatch_reset();
    g.observer_unlock_count = 0U;
    g.bemf_boost_bad_count = 0U;
}

static void speed_pi_step_limited_gains(float measured_rpm, float requested_rpm,
                                        float iq_max_a, float kp, float ki)
{
    float erpm = requested_rpm - measured_rpm;
    float speed_ts = FM_CONTROL_TS_S * (float)FM_SPEED_LOOP_DIV;
    g.speed_int += ki * erpm * speed_ts;
    g.speed_int = FM_Clamp(g.speed_int, -iq_max_a, iq_max_a);
    g.t.iq_ref_a = FM_Clamp(kp * erpm + g.speed_int,
                            -iq_max_a, iq_max_a);
}

static void speed_pi_step_limited(float measured_rpm, float requested_rpm, float iq_max_a)
{
    speed_pi_step_limited_gains(measured_rpm, requested_rpm, iq_max_a,
                                FM_SPEED_KP_A_PER_RPM,
                                FM_SPEED_KI_A_PER_RPM_S);
}

static float run_iq_limit_for_rpm(float measured_rpm)
{
    float rpm = fabsf(measured_rpm);
    if (!isfinite(rpm) || rpm <= FM_SPEED_IQ_TAPER_START_RPM)
        return FM_SPEED_IQ_MAX_A;
    if (rpm >= FM_SPEED_IQ_TAPER_END_RPM)
        return FM_SPEED_IQ_HIGH_RPM_A;

    float k = (rpm - FM_SPEED_IQ_TAPER_START_RPM) /
              (FM_SPEED_IQ_TAPER_END_RPM - FM_SPEED_IQ_TAPER_START_RPM);
    return FM_SPEED_IQ_MAX_A +
           k * (FM_SPEED_IQ_HIGH_RPM_A - FM_SPEED_IQ_MAX_A);
}

static void speed_pi_step(float measured_rpm, float requested_rpm)
{
    speed_pi_step_limited(measured_rpm, requested_rpm,
                          run_iq_limit_for_rpm(measured_rpm));
}

static void ramp_target(float wanted_rpm, float rate_rpm_s)
{
    float step = rate_rpm_s * FM_CONTROL_TS_S;
    if (g.active_target_rpm < wanted_rpm) {
        float next = g.active_target_rpm + step;
        g.active_target_rpm = (next < wanted_rpm) ? next : wanted_rpm;
    } else {
        float next = g.active_target_rpm - step;
        g.active_target_rpm = (next > wanted_rpm) ? next : wanted_rpm;
    }
}

static FM_HOT_ISR void current_control(float theta, FM_AlphaBeta i_ctrl, float id_ref, float iq_ref, FM_AlphaBeta v_hfi)
{
    float st, ct;
    FM_CordicSinCos(theta, &st, &ct);
    FM_DQ idq = park_sc(i_ctrl, st, ct);
    float ed = id_ref - idq.d;
    float eq = iq_ref - idq.q;
    float voltage_limit_ratio = FM_VOLTAGE_LIMIT_RATIO;
    if (g.t.state == FM_STATE_BEMF_BOOST &&
        g.foc_capture_load_boost_count >= FM_HYBRID_CAPTURE_LOAD_BOOST_SAMPLES)
        voltage_limit_ratio = FM_HEAVY_LOAD_VOLTAGE_LIMIT_RATIO;
    float vmax = voltage_limit_ratio * g.vbus;
    /* Use the tuned low-speed launch gains while HFI owns the rotor frame.
     * The normal inverter vector limit and independent current protection remain
     * active; there is no artificial open-loop launch-voltage clamp. */
    uint8_t pi_capture = 0U;

    /* The slingshot needs a genuinely fast current build, but it remains a
     * bounded current loop rather than an uncontrolled full-bus voltage pulse.
     * Once BEMF has been acquired, fall back to the gentler launch gains. */
    uint8_t slingshot_kick = (g.t.state == FM_STATE_SLINGSHOT && g.sling_phase == 1U);
    uint8_t launch_or_handover =
        (g.t.state == FM_STATE_BEMF_BOOST ||
         g.t.state == FM_STATE_OBSERVER_HANDOVER);

    /* At standstill the motor impedance is tiny.  A bus-relative voltage limit
     * allowed the current PI to request 6-8 V before useful motion existed and
     * repeatedly drove the current amplifiers into their ~33 A rails.  Bound
     * ONLY the slingshot kick to a small absolute voltage.  This makes the
     * impulse largely independent of whether the bench supply is 13 V or 23 V
     * and, critically, lets the kick last long enough to create momentum. */
    if (slingshot_kick && vmax > FM_SLING_VOLTAGE_LIMIT_V)
        vmax = FM_SLING_VOLTAGE_LIMIT_V;

    /* Keep the topology boundary bounded, but give FOC the same 2.6-V
     * fundamental-voltage authority already available to six-step.  The old
     * 1.5-V cap could not sustain loaded torque even when the RPM PI requested
     * more current, so the rotor decelerated before the observer could lock. */
    if (g.t.state == FM_STATE_OBSERVER_HANDOVER &&
        vmax > FM_TRAP_FOC_CAPTURE_VOLTAGE_LIMIT_V)
        vmax = FM_TRAP_FOC_CAPTURE_VOLTAGE_LIMIT_V;

    float kp;
    float ki_step;
    if (pi_capture) {
        kp = FM_PI_CAPTURE_KP_V_PER_A;
        ki_step = FM_PI_CAPTURE_KI_STEP_V_PER_A;
    } else if (slingshot_kick) {
        kp = FM_SLING_CURRENT_KP_V_PER_A;
        ki_step = FM_SLING_CURRENT_KI_STEP_V_PER_A;
    } else if (launch_or_handover) {
        kp = FM_LAUNCH_CURRENT_KP_V_PER_A;
        ki_step = FM_LAUNCH_CURRENT_KI_STEP_V_PER_A;
    } else if (g.t.state == FM_STATE_RUN) {
        const float run_ramp_samples =
            (float)(FM_RUN_PI_GAIN_RAMP_MS * 40U);
        float gain_k = FM_Clamp(
            (float)(g.isr_count - g.state_start_isr_count) /
            run_ramp_samples, 0.0f, 1.0f);
        kp = FM_LAUNCH_CURRENT_KP_V_PER_A +
             gain_k * (FM_CURRENT_KP_V_PER_A - FM_LAUNCH_CURRENT_KP_V_PER_A);
        ki_step = FM_LAUNCH_CURRENT_KI_STEP_V_PER_A +
                  gain_k * (FM_CURRENT_KI_STEP_V_PER_A - FM_LAUNCH_CURRENT_KI_STEP_V_PER_A);
    } else {
        kp = FM_CURRENT_KP_V_PER_A;
        ki_step = FM_CURRENT_KI_STEP_V_PER_A;
    }

    g.int_d += ki_step * ed;
    g.int_q += ki_step * eq;
    g.int_d = FM_Clamp(g.int_d, -vmax, vmax);
    g.int_q = FM_Clamp(g.int_q, -vmax, vmax);

    FM_DQ vdq;
    vdq.d = kp * ed + g.int_d;
    vdq.q = kp * eq + g.int_q;

    /* Limit in dq without paying for sqrtf on every unsaturated PWM sample. */
    float v2 = vdq.d * vdq.d + vdq.q * vdq.q;
    float vmax2 = vmax * vmax;
    float vmag = 0.0f;
    if (v2 > vmax2 && v2 > 1.0e-12f) {
        float mag = sqrtf(v2);
        float k = vmax / mag;
        vdq.d *= k;
        vdq.q *= k;

        /* Back-calculate only on true voltage saturation. */
        g.int_d = FM_Clamp(vdq.d - kp * ed, -vmax, vmax);
        g.int_q = FM_Clamp(vdq.q - kp * eq, -vmax, vmax);
        vmag = vmax;
    }

    FM_AlphaBeta vfund = invpark_sc(vdq, st, ct);
    FM_AlphaBeta vtotal = limit_ab(add_ab(vfund, v_hfi), vmax);

    g.last_v_fund = vfund;
    pwm_apply_ab(vtotal);

    g.t.id_a = idq.d;
    g.t.iq_a = idq.q;
    g.t.vd_cmd_v = vdq.d;
    g.t.vq_cmd_v = vdq.q;
    if (v2 > vmax2) g.t.vfund_mag_v = vmag;
}

/* Re-express the already-applied stationary alpha/beta voltage in a new D/Q
 * frame and seed the PI so the FIRST BEMF-only sample requests the same physical
 * stator-voltage vector.  This makes the HFI->BEMF authority transfer bumpless. */
static void seed_current_pi_for_frame(float theta, FM_AlphaBeta i_ctrl,
                                      float id_ref, float iq_ref,
                                      FM_AlphaBeta vfund_ab)
{
    float st, ct;
    FM_CordicSinCos(theta, &st, &ct);
    FM_DQ idq = park_sc(i_ctrl, st, ct);
    FM_DQ vdq = park_sc(vfund_ab, st, ct);
    float ed = id_ref - idq.d;
    float eq = iq_ref - idq.q;
    float vmax = FM_VOLTAGE_LIMIT_RATIO * g.vbus;

    /* current_control() integrates once before forming Vd/Vq, so compensate for
     * that first Ki step as well as the proportional term. */
    g.int_d = FM_Clamp(vdq.d -
                       (FM_LAUNCH_CURRENT_KP_V_PER_A + FM_LAUNCH_CURRENT_KI_STEP_V_PER_A) * ed,
                       -vmax, vmax);
    g.int_q = FM_Clamp(vdq.q -
                       (FM_LAUNCH_CURRENT_KP_V_PER_A + FM_LAUNCH_CURRENT_KI_STEP_V_PER_A) * eq,
                       -vmax, vmax);
}

static uint8_t observer_basic_usable(float *rpm_out)
{
    float rpm = rpm_from_omega(g.obs.omega_e);
    if (rpm_out != NULL) *rpm_out = rpm;
    return (g.obs.polarity_resolved &&
            fabsf(rpm) > FM_RUN_OBSERVER_MIN_RPM &&
            g.obs.bemf_mag >= FM_RUN_HEALTH_MIN_BEMF_V &&
            fabsf(g.obs.phase_err) <= FM_RUN_HEALTH_MAX_PHASE_ERR_RAD);
}

static void observer_recatch_reset(void)
{
    g.observer_recatch_samples = 0U;
    g.observer_recatch_residual_mean = 0.0f;
    g.observer_recatch_residual_m2 = 0.0f;
    g.observer_recatch_blending = 0U;
    g.observer_recatch_blend_count = 0U;
}

/* FOC -> six-step must never carry RUN holdover/re-catch history into the
 * next six-step observer acquisition.  This is intentionally narrower than a
 * full HFI-session reset: rotor angle/speed chosen by the caller are preserved,
 * while all stale RUN observer authority is discarded. */
static void clear_run_observer_history_for_sixstep(void)
{
    g.observer_holdover_active = 0U;
    g.observer_holdover_count = 0U;
    g.last_good_observer_valid = 0U;
    g.last_good_observer_theta = 0.0f;
    g.last_good_observer_omega_e = 0.0f;
    g.observer_unlock_count = 0U;
    g.run_low_speed_confirm_count = 0U;
    g.run_entry_grace_count = 0U;
    g.neg_run_release_count = 0U;
    g.neg_run_release_k = 0.0f;
    g.neg_run_target_ramp = FM_HYBRID_NEG_RUN_SAFE_RAMP_RPM_PER_S;
    g.neg_run_false_lock_count = 0U;
    g.neg_run_false_lock_active = 0U;
    g.neg_run_admission_ready = 0U;
    g.neg_run_quality_bits = 0U;
    g.neg_run_iq_limit_a = FM_HYBRID_NEG_RUN_SAFE_IQ_A;
    g.feedback_good_count = 0U;
    g.forced_observer_good_count = 0U;
    g.state6_entry_good_count = 0U;
    g.state6_entry_blend = 0.0f;
    g.full_release_qualified = 0U;
    g.control_feedback_blend = 0.0f;
    g.forced_observer_blend = 0.0f;
    g.handover_observer_blend_start = 0.0f;
    g.speed_command_pending = 0U;
    observer_recatch_reset();
}

/* Preserve the physical alpha/beta voltage represented by the current-PI
 * integrators when the control frame is deliberately rotated during observer
 * re-capture.  This is the RUN equivalent of the bumpless startup PI seed: the
 * controller does not get a discontinuous integral-voltage vector just because
 * theta authority is being blended. */
static void rotate_current_pi_frame(float theta_old, float theta_new)
{
    float dtheta = FM_WrapPi(theta_new - theta_old);
    if (fabsf(dtheta) < 1.0e-6f)
        return;

    float sn, cs;
    FM_CordicSinCos(dtheta, &sn, &cs);
    float old_d = g.int_d;
    float old_q = g.int_q;
    g.int_d = cs * old_d + sn * old_q;
    g.int_q = -sn * old_d + cs * old_q;
}

/* Re-express a D/Q current reference in a new Park frame while preserving
 * the same stationary alpha/beta current vector.  Kept as a noinline helper
 * so the extra scratch variables exist only on the RUN/rejoin call path, not
 * in the always-hot ADC callback stack frame used by HFI startup. */
__attribute__((noinline))
static void rotate_run_current_ref_frame(float theta_old, float theta_new,
                                         float *id_ref, float *iq_ref)
{
    float dtheta = FM_WrapPi(theta_new - theta_old);
    if (fabsf(dtheta) < 1.0e-6f)
        return;

    float sn, cs;
    FM_CordicSinCos(dtheta, &sn, &cs);
    float old_d = *id_ref;
    float old_q = *iq_ref;
    *id_ref = cs * old_d + sn * old_q;
    *iq_ref = -sn * old_d + cs * old_q;
}

/* During holdover, qualify the independent PM-flux angle against the angle
 * trajectory propagated from the last trustworthy observer state.  The offset
 * itself is allowed to be large (the best 2026-08-26 run recovered ~133 deg
 * away), but it must be internally coherent for a short multi-sample window.
 * This avoids both failure modes of the old code: (1) one noisy sample cannot
 * recapture the motor, and (2) a recovered observer is not permanently locked
 * out merely because it did not land within a fixed 0.60-rad gate. */
static uint8_t observer_recatch_window_step(float held_theta, float held_rpm)
{
    float obs_rpm = rpm_from_omega(g.obs.omega_e);
    float e2 = g.obs.e_alpha * g.obs.e_alpha + g.obs.e_beta * g.obs.e_beta;
    float recatch_min2 = FM_OBSERVER_RECATCH_MIN_BEMF_V *
                         FM_OBSERVER_RECATCH_MIN_BEMF_V;
    if (!g.obs.polarity_resolved || !g.obs.theta_meas_valid ||
        !isfinite(e2) || e2 < recatch_min2 ||
        !isfinite(obs_rpm) || fabsf(obs_rpm) < FM_OBSERVER_RECATCH_MIN_RPM ||
        !isfinite(held_rpm) || obs_rpm * held_rpm <= 0.0f ||
        fabsf(obs_rpm - held_rpm) > FM_OBSERVER_RECATCH_MAX_RPM_DELTA) {
        g.observer_recatch_samples = 0U;
        g.observer_recatch_residual_mean = 0.0f;
        g.observer_recatch_residual_m2 = 0.0f;
        return 0U;
    }

    float raw_theta = g.obs.theta_meas_raw;
    if (g.obs.polarity_pi)
        raw_theta = FM_WrapPi(raw_theta + (float)M_PI);

    float residual = FM_WrapPi(raw_theta - held_theta);
    if (!isfinite(residual) ||
        fabsf(residual) > FM_OBSERVER_RECATCH_MAX_OFFSET_RAD) {
        g.observer_recatch_samples = 0U;
        g.observer_recatch_residual_mean = 0.0f;
        g.observer_recatch_residual_m2 = 0.0f;
        return 0U;
    }

    if (g.observer_recatch_samples == 0U) {
        g.observer_recatch_samples = 1U;
        g.observer_recatch_residual_mean = residual;
        g.observer_recatch_residual_m2 = 0.0f;
        return 0U;
    }

    float innovation = FM_WrapPi(residual - g.observer_recatch_residual_mean);
    if (fabsf(innovation) > FM_OBSERVER_RECATCH_MAX_INNOVATION_RAD) {
        /* Restart from the current sample instead of poisoning the next window. */
        g.observer_recatch_samples = 1U;
        g.observer_recatch_residual_mean = residual;
        g.observer_recatch_residual_m2 = 0.0f;
        return 0U;
    }

    ++g.observer_recatch_samples;
    float n = (float)g.observer_recatch_samples;
    float old_mean = g.observer_recatch_residual_mean;
    g.observer_recatch_residual_mean = FM_WrapPi(old_mean + innovation / n);
    float innovation2 = FM_WrapPi(residual - g.observer_recatch_residual_mean);
    g.observer_recatch_residual_m2 += innovation * innovation2;

    if (g.observer_recatch_samples < FM_OBSERVER_RECATCH_SAMPLES)
        return 0U;

    float variance = g.observer_recatch_residual_m2 /
                     (float)(g.observer_recatch_samples - 1U);
    if (!isfinite(variance) || variance > FM_OBSERVER_RECATCH_MAX_VARIANCE_RAD2) {
        g.observer_recatch_samples = 0U;
        g.observer_recatch_residual_mean = 0.0f;
        g.observer_recatch_residual_m2 = 0.0f;
        return 0U;
    }

    /* Re-anchor the observer PLL to the coherent PM-flux branch, but do NOT
     * jump the FOC frame there.  The control frame is blended below. */
    g.obs.theta = FM_WrapPi(held_theta + g.observer_recatch_residual_mean);
    float held_omega = g.last_good_observer_omega_e;
    float omega_window = omega_from_rpm(FM_OBSERVER_RECATCH_MAX_RPM_DELTA);
    g.obs.omega_e = FM_Clamp(g.obs.omega_e,
                             held_omega - omega_window,
                             held_omega + omega_window);
    g.obs.phase_err = 0.0f;
    g.observer_recatch_blending = 1U;
    g.observer_recatch_blend_count = 0U;
    return 1U;
}

/* Choose the real observer state when trustworthy.  On a brief dropout, keep
 * propagating the last trustworthy REAL angle/speed, but allow a coherent raw
 * PM-flux angle to re-capture authority through a short blend.  This is designed
 * for a drone ESC: no blind 133-degree jump, no fixed 34-degree re-entry trap,
 * and no cold current-PI restart. */
static uint8_t real_angle_with_holdover(float *theta_out, float *rpm_out)
{
    float obs_rpm = rpm_from_omega(g.obs.omega_e);
    uint8_t usable = observer_basic_usable(NULL);

    if (usable && g.last_good_observer_valid && !g.observer_holdover_active) {
        float prev_rpm = rpm_from_omega(g.last_good_observer_omega_e);
        if (fabsf(obs_rpm - prev_rpm) > FM_OBSERVER_HOLDOVER_MAX_RPM_JUMP) {
            usable = 0U;
        } else {
            /* One-sample angle sanity guard.  Predict where the last trusted
             * rotor angle should be one control tick later and reject any
             * observer sample that jumps too far from that trajectory.
             * Critically, this check happens BEFORE g.obs.theta can become
             * control_theta, so a bad observer sample goes to holdover rather
             * than rotating the Park frame and throwing Iq into Id. */
            float predicted_theta = FM_WrapPi(g.last_good_observer_theta +
                                               g.last_good_observer_omega_e * FM_CONTROL_TS_S);
            float angle_innovation = FM_WrapPi(g.obs.theta - predicted_theta);
            if (!isfinite(angle_innovation) ||
                fabsf(angle_innovation) > FM_RUN_MAX_OBSERVER_ANGLE_INNOVATION_RAD)
                usable = 0U;
        }
    }

    if (usable && !g.observer_holdover_active) {
        g.last_good_observer_theta = g.obs.theta;
        g.last_good_observer_omega_e = g.obs.omega_e;
        g.last_good_observer_valid = 1U;
        g.observer_holdover_count = 0U;
        g.observer_holdover_active = 0U;
        observer_recatch_reset();
        g.control_feedback_blend = 1.0f;
        *theta_out = g.obs.theta;
        *rpm_out = obs_rpm;
        return 1U;
    }

    /* If the observer has recovered while RUN is already in holdover, start
     * authority return immediately from ONE healthy sample -- but do not jump
     * the Park frame directly to the observer.  The previous build could jump
     * ~80 electrical degrees in one ISR even though the observer's own phase
     * residual was tiny.  That rotated Iq into Id and hit the current-sense
     * rail.
     *
     * Instead, this single sample starts a very short control-frame slew.  The
     * existing recatch blend machinery then advances the propagated angle and
     * moves it toward the observer over FM_OBSERVER_DIRECT_REJOIN_SLEW_SAMPLES.
     * There is NO extra coherence/sample qualification before starting it. */
    float held_dir_rpm = rpm_from_omega(g.last_good_observer_omega_e);
    if (g.observer_holdover_active && !g.observer_recatch_blending &&
        g.obs.polarity_resolved && g.obs.theta_meas_valid &&
        isfinite(obs_rpm) && fabsf(obs_rpm) >= FM_OBSERVER_DIRECT_REJOIN_MIN_RPM &&
        g.last_good_observer_valid && isfinite(held_dir_rpm) &&
        obs_rpm * held_dir_rpm > 0.0f &&
        isfinite(g.obs.bemf_mag) &&
        g.obs.bemf_mag >= FM_OBSERVER_DIRECT_REJOIN_MIN_BEMF_V &&
        isfinite(g.obs.phase_err) &&
        fabsf(g.obs.phase_err) <= FM_OBSERVER_DIRECT_REJOIN_MAX_PHASE_ERR_RAD) {
        g.observer_recatch_samples = 0U;
        g.observer_recatch_residual_mean = 0.0f;
        g.observer_recatch_residual_m2 = 0.0f;
        g.observer_recatch_blending = 1U;
        g.observer_recatch_blend_count = 0U;
        /* Keep holdover ACTIVE until the slew is complete.  This is critical:
         * last_good_observer_theta remains the continuously propagated control
         * trajectory instead of being overwritten by the recovered observer. */
    }

    if (!g.last_good_observer_valid)
        return 0U;

    /* FM_OBSERVER_HOLDOVER_SAMPLES is the SEARCH deadline only.  Once a
     * coherent PM-flux recapture has started, do not let the original
     * holdover timer kill the motor halfway through the bumpless blend.
     * The blend has its own bounded sample count below. */
    if (!g.observer_recatch_blending &&
        g.observer_holdover_count >= FM_OBSERVER_HOLDOVER_SAMPLES)
        return 0U;

    /* Propagate the last trusted rotor trajectory every holdover sample. */
    g.last_good_observer_theta = FM_WrapPi(g.last_good_observer_theta +
                                           g.last_good_observer_omega_e * FM_CONTROL_TS_S);
    ++g.observer_holdover_count;
    g.observer_holdover_active = 1U;

    float held_theta = g.last_good_observer_theta;
    float held_rpm = rpm_from_omega(g.last_good_observer_omega_e);

    if (!g.observer_recatch_blending)
        (void)observer_recatch_window_step(held_theta, held_rpm);

    if (g.observer_recatch_blending) {
        /* The recapture blend owns a separate bounded deadline.  Normally it
         * completes at FM_OBSERVER_RECATCH_BLEND_SAMPLES; this extra ceiling
         * prevents any future code change from leaving RUN in blend forever. */
        if (g.observer_recatch_blend_count >= FM_OBSERVER_RECATCH_BLEND_TIMEOUT_SAMPLES)
            return 0U;

        /* If BEMF disappears completely during the blend, fall back to the
         * propagated trajectory and rebuild a fresh coherence window. */
        float e2_now = g.obs.e_alpha * g.obs.e_alpha + g.obs.e_beta * g.obs.e_beta;
        float blend_min_v = 0.5f * FM_OBSERVER_RECATCH_MIN_BEMF_V;
        if (!g.obs.theta_meas_valid || !isfinite(e2_now) ||
            e2_now < blend_min_v * blend_min_v ||
            !isfinite(obs_rpm) || fabsf(obs_rpm) < FM_OBSERVER_RECATCH_MIN_RPM ||
            obs_rpm * held_rpm <= 0.0f) {
            g.observer_recatch_blending = 0U;
            g.observer_recatch_blend_count = 0U;
            g.observer_recatch_samples = 0U;
            g.observer_recatch_residual_mean = 0.0f;
            g.observer_recatch_residual_m2 = 0.0f;
            g.control_feedback_blend = 0.0f;
            *theta_out = held_theta;
            *rpm_out = held_rpm;
            return 1U;
        }

        ++g.observer_recatch_blend_count;
        float k = FM_Clamp((float)g.observer_recatch_blend_count /
                           (float)FM_OBSERVER_DIRECT_REJOIN_SLEW_SAMPLES,
                           0.0f, 1.0f);
        *theta_out = blend_angle(held_theta, g.obs.theta, k);
        *rpm_out = (1.0f - k) * held_rpm + k * obs_rpm;
        g.control_feedback_blend = k;

        if (g.observer_recatch_blend_count >= FM_OBSERVER_DIRECT_REJOIN_SLEW_SAMPLES) {
            g.last_good_observer_theta = g.obs.theta;
            g.last_good_observer_omega_e = g.obs.omega_e;
            g.last_good_observer_valid = 1U;
            g.observer_holdover_count = 0U;
            g.observer_holdover_active = 0U;
            observer_recatch_reset();
            g.control_feedback_blend = 1.0f;
            *theta_out = g.obs.theta;
            *rpm_out = obs_rpm;
        }
        return 1U;
    }

    g.control_feedback_blend = 0.0f;
    *theta_out = held_theta;
    *rpm_out = held_rpm;
    return 1U;
}


static void start_hfi_converge(void)
{
    uint8_t best = 0U;
    float best_score = g.hfi.scan_score[0];
    for (uint8_t k = 1U; k < FM_HFI_SCAN_ANGLE_COUNT; ++k) {
        if (g.hfi.scan_score[k] > best_score) {
            best_score = g.hfi.scan_score[k];
            best = k;
        }
    }

    g.hfi.saliency_scale_a = best_score;
    g.t.hfi_saliency_a = best_score;
    if (best_score < FM_HFI_MIN_SALIENCY_A) {
        FastMotor_Stop(30U);
        return;
    }

    /* Seed the tracker from the strongest of eight aligned scan axes. */
    g.hfi.theta = hfi_scan_angle(best);
    g.hfi.omega_e = 0.0f;
    g.hfi.omega_filt = 0.0f;
    g.hfi.demod_lpf = 0.0f;
    g.hfi.error_a = 0.0f;
    g.hfi.converge_count = 0U;
    g.hfi.carrier_phase = 0.0f;
    g.hfi.carrier_index = 0U;
    g.hfi.polarity_probe_started = 0U;
    g.hfi.polarity_decision = 0U;
    g.hfi.polarity_axis_retry = 0U;
    g.hfi.polarity_sample_count = 0U;
    g.hfi.polarity_unwrapped_delta = 0.0f;
    g.hfi.polarity_last_theta = hfi_foc_theta();
    g.t.hfi_converge_count = 0U;
    set_state(FM_STATE_HFI_CONVERGE);
}

static inline uint8_t hfi_polarity_is_proven(void)
{
    /* 1/2 = standstill probe resolved; 6/7 = real-motion BEMF resolved.
     * 4 means still provisional and therefore must retain conservative
     * negative six-step catch qualification. */
    return (g.hfi.polarity_decision == 1U ||
            g.hfi.polarity_decision == 2U ||
            g.hfi.polarity_decision == 6U ||
            g.hfi.polarity_decision == 7U);
}

/* Resolve the pi ambiguity of saliency HFI without an alignment hold.
 *
 * A saliency scan cannot distinguish theta from theta+pi.  However, a positive
 * q-axis torque command has opposite PHYSICAL torque on those two branches.
 * Keep HFI active, apply a tiny regulated +Iq pulse, and unwrap the measured
 * HFI/FOC-axis displacement through a short HFI-only coast:
 *
 *   +delta theta  -> candidate polarity is already correct
 *   -delta theta  -> candidate is pi-wrong, add pi to the HFI->FOC mapping
 *
 * Return 0 while probing, +1 when resolved, -1 if both probes are ambiguous. */
static int hfi_polarity_probe_step(FM_AlphaBeta i_raw, FM_AlphaBeta vhf)
{
    const uint32_t settle_end = FM_HFI_POLARITY_SETTLE_SAMPLES;
    const uint32_t pulse_end  = settle_end + FM_HFI_POLARITY_PULSE_SAMPLES;
    const uint32_t coast_end  = pulse_end + FM_HFI_POLARITY_COAST_SAMPLES;

    if (!g.hfi.polarity_probe_started) {
        g.hfi.polarity_probe_started = 1U;
        g.hfi.polarity_sample_count = 0U;
        g.hfi.polarity_unwrapped_delta = 0.0f;
        g.hfi.polarity_last_theta = hfi_foc_theta();
        g.hfi.polarity_decision = 0U;
        g.int_d = 0.0f;
        g.int_q = 0.0f;
    }

    uint32_t n = g.hfi.polarity_sample_count;
    float theta_now = hfi_foc_theta();

    if (n < settle_end) {
        /* Establish a quiet local angle baseline. */
        g.hfi.polarity_last_theta = theta_now;
        g.hfi.polarity_unwrapped_delta = 0.0f;
        g.t.polarity_probe_iq_a = 0.0f;
        g.t.iq_ref_a = 0.0f;
        g.last_v_fund.alpha = 0.0f;
        g.last_v_fund.beta = 0.0f;
        pwm_apply_ab(limit_ab(vhf, FM_VOLTAGE_LIMIT_RATIO * g.vbus));
    } else {
        /* HFI angle is locally continuous even though the global saliency
         * solution is pi-periodic, so a wrapped one-step difference gives an
         * unambiguous displacement sign for this very small motion. */
        float dtheta = FM_WrapPi(theta_now - g.hfi.polarity_last_theta);
        g.hfi.polarity_unwrapped_delta += dtheta;
        g.hfi.polarity_last_theta = theta_now;

        if (n < pulse_end) {
            float probe_iq = (g.hfi.polarity_axis_retry == 0U) ?
                             FM_HFI_POLARITY_PROBE_IQ_A :
                             FM_HFI_POLARITY_RETRY_IQ_A;
            g.t.polarity_probe_iq_a = probe_iq;
            g.t.iq_ref_a = probe_iq;

            /* Carrier-notched current is the fundamental feedback; keep the
             * HFI voltage riding on top so the position tracker never goes
             * blind during the torque pulse. */
            current_control(theta_now, g.i_lf, 0.0f, probe_iq, vhf);
        } else {
            if (n == pulse_end) {
                /* Remove stored q-axis voltage before the coast proof. */
                g.int_d = 0.0f;
                g.int_q = 0.0f;
            }
            g.t.polarity_probe_iq_a = 0.0f;
            g.t.iq_ref_a = 0.0f;
            g.last_v_fund.alpha = 0.0f;
            g.last_v_fund.beta = 0.0f;
            pwm_apply_ab(limit_ab(vhf, FM_VOLTAGE_LIMIT_RATIO * g.vbus));
        }
    }

    g.t.polarity_delta_theta_rad = g.hfi.polarity_unwrapped_delta;
    g.t.polarity_decision = g.hfi.polarity_decision;

    if (g.hfi.polarity_sample_count < 0xFFFFFFFFU)
        ++g.hfi.polarity_sample_count;

    if (g.hfi.polarity_sample_count < coast_end)
        return 0;

    float delta = g.hfi.polarity_unwrapped_delta;

    if (delta >= FM_HFI_POLARITY_MIN_DELTA_RAD) {
        g.hfi.polarity_decision = 1U; /* direct branch */
    } else if (delta <= -FM_HFI_POLARITY_MIN_DELTA_RAD) {
        g.hfi.polarity_decision = 2U; /* theta + pi */
        g.hfi_to_foc_offset =
            FM_WrapPi(g.hfi_to_foc_offset + (float)M_PI);
        g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
    } else if (g.hfi.polarity_axis_retry == 0U) {
        /* Ambiguous tiny motion: retry once, stronger, from the newly settled
         * rotor position.  Do not guess a polarity branch. */
        g.hfi.polarity_axis_retry = 1U;
        g.hfi.polarity_decision = 3U;
        g.hfi.polarity_sample_count = 0U;
        g.hfi.polarity_unwrapped_delta = 0.0f;
        g.hfi.polarity_last_theta = hfi_foc_theta();
        g.int_d = 0.0f;
        g.int_q = 0.0f;
        g.t.polarity_probe_iq_a = 0.0f;
        g.t.polarity_decision = 3U;
        return 0;
    } else {
        /* The rotor did not move far enough for the tiny standstill torque
         * pulse to distinguish theta from theta+pi.  This is NOT a motor
         * fault.  Keep the current HFI branch provisionally and let the first
         * trustworthy signed PM-flux/BEMF motion resolve it in six-step.
         *
         * This specifically removes the build-26090323 failure where a valid
         * HFI axis was followed by fault 32 solely because static friction / a
         * quiet rotor made both displacement probes inconclusive. */
        g.hfi.polarity_decision = 4U; /* deferred to real motion */
        g.t.polarity_decision = 4U;
        g.t.polarity_probe_iq_a = 0.0f;
        g.t.iq_ref_a = 0.0f;
        g.int_d = 0.0f;
        g.int_q = 0.0f;
        g.hfi.omega_e = 0.0f;
        g.hfi.omega_filt = 0.0f;
        return 1;
    }

    g.t.polarity_decision = g.hfi.polarity_decision;
    g.t.polarity_probe_iq_a = 0.0f;
    g.t.iq_ref_a = 0.0f;
    g.int_d = 0.0f;
    g.int_q = 0.0f;

    /* The pulse may have left a small HFI speed estimate.  Six-step only needs
     * the resolved instantaneous rotor angle, so do not seed it with probe
     * motion as if it were a real launch-speed measurement. */
    g.hfi.omega_e = 0.0f;
    g.hfi.omega_filt = 0.0f;
    return 1;
}

/* Start a new standstill acquisition with the same DYNAMIC controller state as
 * a cold boot, while preserving hardware calibration, measured VBUS and the
 * user RPM command.  Sequential tests used to carry last-good observer state,
 * six-step sync counters and FOC retry cooldown into the next HFI scan. */
static void reset_control_session_for_hfi(FM_AlphaBeta i_raw)
{
    float requested_rpm = g.t.target_rpm;
    float vbus = g.vbus;
    uint8_t vbus_has_measurement = g.vbus_has_measurement;
    uint16_t vbus_adc_counts = g.t.vbus_adc_counts;
    uint8_t vbus_adc_valid = g.t.vbus_adc_valid;
    uint16_t off_u = g.t.offset_iu_counts;
    uint16_t off_v = g.t.offset_iv_counts;
    uint16_t off_w = g.t.offset_iw_counts;
    uint8_t cordic_ok = g.t.cordic_selftest_ok;
    float cordic_s = g.t.cordic_test_sin45;
    float cordic_c = g.t.cordic_test_cos45;

    /* Preserve cumulative timing counters only for diagnostics; they have no
     * control authority.  Reset the previous-entry latch so the first HFI ISR
     * cannot manufacture a late interval across the restart boundary. */
    uint32_t timing_max = g.isr_cycles_max;
    uint32_t timing_overruns = g.isr_overrun_count;
    uint32_t timing_interval_max = g.isr_interval_cycles_max;
    uint32_t timing_late = g.isr_late_count;

    memset(&g, 0, sizeof(g));

    g.vbus = vbus;
    g.vbus_has_measurement = vbus_has_measurement;
    g.hfi_to_foc_offset = FM_HFI_RUNTIME_FOC_OFFSET_RAD;
    g.hfi_angle_sign = FM_HFI_RUNTIME_ANGLE_SIGN;
    g.t.target_rpm = requested_rpm;
    g.t.vbus_v = vbus;
    g.t.vbus_adc_counts = vbus_adc_counts;
    g.t.vbus_adc_valid = vbus_adc_valid;
    g.t.offset_iu_counts = off_u;
    g.t.offset_iv_counts = off_v;
    g.t.offset_iw_counts = off_w;
    g.t.cordic_selftest_ok = cordic_ok;
    g.t.cordic_test_sin45 = cordic_s;
    g.t.cordic_test_cos45 = cordic_c;
    g.t.hfi_angle_sign = g.hfi_angle_sign;
    g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
    g.t.firmware_build_id = FM_FIRMWARE_BUILD_ID;
    /* HFI restarts clear FM_App. Restore mailbox CCR normalization and the
     * full-original reference identity without calling expf in the ISR. */
    g.v13_inv_arr = 1.0f / (float)htim1.Init.Period;
    g.t.v13_reference_complete = V13_REFERENCE_COMPLETE;
    g.t.v13_shadow_decimation = s_v13_fast_disabled ? FM_V13_SHADOW_DECIMATION : FM_V13_FAST_DIV;
    g.t.v13_fast_mode = s_v13_fast_disabled ? 2U : 0U;
    g.t.v13_fast_overruns = s_v13_fast_overruns;

    g.isr_cycles_max = timing_max;
    g.isr_overrun_count = timing_overruns;
    g.isr_interval_cycles_max = timing_interval_max;
    g.isr_late_count = timing_late;
    g.isr_prev_entry_valid = 0U;

    g.i_lf = i_raw;
    reset_notch_to_steady_input(&g.notch_alpha, i_raw.alpha);
    reset_notch_to_steady_input(&g.notch_beta, i_raw.beta);
    g.last_vbus_update_ms = HAL_GetTick();
}

static void begin_fresh_hfi_start(FM_AlphaBeta i_raw)
{
    FM_AlphaBeta zero = {0.0f, 0.0f};
    float requested_rpm = g.t.target_rpm;
    reset_control_session_for_hfi(i_raw);
    g.t.target_rpm = requested_rpm;

    FM_PWM_EnableAllPhaseOutputs();
    pwm_apply_ab(zero);
    FM_PWM_Enable();

    g.hfi.scan_index = 0U;
    g.hfi.carrier_phase = 0.0f;
    g.hfi.carrier_index = 0U;
    g.t.active_estimator = FM_ESTIMATOR_HFI;
    g.t.launch_phase = 0U;
    set_state(FM_STATE_HFI_SCAN);
}

void FastMotor_Init(void)
{
    memset(&g, 0, sizeof(g));

    /* Enable the Cortex-M4 cycle counter before the 40 kHz injected ADC ISR is
     * started.  This has no interrupt traffic and lets telemetry prove whether
     * the control callback ever exceeds its 25 us budget. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_isr_budget_cycles = SystemCoreClock / 40000U;
    if (s_isr_budget_cycles == 0U) s_isr_budget_cycles = 1U;

    g.handover_observer_blend_start = 0.0f;
    g.vbus = FM_VBUS_NOMINAL_V;
    g.hfi_to_foc_offset = FM_HFI_RUNTIME_FOC_OFFSET_RAD;
    g.hfi_angle_sign = FM_HFI_RUNTIME_ANGLE_SIGN;
    g.t.hfi_angle_sign = g.hfi_angle_sign;
    g.t.align_expected_angle_rad = 0.0f;
    g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
    g.t.vbus_v = g.vbus;
    g.t.vbus_adc_valid = 0U;
    g.t.target_rpm = FM_TARGET_RPM;
    g.t.active_estimator = FM_ESTIMATOR_NONE;
    g.t.state = FM_STATE_BOOT;
    /* Set the build signature before any runtime protection can fire.
     * Build 26091707 only refreshed this from the 1-kHz ISR mirror, so an
     * immediate startup fault misleadingly logged firmware_build_id = 0. */
    g.t.firmware_build_id = FM_FIRMWARE_BUILD_ID;

    /* New filter runs only alongside the original feedback and never controls
     * current PI, rotor estimation, PWM or fault protection. */
    V13_Init(&s_v13_state, FM_PWM_HZ / (float)FM_V13_SHADOW_DECIMATION);
#if FM_V13_RESEARCH_NOEVENT
    V13_Init(&s_v13_noevent_state, FM_PWM_HZ / (float)FM_V13_SHADOW_DECIMATION);
#endif
    s_v13_40k_cached_gain = V13_Alpha40Lut[6U];
    s_v13_40k_gain_sample_tick = 0U;
    s_v13_40k_initialized = 0U;
    s_v13_40k_updates = 0U;
    s_v13_40k_cycles_max = 0U;
    s_v13_40k_guard_off = 0U;
    s_v13_40k_overbudget_consecutive = 0U;
    s_v13_40k_guard_events = 0U;
    V13_Init(&s_v13_fast_state, FM_PWM_HZ / (float)FM_V13_FAST_DIV);
    s_v13_fast_updates = 0U;
    s_v13_fast_run_updates = 0U;
    s_v13_fast_run_start_tick = 0xFFFFFFFFU;
    s_v13_fast_cycles_max = 0U;
    s_v13_fast_overruns = 0U;
    s_v13_fast_this_isr = 0U;
    s_v13_fast_pair_valid = 0U;
    s_v13_run_observer_cycles_last = 0U;
    s_v13_run_observer_cycles_max = 0U;
    s_run20k_session = 0xFFFFFFFFU;
    s_run20k_v_accum.alpha = 0.0f;
    s_run20k_v_accum.beta = 0.0f;
    s_run20k_accum_count = 0U;
    s_run20k_correction_due = 0U;
    s_run20k_updates = 0U;
    s_run20k_predicts = 0U;
    s_run20k_last_observer_tick = 0U;
    g.t.observer20k_mode = 0U;
    g.t.observer20k_updates = 0U;
    g.t.observer20k_predicts = 0U;

    s_v13_fast_disabled = 0U;
    s_v13_ring_head = s_v13_ring_tail = s_v13_ring_dropped = 0U;
    s_v13_bg_session = 0xFFFFFFFFU;
    s_v13_bg_last_tick = 0U;
    s_v13_bg_gap_count = 0U;
    s_v13_session_resets = 0U;
    s_v13_gap_invalid = 0U;
    s_v13_capture_rearm = 0U;
    memset(&g_v13_ram_capture, 0, sizeof(g_v13_ram_capture));
    g.t.v13_fast_mode = 0U;
    g.v13_inv_arr = 1.0f / (float)htim1.Init.Period;
    g.t.v13_reference_complete = V13_REFERENCE_COMPLETE;
    g.t.v13_shadow_decimation = FM_V13_SHADOW_DECIMATION;

    FM_CordicInit();
    FM_PWM_Init();
    FM_PWM_Disable();

    /* Validate the exact PHASE -> COS/SIN sequence used by HFI + FOC before
     * any bridge output is enabled.  A failure here means Park/InvPark cannot
     * be trusted, so do not energize the motor. */
    g.t.cordic_selftest_ok = FM_CordicSelfTest(&g.t.cordic_test_sin45,
                                                &g.t.cordic_test_cos45);
    if (!g.t.cordic_selftest_ok) { FastMotor_Stop(41U); return; }

    if (STSPIN_Init() != HAL_OK) { FastMotor_Stop(1U); return; }
    if (STSPIN_ClearFaults() != HAL_OK) { FastMotor_Stop(2U); return; }

    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp2) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
        FastMotor_Stop(3U); return;
    }
    HAL_Delay(10U);

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        FastMotor_Stop(4U); return;
    }
    HAL_Delay(5U);

    /* Establish a real bus-voltage value before PWM is enabled.  If PF0 is
     * temporarily invalid, retain the 13 V nominal fallback and expose
     * vbus_adc_valid=0 in telemetry instead of fabricating an ADC reading. */
    {
        float sum_v = 0.0f;
        uint16_t valid = 0U;
        uint16_t last_counts = 0U;
        for (uint16_t n = 0U; n < FM_VBUS_STARTUP_SAMPLES; ++n) {
            float v = 0.0f;
            uint16_t counts = 0U;
            if (read_vbus_once(&v, &counts)) {
                sum_v += v;
                ++valid;
                last_counts = counts;
            }
        }
        if (valid > 0U) {
            g.vbus = sum_v / (float)valid;
            g.vbus_has_measurement = 1U;
            g.t.vbus_adc_valid = 1U;
            g.t.vbus_adc_counts = last_counts;
        }
        g.t.vbus_v = g.vbus;
    }

    /* Calibrate offsets through the exact ADC2 injected path used at runtime.
     * TIM1/CH4 is already running from FM_PWM_Init(), so TRGO continues to
     * trigger injected conversions while MOE keeps the bridge outputs disabled. */
    FM_CurrentSenseBeginInjectedCalibration(FM_OFFSET_MEDIAN_SAMPLES);

    if (HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK) {
        FastMotor_Stop(6U); return;
    }

    HAL_StatusTypeDef cal =
        FM_CurrentSenseWaitInjectedCalibration(FM_OFFSET_CAL_TIMEOUT_MS);
    if (cal != HAL_OK) {
        FastMotor_Stop((cal == HAL_TIMEOUT) ? 52U : 51U);
        return;
    }

    g.t.offset_iu_counts = g_fm_adc_offset.iu;
    g.t.offset_iv_counts = g_fm_adc_offset.iv;
    g.t.offset_iw_counts = g_fm_adc_offset.iw;

    /* Seed the raw-current telemetry with the proven zero-current medians.
     * If a fault occurs before the first normal 1-kHz mirror, the capture now
     * still contains meaningful ADC values instead of three zeros. */
    g.t.raw_iu_counts = g_fm_adc_offset.iu;
    g.t.raw_iv_counts = g_fm_adc_offset.iv;
    g.t.raw_iw_counts = g_fm_adc_offset.iw;

    FM_PWM_Enable();
    set_state(FM_STATE_BEEP);
}

/* Only hard real-time motor-control path: one call per ADC2 injected triplet.
 * ADC1_2_IRQHandler captures irq_entry_cycles before clearing JEOS and calls
 * this directly, bypassing the generic HAL ADC dispatcher at 40 kHz. */
FM_HOT_ISR void FastMotor_ADC2InjectedIRQ(uint32_t isr_entry_cycles)
{
    fm_isr_timing_begin(isr_entry_cycles);

    FM_RawCurrents raw;
    raw.iu = (uint16_t)(ADC2->JDR1 & 0x0FFFU);
    raw.iv = (uint16_t)(ADC2->JDR2 & 0x0FFFU);
    raw.iw = (uint16_t)(ADC2->JDR3 & 0x0FFFU);

    /* During startup calibration this very same injected callback records the
     * offset samples.  No bridge output is enabled and no FOC code executes. */
    if (FM_CurrentSenseCalibrationActive()) {
        FM_CurrentSenseOnInjectedSample(&raw);
        fm_isr_timing_end(isr_entry_cycles);
        return;
    }

    if (g.t.state == FM_STATE_BOOT || g.t.state == FM_STATE_FAULT) {
        fm_isr_timing_end(isr_entry_cycles);
        return;
    }

    float iu = ((float)((int32_t)g_fm_adc_offset.iu - (int32_t)raw.iu)) * FM_CURRENT_A_PER_COUNT;
    float iv = ((float)((int32_t)g_fm_adc_offset.iv - (int32_t)raw.iv)) * FM_CURRENT_A_PER_COUNT;
    float iw = ((float)((int32_t)g_fm_adc_offset.iw - (int32_t)raw.iw)) * FM_CURRENT_A_PER_COUNT;
    float phase_current_sum = iu + iv + iw;

    /* Current protection uses the raw injected triplet, but no longer treats
     * one HFI-ripple sample as a sustained overcurrent.  The scale remains
     * the physical 5 mOhm / x5 / 12-bit chain. */
    float phase_peak = fabsf(iu);
    float av = fabsf(iv);
    float aw = fabsf(iw);
    if (av > phase_peak) phase_peak = av;
    if (aw > phase_peak) phase_peak = aw;

    /* ADC/opamp rail hit: current is no longer measurable reliably.
     *
     * IMPORTANT DIAGNOSTIC LATCH:
     * 26091707 moved the normal current telemetry mirror below the protection
     * checks to save ISR time.  That made an immediate fault-13 capture show
     * raw currents/peak current as zero even though a JDR had actually hit a
     * rail.  Only on the fault path, latch the exact triplet and the derived
     * current values before disabling PWM.  The FAULT state stops further ADC
     * processing, so this remains frozen for foreground telemetry. */
    if (raw.iu <= FM_ADC_RAIL_MARGIN_COUNTS || raw.iu >= (4095U - FM_ADC_RAIL_MARGIN_COUNTS) ||
        raw.iv <= FM_ADC_RAIL_MARGIN_COUNTS || raw.iv >= (4095U - FM_ADC_RAIL_MARGIN_COUNTS) ||
        raw.iw <= FM_ADC_RAIL_MARGIN_COUNTS || raw.iw >= (4095U - FM_ADC_RAIL_MARGIN_COUNTS)) {
        g.t.raw_iu_counts = raw.iu;
        g.t.raw_iv_counts = raw.iv;
        g.t.raw_iw_counts = raw.iw;
#if !FM_V13_SHADOW_ENABLED
        g.t.iu_a = iu;
        g.t.iv_a = iv;
        g.t.iw_a = iw;
#endif
        g.t.phase_current_sum_a = phase_current_sum;
        g.t.phase_current_peak_a = phase_peak;
        g.t.overcurrent_count = g.overcurrent_count;
        g.t.current_sum_bad_count = g.current_sum_bad_count;
        g.t.tim1_moe = (TIM1->BDTR & TIM_BDTR_MOE) ? 1U : 0U;
        g.t.tim1_ccr1 = (uint16_t)TIM1->CCR1;
        g.t.tim1_ccr2 = (uint16_t)TIM1->CCR2;
        g.t.tim1_ccr3 = (uint16_t)TIM1->CCR3;
        FastMotor_Stop(13U);
        return;
    }

    /* Catastrophic current threshold is debounced by two consecutive JEOS
     * triplets.  This rejects a single isolated sample such as the 28.5 A event
     * seen exactly at observer qualification, while ADC/opamp rail detection
     * above remains immediate and the 20 A sustained protection remains active. */
    if (phase_peak >= FM_PHASE_CURRENT_HARD_TRIP_A) {
        if (g.hard_overcurrent_count < FM_PHASE_CURRENT_HARD_TRIP_SAMPLES)
            ++g.hard_overcurrent_count;
    } else {
        g.hard_overcurrent_count = 0U;
    }
    if (g.hard_overcurrent_count >= FM_PHASE_CURRENT_HARD_TRIP_SAMPLES) {
        FastMotor_Stop(10U);
        return;
    }

    /* Sustained overcurrent must persist for multiple complete JEOS triplets.
     * Below threshold the counter leaks down instead of being reset by noise. */
    if (phase_peak >= FM_PHASE_CURRENT_TRIP_A) {
        if (g.overcurrent_count < FM_PHASE_CURRENT_TRIP_SAMPLES)
            ++g.overcurrent_count;
    } else if (g.overcurrent_count > 0U) {
        --g.overcurrent_count;
    }
    if (g.overcurrent_count >= FM_PHASE_CURRENT_TRIP_SAMPLES) {
        FastMotor_Stop(10U);
        return;
    }

    /* A valid three-phase instantaneous sample should sum near zero.  This
     * distinguishes a broken ADC triplet/offset from genuine motor current. */
    if (fabsf(phase_current_sum) > FM_CURRENT_SUM_MAX_A) {
        if (g.current_sum_bad_count < FM_CURRENT_SUM_BAD_SAMPLES)
            ++g.current_sum_bad_count;
    } else if (g.current_sum_bad_count > 0U) {
        --g.current_sum_bad_count;
    }
    if (g.current_sum_bad_count >= FM_CURRENT_SUM_BAD_SAMPLES) {
        FastMotor_Stop(12U);
        return;
    }

    FM_AlphaBeta i_raw = clarke(iu, iv);
    /* The 4-kHz carrier notch is only needed while HFI is actively estimating
     * standstill angle.  It used to burn two biquads on every 40-kHz sample in
     * six-step, handover and RUN even though no code consumed g.i_lf there. */
    if (g.t.state == FM_STATE_HFI_SCAN || g.t.state == FM_STATE_HFI_CONVERGE)
        update_hfi_current_feedback(i_raw);
    ++g.isr_count;
    /* 40kHz research EMA: short, bounded float-only work, never feedback. */
    fm_v13_cached40k_update(iu, iv, iw);
    /* Full frozen V13 SHADOW at the prediction-only half of raw 20kHz RUN. */
    fm_shadow20_update(iu, iv, iw);

    /* 4 kHz sample enqueue ONLY while RUN. This contains no filtering,
     * division, UART or wait. Actual motor control always sees RAW current.
     * Drop V13 diagnostic records on overflow; no ADC samples are skipped. */
#if FM_V13_SHADOW_ENABLED
    if (g.t.state == FM_STATE_RUN &&
        (g.isr_count % FM_V13_SHADOW_DECIMATION) == 0U) {
        uint32_t head = s_v13_ring_head;
        if ((uint32_t)(head - s_v13_ring_tail) < FM_V13_BG_RING_CAPACITY) {
            FM_V13_RingFrame *frame = &s_v13_ring[head & FM_V13_BG_RING_MASK];
            frame->tick = g.isr_count;
            frame->current[0] = iu;
            frame->current[1] = iv;
            frame->current[2] = iw;
            frame->ccr[0] = (uint16_t)TIM1->CCR1;
            frame->ccr[1] = (uint16_t)TIM1->CCR2;
            frame->ccr[2] = (uint16_t)TIM1->CCR3;
            frame->bus = g.vbus;
            frame->observer_theta = g.obs.theta_meas_raw;
            frame->control_theta = g.control_theta;
            frame->phase_error = g.obs.phase_err;
            frame->flux_omega_e = g.obs.flux_omega_e;
            frame->feedback_rpm = g.control_speed_feedback_rpm;
            frame->id_a = g.t.id_a;
            frame->iq_a = g.t.iq_a;
            frame->observer_flags = (g.observer_phase_valid ? 1U : 0U) |
                (g.obs.theta_meas_valid ? 2U : 0U) |
                (g.obs.locked ? 4U : 0U) |
                (g.observer_holdover_active ? 8U : 0U);
            const uint32_t outputs = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                     TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                     TIM_CCER_CC3E | TIM_CCER_CC3NE;
            frame->voltage_valid =
                (((TIM1->CCER & outputs) == outputs) &&
                 ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)) ? 1U : 0U;
            __DMB();
            s_v13_ring_head = head + 1U;
        } else {
            ++s_v13_ring_dropped;
        }
    }
#endif

    /* Commissioning telemetry is sampled at 1 kHz, not 40 kHz.  None of these
     * fields participates in control; removing the per-sample RAM traffic
     * keeps the hard ISR focused on protection/current/angle work. */
    if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U) {
        g.t.raw_iu_counts = raw.iu;
        g.t.raw_iv_counts = raw.iv;
        g.t.raw_iw_counts = raw.iw;
#if !FM_V13_SHADOW_ENABLED
        g.t.iu_a = iu;
        g.t.iv_a = iv;
        g.t.iw_a = iw;
#endif
        g.t.phase_current_sum_a = phase_current_sum;
        g.t.phase_current_peak_a = phase_peak;
        g.t.overcurrent_count = g.overcurrent_count;
        g.t.current_sum_bad_count = g.current_sum_bad_count;
        g.t.firmware_build_id = FM_FIRMWARE_BUILD_ID;
        g.t.bemf_v = (g.t.state == FM_STATE_SLINGSHOT) ?
                     g.trap_bemf_vector_mag_cached : g.obs.bemf_mag;
        g.t.phase_error_rad = g.obs.phase_err;
        g.t.observer_rpm = rpm_from_omega(g.obs.flux_omega_e);
        g.t.raw_flux_rpm = g.t.observer_rpm;
        g.t.observer_phase_valid = g.observer_phase_valid;
        g.t.sixstep_bemf_sync = g.trap_bemf_sync_active;
        g.t.sixstep_measured_rpm = g.trap_measured_rpm;
        /* Three 1-kHz stores are cheap and make the optimized ISR measurable
         * again; 26091705 accidentally left these runtime channels at zero. */
        g.t.isr_cycles_last = g.isr_cycles_last;
        g.t.isr_cycles_max = g.isr_cycles_max;
        g.t.isr_overrun_count = g.isr_overrun_count;
        if (g.t.state == FM_STATE_IDLE) {
            g.t.rpm = 0.0f;
        } else if (g.t.active_estimator == FM_ESTIMATOR_HFI) {
            g.t.rpm = hfi_foc_rpm();
        } else if (g.t.active_estimator == FM_ESTIMATOR_HFI_SEED) {
            g.t.rpm = g.t.sling_rpm;
        } else {
            g.t.rpm = g.control_speed_feedback_rpm;
        }
    }

    uint32_t state_elapsed_samples = g.isr_count - g.state_start_isr_count;
    FM_AlphaBeta zero = {0.0f, 0.0f};

    /* The old telemetry-only transient clears wrote ten fields every 25 us.
     * They are not streamed in the runtime profile and are set explicitly by
     * the states that use them, so do not burn ISR budget clearing them. */
#if 0
    /* Per-ISR startup diagnostics: clear transient gates so telemetry never
     * mistakes a stale state-4 value for a current decision. */
    g.t.trap_lock_confident = 0U;
    g.t.trap_locked_commutations = 0U;
    g.t.early_foc_window = 0U;
    g.t.early_foc_angle_error_rad = 0.0f;
    g.t.early_foc_speed_error_rpm = 0.0f;
    g.t.pos_slip_limit_active = 0U;
    g.t.pos_slip_limit_rpm = 0.0f;
    g.t.trap_flux_available = 0U;
    g.t.trap_bemf_theta_valid = 0U;
#endif

    switch (g.t.state) {
    case FM_STATE_BEEP: {
        uint32_t half = (uint32_t)(FM_PWM_HZ / (2.0f * FM_BEEP_HZ));
        if (half < 1U) half = 1U;
        float sign = ((g.isr_count / half) & 1U) ? 1.0f : -1.0f;
        FM_AlphaBeta v = {FM_BEEP_V * sign, 0.0f};
        g.last_v_fund = zero;
        pwm_apply_ab(v);
        if (state_elapsed_samples >= (FM_BEEP_MS * 40U)) {
            memset(&g.hfi, 0, sizeof(g.hfi));
            g.hfi_to_foc_offset = FM_HFI_RUNTIME_FOC_OFFSET_RAD;
            g.hfi_angle_sign = FM_HFI_RUNTIME_ANGLE_SIGN;
            g.t.hfi_angle_sign = g.hfi_angle_sign;
            g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
            g.t.align_expected_angle_rad = 0.0f;
            g.t.active_estimator = FM_ESTIMATOR_HFI;
            g.hfi.scan_index = 0U;
            g.hfi.carrier_phase = 0.0f;
    g.hfi.carrier_index = 0U;
            /* Remove the audible-beep fundamental from the carrier-notch
             * history before HFI saliency measurements begin. */
            reset_notch_to_steady_input(&g.notch_alpha, i_raw.alpha);
            reset_notch_to_steady_input(&g.notch_beta, i_raw.beta);
            g.i_lf = i_raw;
            g.t.launch_phase = 0U;
            set_state(FM_STATE_HFI_SCAN);
        }
        break;
    }

    case FM_STATE_HFI_SCAN: {
        const uint32_t inject_end = FM_HFI_SCAN_SETTLE_SAMPLES + FM_HFI_SCAN_MEASURE_SAMPLES;
        const uint32_t test_end = inject_end + FM_HFI_SCAN_IDLE_SAMPLES;
        g.last_v_fund = zero;
        g.t.iq_ref_a = 0.0f;
        g.t.launch_phase = 0U;

        if (g.hfi.scan_index >= FM_HFI_SCAN_ANGLE_COUNT) {
            start_hfi_converge();
            break;
        }

        if (g.hfi.scan_sample < inject_end) {
            static const float scan_sin[FM_HFI_SCAN_ANGLE_COUNT] = {
                0.0f, 0.3826834324f, 0.7071067812f, 0.9238795325f,
                1.0f, 0.9238795325f, 0.7071067812f, 0.3826834324f
            };
            static const float scan_cos[FM_HFI_SCAN_ANGLE_COUNT] = {
                1.0f, 0.9238795325f, 0.7071067812f, 0.3826834324f,
                0.0f,-0.3826834324f,-0.7071067812f,-0.9238795325f
            };
            float sh, ch;
            hfi_carrier(&sh, &ch);
            uint8_t scan_k = g.hfi.scan_index;
            float st = scan_sin[scan_k];
            float ct = scan_cos[scan_k];
            FM_AlphaBeta i_hf = hfi_carrier_current(i_raw);
            FM_DQ iq_test = park_sc(i_hf, st, ct);

            if (g.hfi.scan_sample >= FM_HFI_SCAN_SETTLE_SAMPLES)
                g.hfi.scan_accum += iq_test.q * ch;

            FM_DQ vhf_dq = {hfi_effective_voltage() * sh, 0.0f};
            FM_AlphaBeta vhf = invpark_sc(vhf_dq, st, ct);
            pwm_apply_ab(limit_ab(vhf, FM_VOLTAGE_LIMIT_RATIO * g.vbus));
        } else {
            pwm_apply_ab(zero);
        }

        ++g.hfi.scan_sample;
        if (g.hfi.scan_sample >= test_end) {
            g.hfi.scan_score[g.hfi.scan_index] =
                fabsf(g.hfi.scan_accum / (float)FM_HFI_SCAN_MEASURE_SAMPLES);
            ++g.hfi.scan_index;
            g.hfi.scan_sample = 0U;
            g.hfi.scan_accum = 0.0f;
            g.hfi.carrier_phase = 0.0f;
    g.hfi.carrier_index = 0U;
            if (g.hfi.scan_index >= FM_HFI_SCAN_ANGLE_COUNT)
                start_hfi_converge();
        }
        break;
    }

    case FM_STATE_HFI_CONVERGE: {
        /* Stationary, HFI-only angle acquisition.  No alignment field and no
         * synthetic angle exist in this state. */
        FM_AlphaBeta vhf = hfi_track_step(i_raw, 1.0f);
        g.last_v_fund = zero;
        pwm_apply_ab(limit_ab(vhf, FM_VOLTAGE_LIMIT_RATIO * g.vbus));
        g.t.active_estimator = FM_ESTIMATOR_HFI;
        g.t.launch_phase = 0U;
        g.t.iq_ref_a = 0.0f;
        g.control_theta = hfi_foc_theta();
        g.control_speed_feedback_rpm = hfi_foc_rpm();

        uint8_t converged = (state_elapsed_samples >= (FM_HFI_CONVERGE_MIN_MS * 40U) &&
                             fabsf(g.hfi.err_norm) <= FM_HFI_CONVERGE_ERR_NORM &&
                             fabsf(g.hfi.omega_e) <= FM_HFI_CONVERGE_OMEGA_RAD_S);
        if (converged) {
            if (g.hfi.converge_count < FM_HFI_CONVERGE_SAMPLES)
                ++g.hfi.converge_count;
        } else if (g.hfi.converge_count > FM_HFI_CONVERGE_BAD_DEC) {
            g.hfi.converge_count -= FM_HFI_CONVERGE_BAD_DEC;
        } else {
            g.hfi.converge_count = 0U;
        }
        g.t.hfi_converge_count = g.hfi.converge_count;

        if (g.hfi.polarity_probe_started ||
            g.hfi.converge_count >= FM_HFI_CONVERGE_SAMPLES) {
            int polarity_status = hfi_polarity_probe_step(i_raw, vhf);
            if (polarity_status < 0) {
                /* Defensive fallback: polarity ambiguity is never a fatal
                 * startup condition.  Real signed motion will resolve it. */
                g.hfi.polarity_decision = 4U;
                g.t.polarity_decision = 4U;
                polarity_status = 1;
            }
            if (polarity_status == 0)
                break;

            /* HFI now supplies a FULL electrical rotor angle, including the
             * theta-vs-theta+pi magnetic-polarity branch.  From here the
             * deterministic six-step launcher owns the bridge until the
             * dedicated trapezoid -> FOC handover speed is reached. */
            g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
            g.sling_hfi_seed_theta = hfi_foc_theta();
            float launch_dir = hybrid_sign(g.t.target_rpm);
            g.trap_sector = trap_sector_from_rotor_theta(g.sling_hfi_seed_theta,
                                                          launch_dir);
            /* HFI is a REAL rotor-position measurement.  It chooses the first
             * torque-producing hardware sector; no synthetic sector phase is
             * advanced after this point. */
            g.trap_sector_phase = trap_phase_from_rotor_theta_dir(
                                      g.trap_sector,
                                      g.sling_hfi_seed_theta,
                                      launch_dir);
            /* The first sector is chosen from a real HFI rotor angle.  BEMF is
             * exactly zero at standstill, so a finite startup commutation clock
             * is physically unavoidable until the first real floating-phase ZC.
             * Keep that clock separate from all rotor telemetry/estimators. */
            g.trap_bootstrap_phase = g.trap_sector_phase;
            g.trap_command_rpm = launch_dir * FM_TRAP_START_RPM;
            g.trap_line_voltage_v = FM_TRAP_START_LINE_V;
            g.trap_iq_avg = 0.0f;
            g.trap_id_avg = 0.0f;
            g.trap_cal_observer_started = 0U;
            /* Sector->rotor geometry is fixed by the six-step vector table.
             * Never learn a fake offset from open-loop slip. */
            g.trap_cal_ready = 1U;
            g.trap_cal_samples = 0U;
            g.trap_cal_sin_sum = 0.0f;
            g.trap_cal_cos_sum = 0.0f;
            g.trap_cal_offset_rad = 0.0f;
            g.trap_bemf_sync_active = 0U;
            g.trap_had_bemf_lock = 0U;
            g.trap_bemf_sync_good_count = 0U;
            g.trap_bemf_sync_bad_count = 0U;
            g.trap_measured_rpm = 0.0f;
            g.trap_sync_speed_err_rpm = 0.0f;
            g.trap_sync_phase_err_rad = 0.0f;
            g.trap_speed_int_v = 0.0f;
            g.trap_sector_candidate = g.trap_sector;
            g.trap_sector_candidate_count = 0U;
            g.trap_zc_locked = 0U;
            g.trap_zc_good_events = 0U;
            g.trap_zc_rpm = 0.0f;
            g.trap_last_sector_ticks = 0U;
            trap_zc_reset_for_sector();
            g.trap_commutation_count = 0U;
            g.trap_lock_commutation_start = 0U;
            g.trap_foc_retry_cooldown_samples = 0U;
            g.trap_obs_v_accum.alpha = 0.0f;
            g.trap_obs_v_accum.beta = 0.0f;
            g.trap_obs_decim_count = 0U;
            g.trap_theta_cached = g.sling_hfi_seed_theta;
            g.trap_bemf_theta_cached = g.sling_hfi_seed_theta;
            g.trap_bemf_vector_mag_cached = 0.0f;
            g.trap_flux_rpm_cached = 0.0f;
            g.trap_theta_cached_valid = 1U;
            g.trap_bemf_theta_valid_cached = 0U;
            g.trap_flux_angle_available_cached = 0U;
            g.trap_flux_available_cached = 0U;
            g.handover_obs_v_accum.alpha = 0.0f;
            g.handover_obs_v_accum.beta = 0.0f;
            g.handover_obs_decim_count = 0U;
            g.last_v_fund = zero;
            g.sling_attempt = 1U;
            g.sling_axis_quadrature = 0U;
            g.sling_sample = 0U;
            g.sling_phase = (uint8_t)(g.trap_sector + 1U);
            g.sling_theta = g.sling_hfi_seed_theta;
            g.forced_theta = g.sling_hfi_seed_theta;
            g.forced_omega_e = 0.0f;
            g.forced_accel_rpm_s = FM_TRAP_ACCEL_RPM_PER_S;
            g.forced_voltage_v = g.trap_line_voltage_v;
            g.active_target_rpm = g.trap_command_rpm;

            g.int_d = 0.0f;
            g.int_q = 0.0f;
            g.speed_int = 0.0f;
            g.feedback_good_count = 0U;
            g.forced_observer_good_count = 0U;
            g.forced_observer_blend = 0.0f;
            memset(&g.obs, 0, sizeof(g.obs));
            {
                float ss, cc;
                FM_CordicSinCos(g.sling_hfi_seed_theta, &ss, &cc);
                g.obs.theta = g.sling_hfi_seed_theta;
                g.obs.omega_e = 0.0f;
                g.obs.flux_alpha = FM_MOTOR_FLUX_LINKAGE_WB * cc;
                g.obs.flux_beta = FM_MOTOR_FLUX_LINKAGE_WB * ss;
                g.obs.flux_theta_prev = g.sling_hfi_seed_theta;
                g.obs.flux_initialized = 1U;
                g.obs.flux_theta_prev_valid = 0U;
                g.obs.i_alpha_prev = i_raw.alpha;
                g.obs.i_beta_prev = i_raw.beta;
            }
            g.trap_cal_observer_started = 1U;
            g.control_feedback_blend = 0.0f;
            g.control_theta = g.sling_hfi_seed_theta;
            g.control_speed_feedback_rpm = 0.0f;
            g.t.active_estimator = FM_ESTIMATOR_HFI_SEED;
            g.t.launch_phase = 1U;
            g.t.polarity_decision = g.hfi.polarity_decision;
            g.t.polarity_delta_theta_rad = g.hfi.polarity_unwrapped_delta;
            g.t.polarity_probe_iq_a = 0.0f;
            g.t.sling_phase = g.sling_phase;
            g.t.sling_attempt = 1U;
            g.t.sling_axis_quadrature = 0U;
            g.t.sling_seed_angle_rad = g.sling_hfi_seed_theta;
            g.t.sling_iq_cmd_a = 0.0f;
            g.t.sling_peak_phase_current_a = 0.0f;
            g.t.sling_peak_iq_a = 0.0f;
            g.t.sling_peak_vq_v = 0.0f;

            /* Six-step uses immediate CCR updates and changes phase ownership
             * only on actual commutations. */
            FM_PWM_EnterSixStepMode();
            {
                float duty0 = (g.vbus > 1.0f) ?
                              (g.trap_line_voltage_v / g.vbus) : 0.0f;
                FM_PWM_SixStep(g.trap_sector, duty0);
                g.last_v_fund = trap_sixstep_voltage_ab(g.trap_sector,
                                                         g.trap_line_voltage_v);
            }
            set_state(FM_STATE_SLINGSHOT);
            break;
        }
        if (state_elapsed_samples >= (FM_HFI_CONVERGE_TIMEOUT_MS * 40U)) {
            FastMotor_Stop(31U); /* HFI did not settle at standstill */
        }
        break;
    }

    case FM_STATE_SLINGSHOT: {
        if (g.trap_foc_retry_cooldown_samples > 0U)
            --g.trap_foc_retry_cooldown_samples;

        /* PM-FLUX/BEMF CLOSED-LOOP SIX-STEP.
         *
         * HFI supplies the initial rotor angle at standstill.  A short, bounded
         * six-step bootstrap is unavoidable while BEMF is near zero.  Once the
         * voltage/current PM-flux observer proves sustained SAME-DIRECTION real
         * motion, the bootstrap clock is removed from authority.
         *
         * From that point on the measured/predicted PM-flux rotor angle chooses
         * only the next legal six-step sector, and observer RPM closes the
         * six-step voltage/speed loop.  The old reconstructed floating-phase ZC
         * signal is diagnostic only: the PCB does not measure U/V/W terminal
         * voltages, and the 26090309 log proved that model could self-lock while
         * the physical rotor was stationary. */
        g.t.active_estimator = FM_ESTIMATOR_HFI_SEED;
        g.t.launch_phase = 1U;
        g.t.sling_attempt = 1U;
        g.t.sling_axis_quadrature = 0U;
        g.t.sling_seed_angle_rad = g.sling_hfi_seed_theta;
        g.control_feedback_blend = 0.0f;

        FM_AlphaBeta trap_observer_v_sample = g.last_v_fund;

        /* Track real time since the last hardware commutation.  The old
         * floating-phase ZC scheduler is deliberately disabled in this build. */
        if (g.trap_sector_ticks < 0xFFFFFFFFU)
            ++g.trap_sector_ticks;
        g.trap_zc_pending = 0U;
        g.trap_zc_armed = 0U;
        g.trap_zc_locked = 0U;
        g.trap_zc_good_events = 0U;
        g.trap_zc_rpm = 0.0f;
        g.trap_zc_countdown_ticks = 0U;
        g.trap_zc_due_sector_ticks = 0U;
        g.trap_zc_holdover_sectors = 0U;
        g.trap_floating_bemf_v = 0.0f;
        g.trap_floating_current_a = 0.0f;

        float requested_mag = fabsf(g.t.target_rpm);
        if (requested_mag < FM_HYBRID_STOP_RPM) {
            /* Zero command during startup/reversal is a real stop request, not
             * a request to hold one six-step sector with stall current. */
            g.trap_command_rpm = 0.0f;
            g.active_target_rpm = 0.0f;
            g.t.iq_ref_a = 0.0f;
            g.if_iq_ref_a = 0.0f;
            g.speed_int = 0.0f;
            g.int_d = 0.0f;
            g.int_q = 0.0f;
            g.last_v_fund = zero;
            g.t.active_estimator = FM_ESTIMATOR_NONE;
            g.t.launch_phase = 0U;
            FM_PWM_Disable();
            set_state(FM_STATE_IDLE);
            break;
        }
        float desired_dir = hybrid_sign(g.t.target_rpm);
        float current_dir = hybrid_sign(g.trap_command_rpm);
        float goal_mag = (requested_mag >= FM_HYBRID_FOC_ENTER_RPM) ?
                         FM_TRAP_FIELD_CEILING_RPM :
                         requested_mag;
        if (goal_mag > 0.0f && goal_mag < FM_TRAP_START_RPM)
            goal_mag = FM_TRAP_START_RPM;

        /* trap_command_rpm is now ONLY a signed SPEED SETPOINT.  It never moves
         * electrical angle or sectors. */
        float cmd_mag = fabsf(g.trap_command_rpm);
        float accel_rpm_s = g.trap_bemf_sync_active ?
                            FM_TRAP_ACCEL_RPM_PER_S :
                            FM_TRAP_BOOTSTRAP_ACCEL_RPM_PER_S;
        float rpm_step_up = accel_rpm_s * FM_CONTROL_TS_S;
        float rpm_step_down = FM_TRAP_ACQUIRE_DECEL_RPM_PER_S * FM_CONTROL_TS_S;

        float flux_rpm = 0.0f;
        uint8_t flux_available = trap_flux_measurement_available(&flux_rpm);

        /* Direction reversal is never faked by negating an open-loop oscillator.
         * Keep the present physical commutation direction until real speed is near
         * zero; then HFI is allowed to establish the opposite starting sector. */
        if (desired_dir != current_dir) {
            uint8_t still_moving =
                (g.trap_bemf_sync_active && isfinite(g.trap_measured_rpm) &&
                 fabsf(g.trap_measured_rpm) > FM_HYBRID_STOP_RPM);
            if (cmd_mag > rpm_step_down)
                cmd_mag = fmaxf(0.0f, cmd_mag - rpm_step_down);
            else
                cmd_mag = 0.0f;

            if (!still_moving && cmd_mag <= FM_HYBRID_STOP_RPM) {
                /* Direction reversal now starts from a truly fresh standstill
                 * acquisition.  The old path retained observer/holdover and
                 * six-step retry history from the previous direction. */
                begin_fresh_hfi_start(i_raw);
                break;
            }
        } else if (cmd_mag < goal_mag) {
            cmd_mag = fminf(goal_mag, cmd_mag + rpm_step_up);
        } else if (cmd_mag > goal_mag) {
            cmd_mag = fmaxf(goal_mag, cmd_mag - rpm_step_down);
        }

        g.trap_command_rpm = current_dir * cmd_mag;
        g.active_target_rpm = g.trap_command_rpm;

        /* Run the PM-flux/BEMF observer at 10 kHz using the average voltage that
         * actually produced the current samples.  Between observer updates, a
         * locked rotor angle is propagated at 40 kHz from the last measured RPM;
         * every new BEMF/flux sample pulls that prediction back toward reality. */
        uint8_t observer_updated = 0U;
        g.trap_obs_v_accum.alpha += trap_observer_v_sample.alpha;
        g.trap_obs_v_accum.beta += trap_observer_v_sample.beta;
        if (g.trap_obs_decim_count < 255U)
            ++g.trap_obs_decim_count;

        if (g.trap_bemf_sync_active && g.trap_theta_cached_valid &&
            isfinite(g.trap_measured_rpm)) {
            g.trap_theta_cached = FM_WrapPi(
                g.trap_theta_cached +
                omega_from_rpm(g.trap_measured_rpm) * FM_CONTROL_TS_S);
        }

        if (g.trap_obs_decim_count >= FM_TRAP_OBSERVER_DIV) {
            float inv_n = 1.0f / (float)g.trap_obs_decim_count;
            FM_AlphaBeta vavg = {
                g.trap_obs_v_accum.alpha * inv_n,
                g.trap_obs_v_accum.beta * inv_n
            };
            float observer_dt = FM_CONTROL_TS_S *
                                (float)g.trap_obs_decim_count;
            observer_step(i_raw, vavg, 0U,
                          0.0f, 0.0f,
                          0.0f, 0.0f, 0.0f,
                          observer_dt);
            g.trap_obs_v_accum.alpha = 0.0f;
            g.trap_obs_v_accum.beta = 0.0f;
            g.trap_obs_decim_count = 0U;
            observer_updated = 1U;
        }

        float trap_theta_cont = g.trap_theta_cached_valid ?
                                g.trap_theta_cached : g.sling_hfi_seed_theta;
        float bemf_theta = g.trap_bemf_theta_cached;
        float bemf_vector_mag = g.trap_bemf_vector_mag_cached;
        uint8_t bemf_theta_valid = g.trap_bemf_theta_valid_cached;
        uint8_t flux_angle_available = g.trap_flux_angle_available_cached;

        if (observer_updated) {
            float flux2 = g.obs.flux_alpha * g.obs.flux_alpha +
                          g.obs.flux_beta  * g.obs.flux_beta;
            float min_flux = FM_TRAP_MODEL_MIN_FLUX_RATIO *
                             FM_MOTOR_FLUX_LINKAGE_WB;
            flux_angle_available =
                (g.obs.flux_initialized && isfinite(flux2) &&
                 flux2 >= min_flux * min_flux);

            bemf_theta_valid =
                trap_direct_bemf_rotor_theta(current_dir,
                                              &bemf_theta,
                                              &bemf_vector_mag);
            flux_available = trap_flux_measurement_available(&flux_rpm);

            float flux_theta = trap_theta_cont;
            if (flux_angle_available) {
                /* observer_step() has just computed this exact PM-flux atan2
                 * and stored it in theta_meas_raw.  Re-running the CORDIC here
                 * made every 10-kHz six-step observer slot unnecessarily
                 * heavier and was a major source of the 1-in-4 ISR overruns. */
                flux_theta = g.obs.theta_meas_valid ?
                             g.obs.theta_meas_raw :
                             FM_CordicAtan2(g.obs.flux_beta, g.obs.flux_alpha);
            }

            /* Direct BEMF angle has no integral memory and is therefore the
             * preferred phase measurement once its magnitude is strong.  The PM
             * flux angle remains a holdover/cross-check. */
            float measured_theta = bemf_theta_valid ? bemf_theta : flux_theta;
            uint8_t angle_available = bemf_theta_valid || flux_angle_available;

            float bootstrap_theta =
                trap_bootstrap_rotor_ref(g.trap_sector,
                                         g.trap_bootstrap_phase,
                                         current_dir);
            float phase_to_bootstrap = angle_available ?
                FM_WrapPi(measured_theta - bootstrap_theta) : 0.0f;

            uint8_t same_direction =
                (flux_available && isfinite(flux_rpm) &&
                 flux_rpm * current_dir > 0.0f);
            uint8_t speed_physical =
                (same_direction &&
                 fabsf(flux_rpm) >= FM_TRAP_FLUX_LOCK_MIN_RPM);
            uint8_t command_near_enough =
                (speed_physical &&
                 fabsf(flux_rpm - g.trap_command_rpm) <=
                     FM_TRAP_FLUX_LOCK_MAX_CMD_ERR_RPM);
            uint8_t bemf_strong =
                (bemf_theta_valid && isfinite(bemf_vector_mag) &&
                 bemf_vector_mag >= FM_TRAP_FLUX_LOCK_MIN_BEMF_V);

            /* Deferred theta-vs-theta+pi resolution.
             *
             * If the tiny standstill polarity pulse was inconclusive, do not
             * guess forever.  Once the motor has generated genuinely signed
             * flux speed and direct BEMF, compare that physical rotor evidence
             * with the provisional six-step trajectory.  A same-direction,
             * small phase residual confirms the original branch.  Opposite
             * motion or an approximately-pi phase residual means the HFI branch
             * was inverted: record the pi correction and immediately re-anchor
             * the six-step sector to the measured physical rotor angle.
             *
             * This can execute only once because decision 4 is replaced with
             * 6 (motion-confirmed direct) or 7 (motion-confirmed pi flip). */
            if (g.hfi.polarity_decision == 4U &&
                flux_available && bemf_theta_valid &&
                isfinite(flux_rpm) && isfinite(bemf_vector_mag) &&
                fabsf(flux_rpm) >= FM_HFI_POLARITY_MOTION_MIN_RPM &&
                bemf_vector_mag >= FM_HFI_POLARITY_MOTION_MIN_BEMF_V) {
                float abs_phase_boot = fabsf(phase_to_bootstrap);
                uint8_t physical_same_dir = (flux_rpm * current_dir > 0.0f);
                uint8_t direct_geometry =
                    (physical_same_dir &&
                     abs_phase_boot <= FM_TRAP_FLUX_LOCK_ACQUIRE_MAX_ERR_RAD);
                uint8_t pi_geometry =
                    (!physical_same_dir ||
                     abs_phase_boot >=
                        ((float)M_PI - FM_TRAP_FLUX_LOCK_ACQUIRE_MAX_ERR_RAD));

                if (direct_geometry) {
                    g.hfi.polarity_decision = 6U;
                    g.t.polarity_decision = 6U;
                } else if (pi_geometry) {
                    g.hfi_to_foc_offset =
                        FM_WrapPi(g.hfi_to_foc_offset + (float)M_PI);
                    g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
                    g.hfi.polarity_decision = 7U;
                    g.t.polarity_decision = 7U;

                    /* Direct BEMF now gives the physical rotor angle, so use
                     * it rather than replaying the uncertain HFI seed.  If the
                     * provisional branch accelerated the rotor the wrong way,
                     * this sector geometry applies torque in the requested
                     * direction and naturally brakes through zero. */
                    g.sling_hfi_seed_theta = measured_theta;
                    g.trap_sector =
                        trap_sector_from_rotor_theta(measured_theta, current_dir);
                    g.trap_bootstrap_phase =
                        trap_phase_from_rotor_theta_dir(g.trap_sector,
                                                        measured_theta,
                                                        current_dir);
                    g.trap_sector_candidate = g.trap_sector;
                    g.trap_sector_candidate_count = 0U;
                    g.trap_bemf_sync_good_count = 0U;
                    g.trap_bemf_sync_bad_count = 0U;
                    g.trap_theta_cached = measured_theta;
                    g.trap_theta_cached_valid = 0U;
                    g.control_theta = measured_theta;
                    g.forced_theta = measured_theta;

                    float restart_mag = FM_Clamp(fabsf(flux_rpm),
                                                  FM_TRAP_START_RPM,
                                                  FM_TRAP_FIELD_CEILING_RPM);
                    g.trap_command_rpm = current_dir * restart_mag;
                    g.active_target_rpm = g.trap_command_rpm;
                    g.trap_speed_int_v = 0.0f;
                }
            }

            if (!g.trap_bemf_sync_active) {
                /* Do not let the command-synchronous voltage model claim motion
                 * from standstill.  Acquisition needs real flux-speed history,
                 * direct BEMF magnitude, correct sign, and several ms of phase
                 * agreement with the bootstrap trajectory. */
                uint8_t acquire_good =
                    (fabsf(g.trap_command_rpm) >=
                         FM_TRAP_FLUX_LOCK_MIN_CMD_RPM &&
                     speed_physical && command_near_enough && bemf_strong &&
                     fabsf(phase_to_bootstrap) <=
                         FM_TRAP_FLUX_LOCK_ACQUIRE_MAX_ERR_RAD);

                if (acquire_good) {
                    if (g.trap_bemf_sync_good_count < 0xFFFFFFFFU)
                        ++g.trap_bemf_sync_good_count;
                    g.trap_bemf_sync_bad_count = 0U;
                } else {
                    if (g.trap_bemf_sync_good_count >
                        FM_TRAP_FLUX_LOCK_BAD_DEC)
                        g.trap_bemf_sync_good_count -=
                            FM_TRAP_FLUX_LOCK_BAD_DEC;
                    else
                        g.trap_bemf_sync_good_count = 0U;
                    if (g.trap_bemf_sync_bad_count < 0xFFFFFFFFU)
                        ++g.trap_bemf_sync_bad_count;
                }

                if (g.trap_bemf_sync_good_count >=
                    FM_TRAP_BEMF_SYNC_GOOD_SAMPLES) {
                    g.trap_bemf_sync_active = 1U;
                    g.trap_had_bemf_lock = 1U;
                    g.trap_lock_commutation_start = g.trap_commutation_count;
                    g.trap_theta_cached = measured_theta;
                    g.trap_theta_cached_valid = 1U;
                    g.trap_measured_rpm = flux_rpm;
                    g.trap_sector_candidate = g.trap_sector;
                    g.trap_sector_candidate_count = 0U;
                    g.trap_bemf_sync_bad_count = 0U;
                    trap_theta_cont = g.trap_theta_cached;
                } else if (angle_available) {
                    trap_theta_cont = measured_theta; /* diagnostic only */
                }
            } else {
                float innovation = angle_available ?
                    FM_WrapPi(measured_theta - g.trap_theta_cached) : 0.0f;
                uint8_t track_good =
                    (speed_physical && bemf_strong && angle_available &&
                     fabsf(innovation) <=
                         FM_TRAP_FLUX_LOCK_TRACK_MAX_ERR_RAD &&
                     fabsf(flux_rpm - g.trap_measured_rpm) <=
                         FM_TRAP_FLUX_LOCK_MAX_RPM_JUMP);

                if (track_good) {
                    g.trap_theta_cached = FM_WrapPi(
                        g.trap_theta_cached +
                        FM_TRAP_FLUX_ANGLE_CORRECTION * innovation);
                    g.trap_measured_rpm +=
                        FM_TRAP_FLUX_SPEED_ALPHA *
                        (flux_rpm - g.trap_measured_rpm);
                    g.trap_bemf_sync_bad_count = 0U;
                    if (g.trap_bemf_sync_good_count < 0xFFFFFFFFU)
                        ++g.trap_bemf_sync_good_count;
                } else {
                    if (g.trap_bemf_sync_bad_count < 0xFFFFFFFFU)
                        ++g.trap_bemf_sync_bad_count;
                }

                if (g.trap_bemf_sync_bad_count >=
                    FM_TRAP_BEMF_SYNC_BAD_SAMPLES) {
                    /* Return to a matched bootstrap at the LAST observer angle
                     * and speed; never restart from zero and never invent RPM. */
                    float held_theta = g.trap_theta_cached;
                    float held_rpm = g.trap_measured_rpm;
                    g.trap_bemf_sync_active = 0U;
                    g.trap_had_bemf_lock = 0U;
                    g.trap_lock_commutation_start = g.trap_commutation_count;
                    g.trap_bemf_sync_good_count = 0U;
                    g.trap_bemf_sync_bad_count = 0U;
                    g.trap_sector =
                        trap_sector_from_rotor_theta(held_theta, current_dir);
                    g.trap_bootstrap_phase =
                        trap_phase_from_rotor_theta_dir(g.trap_sector,
                                                        held_theta,
                                                        current_dir);
                    if (isfinite(held_rpm) && held_rpm * current_dir > 0.0f) {
                        float held_mag = FM_Clamp(fabsf(held_rpm),
                                                  FM_TRAP_START_RPM,
                                                  FM_TRAP_FIELD_CEILING_RPM);
                        g.trap_command_rpm = current_dir * held_mag;
                        g.active_target_rpm = g.trap_command_rpm;
                    }
                    g.trap_measured_rpm = 0.0f;
                    g.trap_speed_int_v = 0.0f;
                }
                trap_theta_cont = g.trap_theta_cached;
            }

            g.trap_bemf_theta_cached = bemf_theta;
            g.trap_bemf_vector_mag_cached = bemf_vector_mag;
            g.trap_flux_rpm_cached = flux_rpm;
            g.trap_bemf_theta_valid_cached = bemf_theta_valid;
            g.trap_flux_angle_available_cached = flux_angle_available;
            g.trap_flux_available_cached = flux_available;
        } else {
            flux_rpm = g.trap_flux_rpm_cached;
            flux_available = g.trap_flux_available_cached;
            trap_theta_cont = g.trap_bemf_sync_active ?
                              g.trap_theta_cached : trap_theta_cont;
        }

        g.t.trap_flux_available = flux_available;
        g.t.trap_bemf_theta_valid = bemf_theta_valid;

        if (g.trap_bemf_sync_active) {
            /* CLOSED-LOOP commutation: the observer rotor angle owns the sector.
             * A speed-dependent torque advance moves the legal sector boundary,
             * but no target-RPM oscillator can advance the bridge. */
            float advance = trap_torque_advance_for_rpm(g.trap_measured_rpm);
            float theta_for_commutation = FM_WrapPi(
                g.trap_theta_cached + current_dir * advance);
            (void)trap_commutate_from_rotor_theta(theta_for_commutation,
                                                   current_dir);
            g.trap_sector_phase =
                trap_phase_from_rotor_theta_dir(g.trap_sector,
                                                g.trap_theta_cached,
                                                current_dir);
            g.t.active_estimator = FM_ESTIMATOR_BEMF;
        } else {
            /* ZERO/LOW-BEMF bootstrap only.  HFI supplied the initial sector.
             * Ramp slower than the closed-loop command so the rotor has time to
             * follow before the observer is allowed to take authority. */
            float omega_boot_e = omega_from_rpm(g.trap_command_rpm);
            g.trap_bootstrap_phase +=
                (omega_boot_e * FM_CONTROL_TS_S) / ((float)M_PI / 3.0f);

            uint8_t bootstrap_commutated = 0U;
            while (g.trap_bootstrap_phase >= 1.0f) {
                g.trap_bootstrap_phase -= 1.0f;
                g.trap_sector = (uint8_t)((g.trap_sector + 1U) % 6U);
                ++g.trap_commutation_count;
                bootstrap_commutated = 1U;
            }
            while (g.trap_bootstrap_phase < 0.0f) {
                g.trap_bootstrap_phase += 1.0f;
                g.trap_sector = (uint8_t)((g.trap_sector + 5U) % 6U);
                ++g.trap_commutation_count;
                bootstrap_commutated = 1U;
            }
            if (bootstrap_commutated)
                g.trap_sector_ticks = 0U;
            g.trap_sector_phase = FM_Clamp(g.trap_bootstrap_phase, 0.0f, 1.0f);
            trap_theta_cont = trap_bootstrap_rotor_ref(g.trap_sector,
                                                        g.trap_bootstrap_phase,
                                                        current_dir);
            g.trap_measured_rpm = 0.0f;
            g.t.active_estimator = FM_ESTIMATOR_HFI_SEED;
        }

        /* Never expose a model-only RPM as physical mechanical speed before the
         * observer has passed the sustained lock qualification. */
        if (g.trap_bemf_sync_active && isfinite(g.trap_measured_rpm)) {
            g.t.sling_rpm = g.trap_measured_rpm;
            g.control_speed_feedback_rpm = g.trap_measured_rpm;
        } else {
            g.t.sling_rpm = 0.0f;
            g.control_speed_feedback_rpm = 0.0f;
        }

        g.trap_sync_speed_err_rpm = g.trap_bemf_sync_active ?
            (g.trap_command_rpm - g.trap_measured_rpm) : 0.0f;
        g.trap_sync_phase_err_rad = g.trap_bemf_sync_active ?
            FM_WrapPi(g.trap_theta_cached -
                      trap_rotor_ref_for_sector_dir(g.trap_sector,
                                                    current_dir)) : 0.0f;

        /* Deprecated floating-ZC diagnostics are forced to zero so they cannot
         * be mistaken for a real sensor again. */
        g.trap_floating_bemf_v = 0.0f;
        g.trap_floating_current_a = 0.0f;
        g.trap_zc_rpm = 0.0f;
        g.trap_zc_holdover_sectors = 0U;

        g.sling_phase = (uint8_t)(g.trap_sector + 1U);
        g.t.sling_phase = g.sling_phase;
        g.sling_theta = trap_theta_cont;
        g.forced_theta = trap_theta_cont;
        g.control_theta = trap_theta_cont;
        g.forced_omega_e = flux_available ?
                           omega_from_rpm(flux_rpm) : 0.0f;

        /* Speed error changes TORQUE/VOLTAGE only; it never changes sector timing.
         * This is the essential closed-loop split:
         *   rotor/BEMF angle -> commutation
         *   speed error       -> voltage/torque
         * The old 26082834 feed-forward profile remains the baseline so no-load
         * behavior does not receive an unconditional high-current kick. */
        uint8_t flux_speed_available =
            (g.trap_bemf_sync_active && isfinite(g.trap_measured_rpm) &&
             g.trap_measured_rpm * current_dir > 0.0f);
        float sixstep_speed_feedback_rpm = flux_speed_available ?
                                           g.trap_measured_rpm : 0.0f;
        float speed_for_current = flux_speed_available ?
                                  fabsf(g.trap_measured_rpm) : cmd_mag;
        /* Before qualified PM-flux/BEMF lock, keep the low-speed launch current
         * ceiling available so the bootstrap can create real motion.  After a
         * sustained observer lock, taper current toward the gentle FOC handover. */
        uint32_t locked_commutations =
            g.trap_commutation_count - g.trap_lock_commutation_start;
        uint8_t trap_lock_confident =
            (g.trap_bemf_sync_active &&
             g.trap_bemf_sync_good_count >=
                 (FM_TRAP_BEMF_SYNC_GOOD_SAMPLES +
                  FM_TRAP_FLUX_LOCK_EXTRA_GOOD_SAMPLES) &&
             locked_commutations >= FM_TRAP_FOC_MIN_LOCKED_COMMUTATIONS &&
             isfinite(g.trap_measured_rpm) &&
             g.trap_measured_rpm * current_dir > 0.0f);
        g.t.trap_lock_confident = trap_lock_confident;
        g.t.trap_locked_commutations = locked_commutations;
        float soft_current_a = trap_lock_confident ?
                               trap_soft_current_limit(speed_for_current) :
                               FM_TRAP_LAUNCH_CURRENT_A;
        float speed_along_dir = (flux_speed_available) ?
                                (current_dir * sixstep_speed_feedback_rpm) : 0.0f;
        if (speed_along_dir < 0.0f)
            speed_along_dir = 0.0f;
        float speed_err_mag = cmd_mag - speed_along_dir;
        if (speed_err_mag < 0.0f)
            speed_err_mag = 0.0f;

        if (flux_speed_available) {
            g.trap_speed_int_v +=
                FM_TRAP_SPEED_KI_V_PER_RPM_S *
                speed_err_mag * FM_CONTROL_TS_S;
            g.trap_speed_int_v =
                FM_Clamp(g.trap_speed_int_v,
                         0.0f, FM_TRAP_SPEED_BOOST_MAX_V);
        } else if (g.trap_speed_int_v > 0.0f) {
            g.trap_speed_int_v =
                fmaxf(0.0f, g.trap_speed_int_v -
                      0.25f * FM_CONTROL_TS_S);
        }

        float speed_boost_v =
            FM_TRAP_SPEED_KP_V_PER_RPM * speed_err_mag +
            g.trap_speed_int_v;
        speed_boost_v = FM_Clamp(speed_boost_v,
                                 0.0f, FM_TRAP_SPEED_BOOST_MAX_V);

        float base_v = trap_base_line_voltage(cmd_mag);
        float desired_line_v =
            FM_Clamp(base_v + speed_boost_v,
                     FM_TRAP_MIN_LINE_V,
                     FM_TRAP_MAX_LINE_V);

        /* Keep high-VBUS startup out of the tiny-duty region.  This is only a
         * requested-voltage floor: the phase-current shaper below can still
         * reduce g.trap_line_voltage_v beneath it whenever current is high. */
        if (g.vbus > 1.0f) {
            float min_effective_line_v = FM_TRAP_MIN_SOURCE_DUTY * g.vbus;
            if (min_effective_line_v > desired_line_v)
                desired_line_v = fminf(min_effective_line_v,
                                       FM_TRAP_MAX_LINE_V);
        }

        /* Keep the gentle 8-A pre-handover cap only when the rotor is actually
         * following the setpoint.  A loaded rotor that is still >150 rpm behind
         * retains the normal soft-current authority so it can reach 1500 rpm. */
        if (flux_speed_available &&
            fabsf(g.trap_measured_rpm) >= FM_TRAP_FOC_PREP_RPM &&
            speed_err_mag < 150.0f &&
            soft_current_a > FM_TRAP_FOC_PREP_CURRENT_A)
            soft_current_a = FM_TRAP_FOC_PREP_CURRENT_A;

        if (phase_peak > soft_current_a) {
            g.trap_line_voltage_v -= FM_TRAP_VOLTAGE_DROP_PER_SAMPLE_V;
        } else if (g.trap_line_voltage_v < desired_line_v) {
            g.trap_line_voltage_v += FM_TRAP_VOLTAGE_RECOVER_PER_SAMPLE_V;
        } else if (g.trap_line_voltage_v > desired_line_v) {
            g.trap_line_voltage_v -= FM_TRAP_VOLTAGE_RECOVER_PER_SAMPLE_V;
        }
        g.trap_line_voltage_v =
            FM_Clamp(g.trap_line_voltage_v,
                     FM_TRAP_MIN_LINE_V,
                     desired_line_v);
        g.forced_voltage_v = g.trap_line_voltage_v;

        float duty = (g.vbus > 1.0f) ?
                     (g.trap_line_voltage_v / g.vbus) : 0.0f;
        FM_PWM_SixStep(g.trap_sector, duty);

        if ((g.isr_count % FM_TRAP_DQ_HISTORY_DIV) == 0U &&
            !observer_updated) {
            FM_DQ trap_idq = park(i_raw, g.control_theta);
            g.trap_id_avg += FM_TRAP_IQ_HISTORY_ALPHA_DECIMATED *
                             (trap_idq.d - g.trap_id_avg);
            g.trap_iq_avg += FM_TRAP_IQ_HISTORY_ALPHA_DECIMATED *
                             (trap_idq.q - g.trap_iq_avg);
            g.handover_id_meas_avg = g.trap_id_avg;
            g.handover_iq_meas_avg = g.trap_iq_avg;
            g.handover_iq_start_a = g.trap_iq_avg;
            g.t.id_a = trap_idq.d;
            g.t.iq_a = trap_idq.q;
        }

        g.t.iq_ref_a = 0.0f;
        g.t.sling_iq_cmd_a = g.trap_iq_avg;
        g.t.polarity_probe_iq_a = 0.0f;
        g.t.vd_cmd_v = 0.0f;
        g.t.vq_cmd_v = 0.0f;
        g.t.vfund_mag_v = g.trap_line_voltage_v;

        g.trap_cal_ready = 1U;
        g.trap_cal_offset_rad = 0.0f;
        g.handover_angle_offset_rad = 0.0f;
        g.state6_entry_good_count = g.trap_bemf_sync_good_count;
        g.state6_entry_blend = g.trap_bemf_sync_active ? 1.0f : 0.0f;

        g.last_v_fund =
            trap_sixstep_voltage_ab(g.trap_sector,
                                    g.trap_line_voltage_v);

        if (phase_peak > g.t.sling_peak_phase_current_a)
            g.t.sling_peak_phase_current_a = phase_peak;
        if (fabsf(g.t.iq_a) > g.t.sling_peak_iq_a)
            g.t.sling_peak_iq_a = fabsf(g.t.iq_a);
        if (g.trap_line_voltage_v > g.t.sling_peak_vq_v)
            g.t.sling_peak_vq_v = g.trap_line_voltage_v;

        /* Opportunistic BEMF -> FOC catch.
         *
         * The state-4 six-step controller is intentionally left identical to the
         * physically continuous 26090310 behavior.  FOC only gets a brief probe
         * when the already-locked BEMF/flux estimate is exceptionally clean.
         * No pre-handover current reduction is used to manufacture the window. */
        float early_foc_angle_err = bemf_theta_valid ?
            fabsf(FM_WrapPi(bemf_theta - g.trap_theta_cached)) : (float)M_PI;
        float early_foc_speed_err = flux_available ?
            fabsf(flux_rpm - g.trap_measured_rpm) : 1.0e9f;
        g.t.early_foc_angle_error_rad = early_foc_angle_err;
        g.t.early_foc_speed_error_rpm = early_foc_speed_err;
        /* Build 26091606: use the same physical direction proof in both
         * directions.  The negative path was reliable because it required both
         * the qualified six-step RPM and the independent flux RPM to have the
         * commanded sign.  Positive previously bypassed that test entirely. */
        uint8_t signed_motion_proven =
            (isfinite(g.trap_measured_rpm) &&
             isfinite(flux_rpm) &&
             g.trap_measured_rpm * current_dir > 0.0f &&
             flux_rpm * current_dir > 0.0f);

        /* Once HFI polarity is proven, use it in either direction.  Otherwise
         * require the same BEMF/angle/speed/commutation evidence that protects
         * the negative path. */
        uint8_t direction_extra_proof =
            (hfi_polarity_is_proven() ||
             (isfinite(bemf_vector_mag) &&
              bemf_vector_mag >= FM_TRAP_NEG_FOC_MIN_BEMF_V &&
              early_foc_angle_err <= FM_TRAP_NEG_FOC_MAX_ANGLE_ERR_RAD &&
              early_foc_speed_err <= FM_TRAP_NEG_FOC_MAX_SPEED_ERR_RPM &&
              locked_commutations >= FM_TRAP_NEG_FOC_MIN_LOCKED_COMMUTATIONS));

        uint8_t direction_catch_quality =
            (signed_motion_proven && direction_extra_proof);

        g.t.handover_signed_motion_ok = signed_motion_proven;
        g.t.handover_extra_proof_ok = direction_extra_proof;

        /* Once HFI/BEMF has actually proven theta-vs-theta+pi, negative launch
         * no longer needs a slower/different catch window than positive launch.
         * Use the same RPM and phase-current admission; keep only signed-motion
         * proof so a wrong-direction flux transient can never qualify. */
        uint8_t symmetric_negative_catch =
            (current_dir < 0.0f && hfi_polarity_is_proven());

        float foc_catch_min_rpm =
            symmetric_negative_catch ? FM_TRAP_EARLY_FOC_CATCH_RPM :
            ((current_dir < 0.0f) ? FM_TRAP_NEG_FOC_CATCH_RPM :
                                    FM_TRAP_EARLY_FOC_CATCH_RPM);
        /* Do NOT force a polarity-proven negative launch through the positive
         * 16-A topology-admission gate.  The 26090325 log shows valid negative
         * six-step locks at ~18-20 A phase peak; the 16-A gate rejected the
         * first strong window and created multi-second lock-time outliers.
         *
         * This is only permission to change topology while six-step current is
         * in its normal negative range.  State 6/5 FOC remains RPM-controlled
         * and capped to FM_TRAP_FOC_ENTRY_IQ_MAX_A (6 A). */
        float foc_catch_max_phase_current =
            (current_dir < 0.0f) ? FM_TRAP_NEG_FOC_MAX_PHASE_CURRENT_A :
                                   FM_TRAP_EARLY_FOC_MAX_PHASE_CURRENT_A;

        /* Keep field-slip telemetry for comparison, but do not use a
         * positive-only slip veto.  The negative starts in this exact build are
         * fast even with much larger command-vs-rotor speed differences. */
        float trap_field_slip_rpm =
            (isfinite(g.trap_measured_rpm) ?
             (g.trap_command_rpm - g.trap_measured_rpm) : 0.0f);
        uint8_t positive_catch_slip_ok = 1U;
        g.t.trap_field_slip_rpm = trap_field_slip_rpm;
        g.t.positive_catch_slip_ok = positive_catch_slip_ok;

        /* Bit = condition currently GOOD.  0xFF means every positive catch
         * quality gate below is satisfied (apart from retry cooldown/target). */
        uint32_t positive_catch_bits = 0U;
        if (trap_lock_confident) positive_catch_bits |= (1U << 0);
        if (bemf_theta_valid && isfinite(bemf_vector_mag) &&
            bemf_vector_mag >= FM_TRAP_EARLY_FOC_MIN_BEMF_V)
            positive_catch_bits |= (1U << 1);
        if (isfinite(g.trap_measured_rpm) &&
            fabsf(g.trap_measured_rpm) >= foc_catch_min_rpm)
            positive_catch_bits |= (1U << 2);
        if (early_foc_angle_err <= FM_TRAP_EARLY_FOC_MAX_ANGLE_ERR_RAD)
            positive_catch_bits |= (1U << 3);
        if (early_foc_speed_err <= FM_TRAP_EARLY_FOC_MAX_SPEED_ERR_RPM)
            positive_catch_bits |= (1U << 4);
        if (phase_peak <= foc_catch_max_phase_current)
            positive_catch_bits |= (1U << 5);
        if (positive_catch_slip_ok) positive_catch_bits |= (1U << 6);
        if (g.trap_bemf_sync_bad_count == 0U) positive_catch_bits |= (1U << 7);
        g.t.positive_catch_block_bits = positive_catch_bits;

        uint8_t early_foc_window =
            (g.trap_foc_retry_cooldown_samples == 0U &&
             trap_lock_confident && direction_catch_quality &&
             bemf_theta_valid && flux_available &&
             isfinite(bemf_vector_mag) &&
             bemf_vector_mag >= FM_TRAP_EARLY_FOC_MIN_BEMF_V &&
             fabsf(g.trap_measured_rpm) >= foc_catch_min_rpm &&
             early_foc_angle_err <= FM_TRAP_EARLY_FOC_MAX_ANGLE_ERR_RAD &&
             early_foc_speed_err <= FM_TRAP_EARLY_FOC_MAX_SPEED_ERR_RPM &&
             g.trap_bemf_sync_bad_count == 0U &&
             phase_peak <= foc_catch_max_phase_current);
        g.t.early_foc_window = early_foc_window;

        if (early_foc_window &&
            fabsf(g.t.target_rpm) >= FM_HYBRID_FOC_ENTER_RPM &&
            g.trap_cal_ready) {
            g.foc_entry_rpm = g.trap_measured_rpm;
            g.foc_entry_command_rpm = g.trap_command_rpm;
            g.foc_entry_slip_rpm = trap_field_slip_rpm;
            g.foc_entry_bemf_v = bemf_vector_mag;
            g.foc_entry_angle_error_rad = early_foc_angle_err;
            g.foc_entry_speed_error_rpm = early_foc_speed_err;
            g.foc_entry_phase_current_a = phase_peak;

            /* Six-step and FOC now share the SAME measured BEMF/flux rotor
             * estimate.  Seed FOC directly from that observer branch so the
             * topology change does not introduce a second angle source. */
            float theta0 = bemf_theta;
            FM_DQ idq0 = park(i_raw, theta0);

            /* The direct BEMF/PM-flux angle already carries physical magnet
             * polarity.  For negative rotation, do NOT use instantaneous Park-Iq
             * from trapezoidal six-step as a catch veto: it varies strongly with
             * sector/advance and rejected smooth, physically valid negative
             * motion in build 26090316.  Signed flux RPM + measured six-step RPM
             * + BEMF/angle coherence now prove direction.  We still never add pi
             * on the negative path. */
            /* The six-step sector was already observer-regulated; this offset is
             * diagnostic only and is never fed into the commutator. */
            g.trap_cal_offset_rad = FM_WrapPi(theta0 - trap_theta_cont);
            g.handover_angle_offset_rad = g.trap_cal_offset_rad;

            /* Direct BEMF already gives the physical rotor-flux angle for the
             * measured direction.  The negative path became reliable when it
             * stopped choosing theta-vs-theta+pi from instantaneous trapezoidal
             * Park-Iq.  Do the same for positive: never add pi here. */
            g.handover_polarity_pi = 0U;
            g.t.handover_polarity_pi_latched = 0U;
            g.t.foc_entry_measured_rpm = g.trap_measured_rpm;
            g.t.foc_entry_flux_rpm = flux_rpm;
            g.t.foc_entry_id_a = idq0.d;
            g.t.foc_entry_iq_a = idq0.q;
            g.t.topology_flux_phase_error_rad = 0.0f;
            g.t.topology_flux_rpm = flux_rpm;
            g.t.first_foc_sample_valid = 0U;
            g.t.first_foc_flux_rpm = 0.0f;
            g.t.first_foc_phase_error_rad = 0.0f;
            g.first_foc_sample_captured = 0U;

            /* Capture the measured current vector, but do not carry a large
             * six-step torque demand into FOC.  State 6 ramps this bounded vector
             * toward the bounded 6 A capture level while the observer synchronizes. */
            g.handover_id_start_a = FM_Clamp(idq0.d,
                                             -FM_TRAP_FOC_ENTRY_ID_MAX_A,
                                              FM_TRAP_FOC_ENTRY_ID_MAX_A);
            g.handover_iq_start_a = FM_Clamp(idq0.q,
                                             -FM_TRAP_FOC_ENTRY_IQ_MAX_A,
                                              FM_TRAP_FOC_ENTRY_IQ_MAX_A);

            g.forced_theta = theta0;
            g.forced_omega_e = omega_from_rpm(g.trap_measured_rpm);
            g.handover_observer_blend_start = 0.0f; /* bounded PLL trim */
            g.control_theta = theta0;
            g.control_speed_feedback_rpm = g.trap_measured_rpm;
            g.control_feedback_blend = 0.0f;
            g.sling_sample = 0U;
            g.trap_foc_prev_id_a = idq0.d;
            g.trap_foc_prev_iq_a = idq0.q;
            g.trap_foc_stable_count = 0U;
            g.trap_foc_prev_current_valid = 1U;
            g.trap_foc_observer_started = 0U;
            g.handover_obs_v_accum.alpha = 0.0f;
            g.handover_obs_v_accum.beta = 0.0f;
            g.handover_obs_decim_count = 0U;
            g.state6_entry_good_count = 0U;
            g.state6_entry_blend = 0.0f;
            handover_history_reset();

            /* Re-enable the formerly floating leg before switching to SVPWM.
             * Approximate the already-applied six-step voltage as a positive-q
             * stationary vector.  Seed the PI against the FILTERED six-step
             * Id/Iq references that will actually be held in state 6, so the
             * first FOC sample starts from the same current-vector command. */
            FM_PWM_EnableAllPhaseOutputs();
            FM_DQ vseed_dq = {0.0f,
                              FM_TRAP_FOC_EQUIV_VQ_SCALE * g.trap_line_voltage_v};
            FM_AlphaBeta vseed_ab = invpark(vseed_dq, theta0);
            g.last_v_fund = vseed_ab;
            seed_current_pi_for_frame(theta0, i_raw,
                                      g.handover_id_start_a,
                                      g.handover_iq_start_a,
                                      vseed_ab);

            /* From the instant the bridge becomes FOC, torque is commanded by
             * the RPM loop -- never by a fixed handover Iq.  Seed the speed PI
             * at the REAL captured rotor speed and with the already-existing
             * torque so the first speed-loop update is bumpless.  Six amps is
             * only the temporary saturation bound of that RPM controller. */
            g.active_target_rpm = g.trap_measured_rpm;
            g.speed_int = FM_Clamp(g.handover_iq_start_a,
                                   -FM_HYBRID_HANDOVER_CAPTURE_IQ_A,
                                    FM_HYBRID_HANDOVER_CAPTURE_IQ_A);
            g.t.iq_ref_a = g.speed_int;
            g.if_iq_ref_a = g.t.iq_ref_a;

            /* Preserve the already-qualified six-step flux observer. */
            g.obs.theta = theta0;
            g.obs.omega_e = g.forced_omega_e;
            g.obs.i_alpha_prev = i_raw.alpha;
            g.obs.i_beta_prev = i_raw.beta;
            g.obs.phase_err = 0.0f;
            g.obs.polarity_resolved = 1U;
            g.obs.polarity_pi = g.handover_polarity_pi;
            g.t.active_estimator = FM_ESTIMATOR_BLEND;
            g.t.launch_phase = 2U;
            set_state(FM_STATE_OBSERVER_HANDOVER);
            break;
        }

        ++g.sling_sample;
        break;
    }

    case FM_STATE_BEMF_BOOST: {
        /* State 5 is a bounded topology/capture transition, not an operating
         * mode. A stop or opposite-direction command must be actionable
         * immediately instead of waiting for the old active-target ramp. */
        float capture_requested_rpm = g.t.target_rpm;
        float capture_signed_rpm = rpm_from_omega(g.forced_omega_e);

        if (fabsf(capture_requested_rpm) < FM_HYBRID_STOP_RPM) {
            g.t.iq_ref_a = 0.0f;
            g.if_iq_ref_a = 0.0f;
            g.speed_int = 0.0f;
            g.int_d = 0.0f;
            g.int_q = 0.0f;
            g.active_target_rpm = 0.0f;
            g.control_speed_feedback_rpm = 0.0f;
            g.control_feedback_blend = 0.0f;
            g.forced_observer_blend = 0.0f;
            g.observer_phase_valid = 0U;
            g.last_good_observer_valid = 0U;
            g.observer_holdover_active = 0U;
            g.last_v_fund = zero;
            FM_PWM_Disable();
            set_state(FM_STATE_IDLE);
            break;
        }

        if (fabsf(capture_signed_rpm) >= FM_HYBRID_STOP_RPM &&
            capture_requested_rpm * capture_signed_rpm < 0.0f) {
            /* Preserve the real moving direction. Six-step owns the decel and
             * fresh-HFI zero-crossing reversal logic. */
            trap_resume_sixstep_after_failed_foc(i_raw);
            break;
        }

        /* RPM-CONTROLLED FOC CAPTURE.
         *
         * The speed PI was already enabled at the six-step -> FOC topology
         * transition.  This state continues that same RPM controller while the
         * observer is proven coherent.  There is no fixed capture Iq and no
         * requirement to accelerate into an artificial 1850..2150-rpm window. */
        FM_AlphaBeta observer_v_sample = g.last_v_fund;
        observer_step(i_raw, observer_v_sample, 1U,
                      g.control_theta, g.forced_omega_e,
                      0.0f, 0.0f, FM_HYBRID_SYNC_MIN_BEMF_V, FM_CONTROL_TS_S);

        float phase_err = 0.0f;
        uint8_t phase_valid = 0U;
        hybrid_control_pll_step(1.0f, FM_HYBRID_SYNC_MIN_BEMF_V, FM_CONTROL_TS_S,
                                &phase_err, &phase_valid);

        float control_rpm = rpm_from_omega(g.forced_omega_e);
        float flux_rpm = rpm_from_omega(g.obs.flux_omega_e);

        if (isfinite(g.obs.bemf_mag) &&
            g.obs.bemf_mag > g.foc_capture_peak_bemf_v)
            g.foc_capture_peak_bemf_v = g.obs.bemf_mag;
        uint8_t flux_speed_valid = (g.obs.theta_meas_valid &&
                                    isfinite(flux_rpm) &&
                                    isfinite(g.obs.bemf_mag) &&
                                    g.obs.bemf_mag >= FM_HYBRID_SYNC_MIN_BEMF_V);
        uint8_t coherent = (phase_valid && flux_speed_valid &&
                            flux_rpm * control_rpm > 0.0f &&
                            fabsf(flux_rpm - control_rpm) <=
                                FM_HYBRID_SYNC_MAX_FLUX_RPM_ERR &&
                            fabsf(phase_err) <= FM_HYBRID_SYNC_LOCK_ERR_RAD);

        g.observer_phase_valid = coherent;
        g.control_speed_feedback_rpm = coherent ? control_rpm :
                                       (flux_speed_valid ? flux_rpm : 0.0f);
        g.control_feedback_blend = coherent ? 1.0f : 0.0f;
        g.forced_observer_blend = g.control_feedback_blend;
        g.t.active_estimator = FM_ESTIMATOR_BEMF;
        g.t.launch_phase = 3U;

        float k_id = FM_Clamp((float)g.bemf_boost_entry_sample /
                              (float)FM_HYBRID_ID_RELEASE_SAMPLES,
                              0.0f, 1.0f);
        g.bemf_boost_id_cmd_a = (1.0f - k_id) * g.handover_id_start_a;
        if (g.bemf_boost_entry_sample < 0xFFFFFFFFU)
            ++g.bemf_boost_entry_sample;

        /* Negative-only fast probe classification.
         * In the 26090321 log, by ~60 ms 13/15 successful FOC catches had
         * reached >=0.27 V peak FOC BEMF, versus only 1/27 failed catches.
         * Do not sit for 250..300 ms on a weak -600..-800 rpm / 6 A lock. */
        if (control_rpm < 0.0f &&
            g.bemf_boost_entry_sample >= FM_HYBRID_NEG_CAPTURE_CLASSIFY_SAMPLES &&
            g.foc_capture_peak_bemf_v < FM_HYBRID_NEG_CAPTURE_MIN_PEAK_BEMF_V) {
            trap_resume_sixstep_after_failed_foc(i_raw);
            g.trap_foc_retry_cooldown_samples =
                FM_HYBRID_NEG_CAPTURE_WEAK_RETRY_COOLDOWN_SAMPLES;
            break;
        }

        uint32_t capture_stable_required =
            (control_rpm < 0.0f) ? FM_HYBRID_NEG_CAPTURE_STABLE_SAMPLES :
                                   FM_HYBRID_CAPTURE_STABLE_SAMPLES;

        if (coherent) {
            /* FOC capture is RPM control, not Iq control.  Keep the proven
             * positive slew, but use a gentler negative slew because 26090315
             * showed much larger phase error immediately after negative entry. */
            float capture_ramp_rpm_s =
                (control_rpm < 0.0f) ? FM_HYBRID_NEG_CAPTURE_RAMP_RPM_PER_S :
                                       FM_HYBRID_CAPTURE_RAMP_RPM_PER_S;
            ramp_target(g.t.target_rpm, capture_ramp_rpm_s);
            if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U)
                speed_pi_step_limited(control_rpm,
                                      g.active_target_rpm,
                                      FM_HYBRID_CAPTURE_IQ_MAX_A);

            g.foc_capture_load_boost_count = 0U;
            g.bemf_boost_iq_cmd_a = g.t.iq_ref_a;

            /* Reject a self-consistent but physically useless low-speed flux
             * solution. The 26090319 log sat near -700 rpm, ~0.23 V BEMF and
             * -6 A for 6.56 s while the requested speed changed underneath it. */
            float capture_speed_error =
                fabsf(g.active_target_rpm - control_rpm);
            uint8_t capture_stalled =
                (capture_speed_error >= FM_HYBRID_CAPTURE_STALL_ERR_RPM &&
                 fabsf(g.bemf_boost_iq_cmd_a) >= FM_HYBRID_CAPTURE_STALL_IQ_A &&
                 (!isfinite(g.obs.bemf_mag) ||
                  g.obs.bemf_mag <= FM_HYBRID_CAPTURE_STALL_MAX_BEMF_V));

            if (capture_stalled) {
                if (g.foc_capture_stall_count < 0xFFFFFFFFU)
                    ++g.foc_capture_stall_count;
            } else {
                g.foc_capture_stall_count = 0U;
            }

            if (g.foc_capture_stall_count >= FM_HYBRID_CAPTURE_STALL_SAMPLES ||
                g.bemf_boost_entry_sample >=
                    FM_HYBRID_CAPTURE_ABSOLUTE_MAX_SAMPLES) {
                trap_resume_sixstep_after_failed_foc(i_raw);
                break;
            }

            g.last_good_observer_theta = g.control_theta;
            g.last_good_observer_omega_e = g.forced_omega_e;
            g.last_good_observer_valid = 1U;
            g.observer_unlock_count = 0U;
            g.run_low_speed_confirm_count = 0U;

            /* RUN qualification proves OBSERVER-CONTROLLED RPM operation.  It
             * deliberately does not require a magic 1850..2150-rpm window: the
             * outer speed loop is already in charge.  We only require coherent
             * angle/speed and enough physical RPM to stay above the low-speed
             * sensorless crossover. */
            float mag = fabsf(control_rpm);
            uint8_t direction_ok =
                (fabsf(g.t.target_rpm) < FM_HYBRID_STOP_RPM) ||
                (control_rpm * g.t.target_rpm > 0.0f);
            /* Do not declare RUN merely because the FOC observer is briefly
             * self-consistent.  The 26091705 log cleanly separates the two
             * populations: durable positive entries had ~0.44 V or more BEMF
             * (normally much more), while failed entries were <=~0.45 V;
             * durable negative entries were >=0.50 V while failed entries were
             * <=0.43 V.  State 5 is already full-torque 22-A FOC, so waiting
             * here costs essentially no six-step time and avoids the ugly
             * 4->6/5->7->4 bounce. */
            uint8_t positive_run_ready =
                (control_rpm <= 0.0f ||
                 (isfinite(g.obs.bemf_mag) &&
                  g.obs.bemf_mag >= FM_HYBRID_POS_FOC_RUN_MIN_BEMF_V &&
                  control_rpm >= FM_HYBRID_POS_FOC_RUN_MIN_RPM &&
                  flux_rpm > 0.0f &&
                  fabsf(flux_rpm - control_rpm) <=
                      FM_HYBRID_POS_FOC_RUN_MAX_SPEED_ERR_RPM));
            uint8_t negative_run_ready =
                (control_rpm >= 0.0f ||
                 (isfinite(g.obs.bemf_mag) &&
                  g.obs.bemf_mag >= FM_HYBRID_NEG_FOC_RUN_MIN_BEMF_V &&
                  fabsf(phase_err) <= FM_HYBRID_NEG_FOC_RUN_MAX_PHASE_ERR_RAD &&
                  control_rpm <= -FM_HYBRID_NEG_FOC_RUN_MIN_RPM &&
                  flux_rpm <= -FM_HYBRID_NEG_FOC_RUN_MIN_RPM &&
                  fabsf(flux_rpm - control_rpm) <=
                      FM_HYBRID_NEG_FOC_RUN_MAX_SPEED_ERR_RPM));
            uint8_t direction_run_ready =
                (control_rpm < 0.0f) ? negative_run_ready : positive_run_ready;
            g.neg_run_admission_ready = (control_rpm < 0.0f) ? negative_run_ready : 0U;
            if (mag >= FM_HYBRID_CAPTURE_MIN_SAFE_RPM &&
                direction_ok && direction_run_ready &&
                fabsf(phase_err) <= FM_HYBRID_SYNC_LOCK_ERR_RAD) {
                if (g.foc_capture_good_count < capture_stable_required)
                    ++g.foc_capture_good_count;
            } else {
                /* Do not destroy an otherwise-good 10-ms FOC proof because one
                 * 25-us sample briefly dips below the BEMF/RPM/readiness gate.
                 * Bad evidence must persist to erase confidence. */
                if (g.foc_capture_good_count > FM_HYBRID_CAPTURE_GOOD_DECAY_SAMPLES)
                    g.foc_capture_good_count -= FM_HYBRID_CAPTURE_GOOD_DECAY_SAMPLES;
                else
                    g.foc_capture_good_count = 0U;
            }
        } else {
            /* Same idea for a short observer dropout: retain only a decaying
             * amount of the previous proof while the last-good trajectory is
             * propagated. Persistent incoherence still erases the proof quickly. */
            if (g.foc_capture_good_count > FM_HYBRID_CAPTURE_GOOD_DECAY_SAMPLES)
                g.foc_capture_good_count -= FM_HYBRID_CAPTURE_GOOD_DECAY_SAMPLES;
            else
                g.foc_capture_good_count = 0U;
            g.foc_capture_stall_count = 0U;

            if (g.bemf_boost_entry_sample >=
                FM_HYBRID_CAPTURE_ABSOLUTE_MAX_SAMPLES) {
                trap_resume_sixstep_after_failed_foc(i_raw);
                break;
            }

            /* Do not kill torque on the first bad observer sample.  The logs show
             * that capture can be perfectly coherent, then lose one short window;
             * forcing Iq to zero immediately removes BEMF and makes recovery much
             * less likely.  Propagate the last real speed/angle for only 5 ms and
             * keep the RPM loop alive with only 4 A of maximum authority.  If
             * coherence does not return quickly, resume six-step at the current
             * speed/sector instead of increasing FOC current. */
            if (g.observer_unlock_count < 0xFFFFFFFFU)
                ++g.observer_unlock_count;

            if (g.observer_unlock_count <=
                FM_HYBRID_CAPTURE_DROPOUT_HOLD_SAMPLES &&
                g.last_good_observer_valid) {
                /* Still RPM control during the short observer dropout: advance
                 * the last real trajectory and let the speed PI choose torque,
                 * but reduce its authority to 4 A.  No fixed Iq is injected. */
                g.forced_omega_e = g.last_good_observer_omega_e;
                g.control_theta = FM_WrapPi(g.control_theta +
                                            g.forced_omega_e * FM_CONTROL_TS_S);
                float hold_rpm = rpm_from_omega(g.last_good_observer_omega_e);
                g.control_speed_feedback_rpm = hold_rpm;
                if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U)
                    speed_pi_step_limited(hold_rpm,
                                          g.active_target_rpm,
                                          FM_HYBRID_CAPTURE_DROPOUT_IQ_A);
                g.bemf_boost_iq_cmd_a = g.t.iq_ref_a;
                g.run_low_speed_confirm_count = 0U;
            } else {
                trap_resume_sixstep_after_failed_foc(i_raw);
                break;
            }
        }

        g.t.iq_ref_a = g.bemf_boost_iq_cmd_a;
        g.if_iq_ref_a = g.bemf_boost_iq_cmd_a;
        current_control(g.control_theta, i_raw,
                        g.bemf_boost_id_cmd_a,
                        g.bemf_boost_iq_cmd_a, zero);

        if (coherent &&
            g.bemf_boost_entry_sample >= FM_HYBRID_ID_RELEASE_SAMPLES &&
            g.foc_capture_good_count >= capture_stable_required) {
            /* RPM loop is already running and bumpless.  Keep its setpoint and
             * integrator when RUN is granted; only the available PI authority
             * will be opened gradually in RUN. */
            g.speed_int = FM_Clamp(g.speed_int,
                                   -FM_HYBRID_CAPTURE_IQ_MAX_A,
                                    FM_HYBRID_CAPTURE_IQ_MAX_A);
            g.handover_iq_ref_avg = g.bemf_boost_iq_cmd_a;
            g.handover_id_meas_avg = 0.0f;
            g.handover_iq_meas_avg = g.bemf_boost_iq_cmd_a;
            g.observer_unlock_count = 0U;
            g.observer_holdover_active = 0U;
            g.observer_holdover_count = 0U;
            g.run_entry_grace_count = 0U;
            g.neg_run_release_count = 0U;
            g.neg_run_release_k = 0.0f;
            g.neg_run_target_ramp = FM_HYBRID_NEG_RUN_SAFE_RAMP_RPM_PER_S;
            g.neg_run_false_lock_count = 0U;
            g.neg_run_false_lock_active = 0U;
            g.neg_run_quality_bits = 0U;
            g.neg_run_iq_limit_a = FM_HYBRID_NEG_RUN_SAFE_IQ_A;
            g.t.launch_phase = 5U;
            set_state(FM_STATE_RUN);
        }
        break;
    }

    case FM_STATE_OBSERVER_HANDOVER: {
        /* Commands stay live during topology handover. Do not finish an old
         * signed FOC transition after the host requests stop or reversal. */
        float handover_requested_rpm_now = g.t.target_rpm;
        float handover_signed_rpm_now = rpm_from_omega(g.forced_omega_e);

        if (fabsf(handover_requested_rpm_now) < FM_HYBRID_STOP_RPM) {
            g.t.iq_ref_a = 0.0f;
            g.if_iq_ref_a = 0.0f;
            g.speed_int = 0.0f;
            g.int_d = 0.0f;
            g.int_q = 0.0f;
            g.active_target_rpm = 0.0f;
            g.last_v_fund = zero;
            FM_PWM_Disable();
            set_state(FM_STATE_IDLE);
            break;
        }

        if (fabsf(handover_signed_rpm_now) >= FM_HYBRID_STOP_RPM &&
            handover_requested_rpm_now * handover_signed_rpm_now < 0.0f) {
            trap_resume_sixstep_after_failed_foc(i_raw);
            break;
        }

        /* SIX-STEP -> RPM-CONTROLLED FOC -> CONTINUOUS OBSERVER PHASE LOCK.
         *
         * The bridge enters FOC with the RPM PI already seeded from the captured
         * physical speed/current.  This replaces fixed-Iq capture and every
         * previous hard/soft observer-angle handover.
         * The Park angle is g.control_theta before, during and after the
         * transition.  At first it advances only from the six-step commutation
         * speed.  After a short constant-speed stabilization, the raw PM-flux
         * angle is allowed to pull that SAME trajectory through a modest
         * type-II phase-lock loop.  There is therefore nothing to switch to. */
        g.t.active_estimator = FM_ESTIMATOR_BLEND;
        g.t.launch_phase = 2U;

        FM_AlphaBeta observer_v_sample = g.last_v_fund;
        float current_release = FM_Clamp((float)g.sling_sample /
                                         (float)FM_HYBRID_HANDOVER_CURRENT_RAMP_SAMPLES,
                                         0.0f, 1.0f);
        float id_start = FM_Clamp(g.handover_id_start_a,
                                  -FM_TRAP_FOC_ENTRY_ID_MAX_A,
                                   FM_TRAP_FOC_ENTRY_ID_MAX_A);
        const float id_ref = (1.0f - current_release) * id_start;

        /* FOC begins under RPM control.  Seeded active_target_rpm starts at the
         * measured handover speed, then slews toward the user's requested RPM.
         * Iq remains solely the speed-PI output, but capture now has the same
         * 22-A torque ceiling as six-step so a loaded rotor is not starved at
         * the topology boundary.  This is an authority limit, not a fixed 22 A
         * command. */
        float handover_rpm = rpm_from_omega(g.forced_omega_e);
        float handover_ramp_rpm_s =
            (handover_rpm < 0.0f) ? FM_HYBRID_NEG_CAPTURE_RAMP_RPM_PER_S :
                                    FM_HYBRID_CAPTURE_RAMP_RPM_PER_S;
        uint32_t sync_lock_required =
            (handover_rpm < 0.0f) ? FM_HYBRID_NEG_SYNC_LOCK_SAMPLES :
                                    FM_HYBRID_SYNC_LOCK_SAMPLES;
        ramp_target(g.t.target_rpm, handover_ramp_rpm_s);
        if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U)
            speed_pi_step_limited(handover_rpm,
                                  g.active_target_rpm,
                                  FM_HYBRID_HANDOVER_CAPTURE_IQ_A);
        const float iq_ref = g.t.iq_ref_a;

        float phase_err = 0.0f;
        uint8_t phase_valid = 0U;
        float authority = 0.0f;

        if (g.sling_sample < FM_HYBRID_TOPOLOGY_SETTLE_SAMPLES) {
            /* During the bridge topology settle, keep the voltage model paused
             * but DO NOT freeze a qualified negative six-step flux vector.
             *
             * Positive startup already works because its observer history
             * remains continuous.  The old negative-only path later rebuilt
             * flux at nominal magnitude, and the log shows the slow negative
             * launches are precisely the handovers that reappear with a huge
             * phase error / wrong instantaneous flux branch.
             *
             * For negative motion, rotate the existing physical PM-flux vector
             * by the already-proven signed six-step omega each control tick.
             * This preserves both its measured magnitude and its physical
             * theta-vs-pi branch while ignoring the SVPWM switching transient.
             * The first FOC observer sample therefore starts from the same real
             * rotor trajectory that six-step was already using. */
            float dtheta_settle = g.forced_omega_e * FM_CONTROL_TS_S;
            g.control_theta = FM_WrapPi(g.control_theta + dtheta_settle);

            if (g.obs.flux_initialized) {
                /* The negative path already propagates the qualified PM-flux
                 * vector across this 0.5-ms observer pause.  Apply the same
                 * treatment to positive so flux and control_theta remain on the
                 * same physical trajectory through the topology change. */
                float sd, cd;
                FM_CordicSinCos(dtheta_settle, &sd, &cd);
                float fa = g.obs.flux_alpha;
                float fb = g.obs.flux_beta;
                g.obs.flux_alpha = fa * cd - fb * sd;
                g.obs.flux_beta  = fa * sd + fb * cd;
                g.obs.flux_theta_prev =
                    FM_WrapPi(g.obs.flux_theta_prev + dtheta_settle);
                g.obs.flux_omega_e = g.forced_omega_e;
                /* Keep derivative continuity local to the new topology. */
                g.obs.i_alpha_prev = i_raw.alpha;
                g.obs.i_beta_prev = i_raw.beta;
                g.obs.di_alpha = 0.0f;
                g.obs.di_beta = 0.0f;

                float flux_theta_now =
                    FM_CordicAtan2(g.obs.flux_beta, g.obs.flux_alpha);
                g.t.topology_flux_phase_error_rad =
                    FM_WrapPi(flux_theta_now - g.control_theta);
                g.t.topology_flux_rpm =
                    rpm_from_omega(g.obs.flux_omega_e);
            } else {
                g.t.topology_flux_phase_error_rad = 0.0f;
                g.t.topology_flux_rpm = 0.0f;
                g.obs.i_alpha_prev = i_raw.alpha;
                g.obs.i_beta_prev = i_raw.beta;
            }

            g.obs.theta_meas_valid = 0U;
            g.obs.theta = g.control_theta;
            g.obs.omega_e = g.forced_omega_e;
            g.trap_foc_observer_started = 0U;
            g.feedback_good_count = 0U;
        } else {
            if (!g.trap_foc_observer_started) {
                /* Resume the qualified observer after topology settling.
                 *
                 * Negative-catch logs showed a very specific failure: the rotor
                 * and six-step flux estimate were cleanly negative before the
                 * bridge changed topology, but roughly half of failed catches
                 * produced a POSITIVE raw_flux_rpm on the first FOC observer
                 * sample.  During the 0.5-ms settle interval the control angle
                 * keeps moving while flux_alpha/beta were previously left
                 * frozen.  At 600..900 mechanical rpm that is ~13..19
                 * electrical degrees of stale flux angle, enough for the first
                 * FOC voltage-model update to create a false signed speed.
                 *
                 * Positive startup is already reliable, so leave it untouched.
                 * For a real negative moving handover only, re-anchor the PM
                 * flux vector to the SAME physical rotor/control angle and
                 * signed omega already proven by six-step.  The raw observer
                 * must then rebuild speed from fresh FOC samples; no fake speed
                 * sample is generated at the topology boundary. */
                /* 26090325: negative flux is already propagated through the
                 * topology-settle interval above.  Do not reconstruct it at
                 * nominal magnitude here; preserving the qualified six-step
                 * observer is the key symmetry with the positive path. */
                g.obs.theta = g.control_theta;
                g.obs.omega_e = g.forced_omega_e;
                g.obs.i_alpha_prev = i_raw.alpha;
                g.obs.i_beta_prev = i_raw.beta;
                g.obs.phase_err = 0.0f;
                g.obs.polarity_resolved = 1U;
                g.obs.polarity_pi = g.handover_polarity_pi;
                g.trap_foc_observer_started = 1U;
                g.feedback_good_count = 0U;
                g.trap_bemf_sync_bad_count = 0U;
                g.handover_obs_v_accum.alpha = 0.0f;
                g.handover_obs_v_accum.beta = 0.0f;
                g.handover_obs_decim_count = 0U;
            }

            uint32_t observer_age = g.sling_sample -
                                    FM_HYBRID_TOPOLOGY_SETTLE_SAMPLES;
            if (observer_age >= FM_HYBRID_IF_STABILIZE_SAMPLES) {
                uint32_t sync_age = observer_age -
                                    FM_HYBRID_IF_STABILIZE_SAMPLES;
                authority = FM_Clamp((float)sync_age /
                                     (float)FM_HYBRID_SYNC_RAMP_SAMPLES,
                                     0.0f, 1.0f);
            }

            /* Observer + phase PLL run at the full 40 kHz during the topology
             * handover.  The 26090305 20-kHz experiment reduced ISR load but
             * repeatedly lost an otherwise-good rotor estimate before RUN.
             * Other diagnostic/HFI work remains removed from this hot path. */
            g.handover_obs_v_accum.alpha += observer_v_sample.alpha;
            g.handover_obs_v_accum.beta += observer_v_sample.beta;
            if (g.handover_obs_decim_count < 255U)
                ++g.handover_obs_decim_count;

            uint8_t handover_observer_updated = 0U;
            if (g.handover_obs_decim_count >= FM_HYBRID_HANDOVER_OBSERVER_DIV) {
                float inv_n = 1.0f / (float)g.handover_obs_decim_count;
                FM_AlphaBeta vavg = {
                    g.handover_obs_v_accum.alpha * inv_n,
                    g.handover_obs_v_accum.beta * inv_n
                };
                float observer_dt = FM_CONTROL_TS_S *
                                    (float)g.handover_obs_decim_count;
                observer_step(i_raw, vavg, 1U,
                              g.control_theta, g.forced_omega_e,
                              0.0f, 0.0f, FM_HYBRID_SYNC_MIN_BEMF_V, observer_dt);
                hybrid_control_pll_step(authority,
                                        FM_HYBRID_SYNC_MIN_BEMF_V, observer_dt,
                                        &phase_err, &phase_valid);
                if (!g.first_foc_sample_captured) {
                    g.first_foc_sample_captured = 1U;
                    g.t.first_foc_sample_valid = 1U;
                    g.t.first_foc_flux_rpm =
                        rpm_from_omega(g.obs.flux_omega_e);
                    g.t.first_foc_phase_error_rad = phase_err;
                }
                g.handover_obs_v_accum.alpha = 0.0f;
                g.handover_obs_v_accum.beta = 0.0f;
                g.handover_obs_decim_count = 0U;
                handover_observer_updated = 1U;
            } else {
                /* Hold the last 20-kHz observer angle for this one interleaved
                 * 40-kHz current sample.  The next observer/PLL update advances
                 * by the full 2*Ts interval, avoiding double-prediction.  At the
                 * ~1500-rpm crossover this is only ~1.6 electrical degrees. */
                g.obs.theta = g.control_theta;
                g.obs.omega_e = g.forced_omega_e;
                phase_err = g.obs.phase_err;
            }

            /* Count lock evidence only when a new physical observer sample exists.
             * With the divider restored to 1 this is every 40-kHz control sample. */
            if (handover_observer_updated) {
                float handover_flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
                uint8_t signed_flux_ok =
                    (handover_rpm >= 0.0f ||
                     !g.obs.theta_meas_valid ||
                     g.obs.flux_speed_sample_count < FM_TRAP_FLUX_SPEED_MIN_SAMPLES ||
                     handover_flux_rpm < 0.0f);

                if (authority >= 0.999f && phase_valid && signed_flux_ok &&
                    hybrid_flux_speed_coherent(rpm_from_omega(g.forced_omega_e)) &&
                    fabsf(phase_err) <= FM_HYBRID_SYNC_LOCK_ERR_RAD) {
                    if (g.feedback_good_count < sync_lock_required)
                        ++g.feedback_good_count;
                } else {
                    g.feedback_good_count = 0U;
                }
            }
        }

        g.control_speed_feedback_rpm = rpm_from_omega(g.forced_omega_e);
        /* active_target_rpm is the RPM-loop setpoint and must NOT be overwritten
         * by measured/predicted speed during handover.  It was seeded to the
         * real capture RPM and is already slewing toward the user's command. */
        g.control_feedback_blend = authority;
        g.forced_observer_blend = authority;
        g.state6_entry_blend = authority;
        g.state6_entry_good_count = g.feedback_good_count;
        g.t.forced_observer_good_count = g.feedback_good_count;
        g.t.iq_ref_a = iq_ref;
        g.if_iq_ref_a = iq_ref;

        /* Current continuity is preserved at the topology switch, then torque is
         * deliberately reduced while the 1500-rpm FOC frame synchronizes.  This
         * prevents the old ~22 A six-step vector from launching FOC straight past
         * the intended 2000-rpm capture point. */
        current_control(g.control_theta, i_raw, id_ref, iq_ref, zero);

        /* If FOC acquisition is grossly inconsistent, do not sit forever on a
         * synthetic I-f speed while the physical rotor is stopped.  After the
         * normal topology/PLL settle interval, persistent absence of a coherent
         * raw flux speed sends us straight back to RUNNING six-step -- not HFI,
         * not zero torque, and not a 120-rpm restart. */
        if (g.handover_obs_decim_count == 0U && authority >= 0.999f &&
            g.feedback_good_count < sync_lock_required) {
            float raw_flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
            float if_rpm = rpm_from_omega(g.forced_omega_e);
            uint8_t gross_handover_bad =
                (!g.obs.theta_meas_valid || !isfinite(raw_flux_rpm) ||
                 raw_flux_rpm * if_rpm <= 0.0f ||
                 fabsf(raw_flux_rpm - if_rpm) >
                     FM_HYBRID_SYNC_MAX_FLUX_RPM_ERR);
            if (gross_handover_bad) {
                if (g.trap_bemf_sync_bad_count <
                    FM_HYBRID_HANDOVER_FALLBACK_SAMPLES)
                    ++g.trap_bemf_sync_bad_count;
            } else {
                g.trap_bemf_sync_bad_count = 0U;
            }

            if (g.trap_bemf_sync_bad_count >=
                FM_HYBRID_HANDOVER_FALLBACK_SAMPLES) {
                trap_resume_sixstep_after_failed_foc(i_raw);
                break;
            }
        } else {
            g.trap_bemf_sync_bad_count = 0U;
        }

        if (authority >= 0.999f &&
            g.feedback_good_count >= sync_lock_required) {
            g.handover_id_start_a = id_ref;
            g.handover_iq_start_a = iq_ref;
            g.bemf_boost_id_cmd_a = id_ref;
            g.bemf_boost_iq_cmd_a = iq_ref;
            g.bemf_boost_entry_sample = 0U;
            g.foc_capture_good_count = 0U;
            g.foc_capture_load_boost_count = 0U;
            g.foc_capture_stall_count = 0U;
            g.foc_capture_peak_bemf_v = 0.0f;
            /* Keep the RPM-loop trajectory/integrator established in state 6.
             * State 5 continues the same controller with the same fixed 22-A
             * capture ceiling. */
            g.speed_int = FM_Clamp(g.speed_int,
                                   -FM_HYBRID_CAPTURE_IQ_MAX_A,
                                    FM_HYBRID_CAPTURE_IQ_MAX_A);
            g.feedback_good_count = 0U;
            g.control_feedback_blend = 1.0f;
            g.forced_observer_blend = 1.0f;
            g.t.active_estimator = FM_ESTIMATOR_BEMF;
            set_state(FM_STATE_BEMF_BOOST);
            break;
        }

        /* No acquisition timeout/fault.  If the observer is slow, the motor
         * simply remains in current-controlled I-f FOC until it phase-locks. */
        ++g.sling_sample;
        break;
    }

    case FM_STATE_RUN: {
        if (g.run_entry_grace_count < 0xFFFFFFFFU)
            ++g.run_entry_grace_count;

        /* HIGH-SPEED SENSORLESS FOC.  A questionable phase measurement is now a
         * ZERO/DECAYING-TORQUE coast-and-reacquire event.  The old code allowed
         * raw flux to pull a still-torquing control frame by tens of electrical
         * degrees, which is how a ~1 A request turned into 14..30 A measured
         * current in the previous ~20 krpm log. */
        FM_AlphaBeta observer_v_sample = g.last_v_fund;
        /* Raw-current 20kHz RUN observer-only A/B experiment.
         * The 40kHz control/ADC/OCP uses the fresh current on EVERY callback.
         * Observer integrates latest raw current and the mean of two previous
         * applied voltage samples every 50us. The interleaved callback only
         * advances the rotor control angle (25us), never repeats PLL correction.
         * No V13 code runs or influences this observer.
         * RUN entry retains the already acquired 40kHz flux observer state. */
        float phase_err = g.obs.phase_err;
        uint8_t phase_valid = g.observer_phase_valid;
#if FM_RUN_RAW_OBSERVER_20KHZ
        if (s_run20k_session != g.state_start_isr_count) {
            s_run20k_session = g.state_start_isr_count;
            s_run20k_v_accum.alpha = 0.0f;
            s_run20k_v_accum.beta = 0.0f;
            s_run20k_accum_count = 0U;
            s_run20k_correction_due = 0U;
            /* Do not clear observer's angle or state at RUN entry. */
        }
        s_run20k_v_accum.alpha += observer_v_sample.alpha;
        s_run20k_v_accum.beta += observer_v_sample.beta;
        ++s_run20k_accum_count;
        g.t.observer20k_mode = 1U;
        if (s_run20k_accum_count == 2U) {
            const FM_AlphaBeta voltage_avg = {
                s_run20k_v_accum.alpha * 0.5f,
                s_run20k_v_accum.beta  * 0.5f
            };
            s_run20k_v_accum.alpha = 0.0f;
            s_run20k_v_accum.beta = 0.0f;
            s_run20k_accum_count = 0U;
            s_run20k_correction_due = 1U;
            uint32_t obs_begin = DWT->CYCCNT;
            observer_step(i_raw, voltage_avg, 1U,
                          g.control_theta, g.forced_omega_e,
                          0.0f, 0.0f, FM_HYBRID_SYNC_MIN_BEMF_V,
                          2.0f * FM_CONTROL_TS_S);
            hybrid_control_pll_step(1.0f, FM_HYBRID_SYNC_MIN_BEMF_V,
                                    FM_CONTROL_TS_S, &phase_err, &phase_valid);
            s_run20k_correction_due = 0U;
            uint32_t obs_cycles = DWT->CYCCNT - obs_begin;
            s_v13_run_observer_cycles_last = obs_cycles;
            if (obs_cycles > s_v13_run_observer_cycles_max)
                s_v13_run_observer_cycles_max = obs_cycles;
            ++s_run20k_updates;
            s_run20k_last_observer_tick = g.isr_count;
        } else {
            /* Predict every missed observer tick so FOC Park angle is fresh.
             * Do not consume old phase error twice or recalculate flux here. */
            g.control_theta = FM_WrapPi(g.control_theta +
                                       g.forced_omega_e * FM_CONTROL_TS_S);
            g.obs.theta = g.control_theta;
            g.obs.omega_e = g.forced_omega_e;
            ++s_run20k_predicts;
        }
#else
        g.t.observer20k_mode = 2U;
        uint32_t obs_begin = DWT->CYCCNT;
        observer_step(i_raw, observer_v_sample, 1U,
                      g.control_theta, g.forced_omega_e,
                      0.0f, 0.0f, FM_HYBRID_SYNC_MIN_BEMF_V,
                      FM_CONTROL_TS_S);
        hybrid_control_pll_step(1.0f, FM_HYBRID_SYNC_MIN_BEMF_V,
                                FM_CONTROL_TS_S, &phase_err, &phase_valid);
        uint32_t obs_cycles = DWT->CYCCNT - obs_begin;
        s_v13_run_observer_cycles_last = obs_cycles;
        if (obs_cycles > s_v13_run_observer_cycles_max)
            s_v13_run_observer_cycles_max = obs_cycles;
        ++s_run20k_updates;
#endif
        if ((g.isr_count % FM_SPEED_LOOP_DIV) == 1U) {
            g.t.observer_run_cycles_last = s_v13_run_observer_cycles_last;
            g.t.observer_run_cycles_max = s_v13_run_observer_cycles_max;
            g.t.observer20k_updates = s_run20k_updates;
            g.t.observer20k_predicts = s_run20k_predicts;
        }

        float pll_rpm = rpm_from_omega(g.forced_omega_e);
        float flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
        uint8_t flux_speed_valid = (g.obs.theta_meas_valid &&
                                    isfinite(flux_rpm) &&
                                    isfinite(g.obs.bemf_mag) &&
                                    g.obs.bemf_mag >= FM_HYBRID_SYNC_MIN_BEMF_V);

        float raw_theta = flux_speed_valid ? hybrid_aligned_raw_theta() :
                                             g.control_theta;
        float rejoin_err = flux_speed_valid ?
                           FM_WrapPi(raw_theta - g.control_theta) : 0.0f;
        g.rejoin_angle_error_rad = rejoin_err;

        float rpm_gate = FM_HYBRID_RUN_REACQUIRE_MAX_RPM_ERR;
        float rpm_scale = 0.20f * fmaxf(fabsf(pll_rpm), fabsf(flux_rpm));
        if (rpm_scale > rpm_gate) rpm_gate = rpm_scale;
        uint8_t run_feedback_valid =
            (phase_valid && flux_speed_valid &&
             pll_rpm * flux_rpm > 0.0f &&
             fabsf(pll_rpm - flux_rpm) <= rpm_gate &&
             fabsf(rejoin_err) <= FM_HYBRID_RUN_REACQUIRE_MAX_ANGLE_ERR_RAD);

        g.observer_phase_valid = run_feedback_valid;
        g.control_speed_feedback_rpm = run_feedback_valid ? pll_rpm :
                                       (flux_speed_valid ? flux_rpm : 0.0f);
        g.t.active_estimator = FM_ESTIMATOR_BEMF;
        g.t.launch_phase = 5U;

        uint8_t normal_torque_authority = 0U;

        if (run_feedback_valid && !g.observer_holdover_active) {
            normal_torque_authority = 1U;
            g.observer_unlock_count = 0U;
            g.run_low_speed_confirm_count = 0U;
            g.observer_holdover_count = 0U;
            g.observer_recatch_samples = 0U;
            g.control_feedback_blend = 1.0f;
            g.forced_observer_blend = 1.0f;
            g.last_good_observer_theta = g.control_theta;
            g.last_good_observer_omega_e = g.forced_omega_e;
            g.last_good_observer_valid = 1U;

            if (g.speed_command_pending) {
                g.active_target_rpm = pll_rpm;
                g.speed_int = FM_Clamp(g.t.iq_ref_a,
                                       -FM_SPEED_IQ_MAX_A, FM_SPEED_IQ_MAX_A);
                g.speed_command_pending = 0U;
            }
        } else {
            /* Whether raw phase is fully absent or merely disagrees with flux
             * speed, do not let the outer speed PI add torque. */
            if (!g.observer_holdover_active) {
                g.observer_holdover_active = 1U;
                g.observer_recatch_samples = 0U;
            }
            if (g.observer_unlock_count < FM_HYBRID_RUN_RAW_LOSS_SAMPLES)
                ++g.observer_unlock_count;
            g.observer_holdover_count = g.observer_unlock_count;
            g.control_feedback_blend = 0.0f;
            g.forced_observer_blend = 0.0f;

            /* A one-frame observer dropout immediately after a valid FOC
             * capture must not remove all torque and strand RUN at zero RPM.
             * For the existing 20 ms raw-loss window, propagate the last
             * coherent angle/speed and retain bounded current.  If coherence
             * still has not returned and the last real speed is in the
             * six-step handover region, return directly to running six-step. */
            float held_entry_rpm = g.last_good_observer_valid ?
                rpm_from_omega(g.last_good_observer_omega_e) :
                g.control_speed_feedback_rpm;
            uint32_t entry_grace_samples =
                (held_entry_rpm < 0.0f) ? FM_HYBRID_NEG_RUN_ENTRY_GRACE_SAMPLES :
                                          FM_HYBRID_RUN_ENTRY_GRACE_SAMPLES;
            uint8_t fresh_run_entry =
                (g.run_entry_grace_count <= entry_grace_samples);
            uint32_t raw_loss_limit = fresh_run_entry ?
                                      entry_grace_samples :
                                      FM_HYBRID_RUN_RAW_LOSS_SAMPLES;
            uint8_t holding_last_good =
                (g.last_good_observer_valid &&
                 g.observer_unlock_count <= raw_loss_limit);
            if (holding_last_good) {
                g.forced_omega_e = g.last_good_observer_omega_e;
                g.control_theta = FM_WrapPi(g.control_theta +
                                            g.forced_omega_e * FM_CONTROL_TS_S);
                g.control_speed_feedback_rpm =
                    rpm_from_omega(g.last_good_observer_omega_e);
                float hold_iq = fresh_run_entry ?
                                FM_HYBRID_RUN_ENTRY_HOLD_IQ_A : 10.0f;
                g.t.iq_ref_a = FM_Clamp(g.t.iq_ref_a, -hold_iq, hold_iq);
            } else {
                float iq_slew = FM_HYBRID_HOLDOVER_IQ_SLEW_A_PER_S * FM_CONTROL_TS_S;
                if (g.t.iq_ref_a > iq_slew) g.t.iq_ref_a -= iq_slew;
                else if (g.t.iq_ref_a < -iq_slew) g.t.iq_ref_a += iq_slew;
                else g.t.iq_ref_a = 0.0f;
            }

#if FM_RUN_LOW_RPM_SIXSTEP_FALLBACK
            if (g.last_good_observer_valid &&
                g.observer_unlock_count >= raw_loss_limit) {
                float last_good_rpm =
                    rpm_from_omega(g.last_good_observer_omega_e);
                if (isfinite(last_good_rpm) &&
                    fabsf(last_good_rpm) <=
                        (FM_HYBRID_FOC_SPEED_FLOOR_RPM + 500.0f) &&
                    fabsf(g.t.target_rpm) >= FM_HYBRID_STOP_RPM &&
                    g.t.target_rpm * last_good_rpm > 0.0f) {
                    g.control_speed_feedback_rpm = last_good_rpm;
                    g.forced_omega_e = g.last_good_observer_omega_e;
                    trap_resume_sixstep_after_failed_foc(i_raw);
                    break;
                }
            }

#endif
            /* If low-speed angle becomes unobservable, reduce torque as the
             * existing holdover does. Eventually shut OFF the bridge rather
             * than apply indefinite torque using a blind rotor angle. */
#if !FM_RUN_LOW_RPM_SIXSTEP_FALLBACK
            if (g.observer_unlock_count > FM_RUN_OBSERVER_LOSS_FAULT_SAMPLES) {
                FastMotor_Stop(0xE1U); /* lost RUN observer; no six-step fallback */
                break;
            }
#endif

            if (run_feedback_valid) {
                if (g.observer_recatch_samples < FM_HYBRID_RUN_REACQUIRE_SAMPLES)
                    ++g.observer_recatch_samples;
                if (g.observer_recatch_samples >= FM_HYBRID_RUN_REACQUIRE_SAMPLES) {
                    /* Torque is already near zero: directly reseed to the real
                     * flux angle/speed after 1 ms of coherent proof. */
                    g.control_theta = raw_theta;
                    g.forced_omega_e = omega_from_rpm(flux_rpm);
                    g.obs.theta = g.control_theta;
                    g.obs.omega_e = g.forced_omega_e;
                    g.active_target_rpm = flux_rpm;
                    g.speed_int = 0.0f;
                    g.t.iq_ref_a = 0.0f;
                    seed_current_pi_for_frame(g.control_theta, i_raw,
                                              0.0f, 0.0f, g.last_v_fund);
                    g.observer_holdover_active = 0U;
                    g.observer_holdover_count = 0U;
                    g.observer_unlock_count = 0U;
                    g.observer_recatch_samples = 0U;
                    g.control_feedback_blend = 1.0f;
                    g.forced_observer_blend = 1.0f;
                    g.last_good_observer_theta = g.control_theta;
                    g.last_good_observer_omega_e = g.forced_omega_e;
                    g.last_good_observer_valid = 1U;
                }
            } else {
                g.observer_recatch_samples = 0U;
            }

#if FM_RUN_LOW_RPM_SIXSTEP_FALLBACK
            /* Six-step fallback still requires positive independent evidence
             * that the physical rotor is genuinely below the 1200-rpm boundary. */
            if (flux_speed_valid && fabsf(flux_rpm) < FM_HYBRID_FOC_EXIT_RPM) {
                if (g.run_low_speed_confirm_count <
                    FM_HYBRID_RUN_TO_SIXSTEP_CONFIRM_SAMPLES)
                    ++g.run_low_speed_confirm_count;
                if (g.run_low_speed_confirm_count >=
                    FM_HYBRID_RUN_TO_SIXSTEP_CONFIRM_SAMPLES) {
                    g.control_speed_feedback_rpm = flux_rpm;
                    g.control_theta = raw_theta;
                    g.forced_omega_e = omega_from_rpm(flux_rpm);
                    g.speed_command_pending = 0U;
                    hybrid_enter_sixstep_from_control();
                    break;
                }
            } else {
                g.run_low_speed_confirm_count = 0U;
            }
#else
            g.run_low_speed_confirm_count = 0U;
#endif
        }

        float measured_rpm = g.control_speed_feedback_rpm;

        /* Negative low-speed false-lock watchdog.  Build 26091607 could enter
         * RUN near -650 rpm, then sit physically locked while the coupled PLL
         * and flux model continued to report ~-600 rpm, ~0.23 V BEMF and the
         * speed PI remained pinned at the -8 A safety ceiling.  A real launch
         * crosses 750 rpm / 0.30 V quickly; a phantom lock does not.  Require
         * that bad signature to persist for 30 ms, then return to the proven
         * moving six-step recovery instead of energizing one rotor position
         * indefinitely. */
        uint8_t neg_false_lock_signature =
            (normal_torque_authority &&
             g.t.target_rpm <= -FM_HYBRID_NEG_FOC_RUN_MIN_RPM &&
             measured_rpm < 0.0f &&
             fabsf(measured_rpm) < FM_HYBRID_NEG_FALSE_LOCK_MAX_RPM &&
             fabsf(g.t.target_rpm - measured_rpm) >=
                 FM_HYBRID_NEG_FALSE_LOCK_MIN_ERR_RPM &&
             isfinite(g.obs.bemf_mag) &&
             g.obs.bemf_mag <= FM_HYBRID_NEG_FALSE_LOCK_MAX_BEMF_V &&
             fabsf(g.t.iq_ref_a) >= FM_HYBRID_NEG_FALSE_LOCK_MIN_IQ_A);

        if (neg_false_lock_signature) {
            if (g.neg_run_false_lock_count < FM_HYBRID_NEG_FALSE_LOCK_SAMPLES)
                ++g.neg_run_false_lock_count;
        } else {
            g.neg_run_false_lock_count = 0U;
        }
        g.neg_run_false_lock_active =
            (g.neg_run_false_lock_count > 0U) ? 1U : 0U;

        if (g.neg_run_false_lock_count >= FM_HYBRID_NEG_FALSE_LOCK_SAMPLES) {
#if FM_RUN_LOW_RPM_SIXSTEP_FALLBACK
            trap_resume_sixstep_after_failed_foc(i_raw);
#else
            FastMotor_Stop(0xE2U); /* phantom negative rotor lock: bridge off */
#endif
            break;
        }

        uint8_t requested_reverse =
            (normal_torque_authority &&
             fabsf(g.t.target_rpm) >= FM_HYBRID_STOP_RPM &&
             g.t.target_rpm * measured_rpm < 0.0f);
        uint8_t stop_requested =
            (fabsf(g.t.target_rpm) < FM_HYBRID_STOP_RPM);
        uint8_t low_speed = (normal_torque_authority && stop_requested &&
                             fabsf(measured_rpm) < FM_HYBRID_FOC_EXIT_RPM);
        uint8_t reverse_low_speed =
            (requested_reverse &&
             fabsf(measured_rpm) < FM_HYBRID_REVERSAL_TO_SIXSTEP_RPM);

        if (low_speed) {
            /* A true zero-speed command must not park the motor in an energized
             * six-step sector.  Coast from the already-low speed with the bridge
             * disabled; a later nonzero command restarts standstill HFI. */
            g.active_target_rpm = 0.0f;
            g.control_speed_feedback_rpm = 0.0f;
            g.t.iq_ref_a = 0.0f;
            g.if_iq_ref_a = 0.0f;
            g.speed_int = 0.0f;
            g.int_d = 0.0f;
            g.int_q = 0.0f;
            g.last_v_fund = zero;
            g.control_feedback_blend = 0.0f;
            g.forced_observer_blend = 0.0f;
            g.observer_phase_valid = 0U;
            g.t.active_estimator = FM_ESTIMATOR_NONE;
            g.t.launch_phase = 0U;
            FM_PWM_Disable();
            set_state(FM_STATE_IDLE);
            break;
        }
        if (reverse_low_speed) {
            hybrid_enter_sixstep_from_control();
            break;
        }

        if (normal_torque_authority) {
            float effective_target_rpm = g.t.target_rpm;
            /* Genuine signed target, including 60..2000 rpm. No floor. */

            if (measured_rpm < 0.0f) {
                /* Negative RUN stabilization.
                 *
                 * IMPORTANT CPU/liveness rule:
                 * 26090320 telemetry showed UART/telemetry gaps beginning almost
                 * exactly when negative speed crossed ~-1800 rpm.  Positive RUN
                 * stayed connected.  At that same threshold this branch began
                 * doing release-evidence tests, floating-point interpolation and
                 * release bookkeeping at the full 40-kHz control rate.  Measured
                 * ISR time increased from ~23.19 us positive to ~23.69..23.88 us
                 * negative on a 25-us period, starving the foreground UART/TLM
                 * services.
                 *
                 * The release decision only has a 10-ms qualification and
                 * 100-ms ramp, so it does NOT belong at 40 kHz.  Evaluate and
                 * update it on the existing 1-kHz speed-loop tick, cache the
                 * release factor/target ramp, and leave the 40-kHz path with
                 * only the cheap target integrator.  Positive RUN is untouched. */
                uint8_t speed_loop_tick =
                    ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U);

                if (speed_loop_tick) {
                    g.neg_run_quality_bits = 0U;
                    if (run_feedback_valid)
                        g.neg_run_quality_bits |= (1U << 0);
                    if (flux_speed_valid)
                        g.neg_run_quality_bits |= (1U << 1);
                    if (flux_rpm < 0.0f)
                        g.neg_run_quality_bits |= (1U << 2);
                    if (fabsf(measured_rpm) >= FM_HYBRID_NEG_RUN_RELEASE_MIN_RPM)
                        g.neg_run_quality_bits |= (1U << 3);
                    if (isfinite(g.obs.bemf_mag) &&
                        g.obs.bemf_mag >= FM_HYBRID_NEG_RUN_RELEASE_MIN_BEMF_V)
                        g.neg_run_quality_bits |= (1U << 4);
                    if (fabsf(phase_err) <= FM_HYBRID_NEG_RUN_RELEASE_MAX_PHASE_ERR_RAD)
                        g.neg_run_quality_bits |= (1U << 5);

                    uint8_t neg_release_evidence =
                        (g.neg_run_quality_bits == 0x3FU);

                    if (neg_release_evidence) {
                        uint32_t max_release_count =
                            FM_HYBRID_NEG_RUN_RELEASE_GOOD_SPEED_TICKS +
                            FM_HYBRID_NEG_RUN_RELEASE_RAMP_SPEED_TICKS;
                        if (g.neg_run_release_count < max_release_count)
                            ++g.neg_run_release_count;
                    } else {
                        /* Do not slam authority from a partially released
                         * 20..30 A limit back to 8 A because of one marginal
                         * 1-kHz observer sample.  Decay proof quickly but
                         * continuously; persistent bad evidence still returns
                         * to the safe floor. */
                        if (g.neg_run_release_count >
                            FM_HYBRID_NEG_RELEASE_DECAY_TICKS)
                            g.neg_run_release_count -=
                                FM_HYBRID_NEG_RELEASE_DECAY_TICKS;
                        else
                            g.neg_run_release_count = 0U;
                    }

                    if (g.neg_run_release_count >
                        FM_HYBRID_NEG_RUN_RELEASE_GOOD_SPEED_TICKS) {
                        g.neg_run_release_k = FM_Clamp(
                            (float)(g.neg_run_release_count -
                                    FM_HYBRID_NEG_RUN_RELEASE_GOOD_SPEED_TICKS) /
                            (float)FM_HYBRID_NEG_RUN_RELEASE_RAMP_SPEED_TICKS,
                            0.0f, 1.0f);
                    } else {
                        g.neg_run_release_k = 0.0f;
                    }

                    g.neg_run_target_ramp =
                        FM_HYBRID_NEG_RUN_SAFE_RAMP_RPM_PER_S +
                        g.neg_run_release_k *
                        (FM_HYBRID_SPEED_SETPOINT_RAMP_RPM_PER_S -
                         FM_HYBRID_NEG_RUN_SAFE_RAMP_RPM_PER_S);

                    float normal_limit = run_iq_limit_for_rpm(measured_rpm);
                    float run_limit =
                        FM_HYBRID_NEG_RUN_SAFE_IQ_A +
                        g.neg_run_release_k *
                        (normal_limit - FM_HYBRID_NEG_RUN_SAFE_IQ_A);
                    if (run_limit > normal_limit)
                        run_limit = normal_limit;
                    g.neg_run_iq_limit_a = run_limit;

                    speed_pi_step_limited(measured_rpm,
                                          g.active_target_rpm,
                                          run_limit);
                }

                ramp_target(effective_target_rpm, g.neg_run_target_ramp);
            } else {
                /* Proven positive path unchanged. */
                g.neg_run_quality_bits = 0U;
                g.neg_run_iq_limit_a = 0.0f;
                uint32_t run_unlock_samples = FM_HYBRID_RUN_IQ_UNLOCK_SAMPLES;
                float run_target_ramp =
                    (g.run_entry_grace_count < run_unlock_samples) ?
                    FM_HYBRID_RUN_ENTRY_SPEED_RAMP_RPM_PER_S :
                    FM_HYBRID_SPEED_SETPOINT_RAMP_RPM_PER_S;
                ramp_target(effective_target_rpm, run_target_ramp);
                if ((g.isr_count % FM_SPEED_LOOP_DIV) == 0U) {
                    float run_limit = run_iq_limit_for_rpm(measured_rpm);
                    if (g.run_entry_grace_count < run_unlock_samples) {
                        float k_unlock = FM_Clamp(
                            (float)g.run_entry_grace_count /
                            (float)run_unlock_samples, 0.0f, 1.0f);
                        run_limit = FM_HYBRID_CAPTURE_IQ_MAX_A +
                                    k_unlock *
                                    (run_limit - FM_HYBRID_CAPTURE_IQ_MAX_A);
                    }
                    speed_pi_step_limited(measured_rpm,
                                          g.active_target_rpm,
                                          run_limit);
                }
            }
        }

        g.handover_iq_ref_avg = g.t.iq_ref_a;
        g.handover_id_meas_avg = 0.0f;
        g.handover_iq_meas_avg = g.t.iq_ref_a;
        g.if_iq_ref_a = g.t.iq_ref_a;
        current_control(g.control_theta, i_raw, 0.0f, g.t.iq_ref_a, zero);
        break;
    }

    case FM_STATE_IDLE: {
        /* Bridge-off zero-command state.  This prevents the old ~8 A stalled
         * six-step current after a commanded stop.  A new nonzero command
         * performs a fresh HFI standstill acquisition before applying torque. */
        FM_PWM_Disable();
        g.last_v_fund = zero;
        g.t.iq_ref_a = 0.0f;
        g.if_iq_ref_a = 0.0f;
        g.active_target_rpm = 0.0f;
        g.control_speed_feedback_rpm = 0.0f;
        g.control_feedback_blend = 0.0f;
        g.forced_observer_blend = 0.0f;
        g.observer_phase_valid = 0U;
        g.t.active_estimator = FM_ESTIMATOR_NONE;
        g.t.launch_phase = 0U;

        if (fabsf(g.t.target_rpm) >= FM_HYBRID_STOP_RPM) {
            /* Sequential starts must behave like a power-on start.  Clear every
             * dynamic observer/six-step/handover state before HFI. */
            begin_fresh_hfi_start(i_raw);
        }
        break;
    }

    default:
        FastMotor_Stop(0xEEU);
        break;
    }


    fm_isr_timing_end(isr_entry_cycles);
}

static void fm_update_diagnostic_telemetry(void)
{
    /* Low-rate/diagnostic data. */
    g.t.tim1_moe = (TIM1->BDTR & TIM_BDTR_MOE) ? 1U : 0U;
    g.t.tim1_ccr1 = (uint16_t)TIM1->CCR1;
    g.t.tim1_ccr2 = (uint16_t)TIM1->CCR2;
    g.t.tim1_ccr3 = (uint16_t)TIM1->CCR3;

    g.t.hfi_angle_rad = g.hfi.theta;
    g.t.hfi_rpm = hfi_foc_rpm();
    g.t.hfi_error_a = g.hfi.error_a;
    g.t.hfi_saliency_a = g.hfi.saliency_scale_a;
    g.t.polarity_delta_theta_rad = g.hfi.polarity_unwrapped_delta;
    g.t.polarity_decision = g.hfi.polarity_decision;
    g.t.bemf_v = (g.t.state == FM_STATE_SLINGSHOT) ?
                 g.trap_bemf_vector_mag_cached : g.obs.bemf_mag;
    g.t.phase_error_rad = g.obs.phase_err;
    g.t.observer_locked = g.obs.locked;
    g.t.observer_lock_count = g.obs.lock_count;
    g.t.observer_unlock_count = g.observer_unlock_count;
    g.t.observer_polarity_pi = g.obs.polarity_pi;
    g.t.observer_polarity_resolved = g.obs.polarity_resolved;
    g.t.forced_voltage_to_rotor_lag_rad = g.obs.voltage_to_rotor_lag_rad;
    g.t.handover_seed_iq_a = g.handover_iq_start_a;
    g.t.firmware_build_id = FM_FIRMWARE_BUILD_ID;
    g.t.forced_observer_good_required =
        (g.t.state == FM_STATE_SLINGSHOT) ? FM_TRAP_ZC_LOCK_EVENTS :
                                            FM_HYBRID_SYNC_LOCK_SAMPLES;
    g.t.state6_entry_good_count = g.state6_entry_good_count;
    g.t.state6_entry_blend = g.state6_entry_blend;
    g.t.full_release_qualified = g.full_release_qualified;
    g.t.control_feedback_blend = g.control_feedback_blend;
    g.t.control_angle_rad = g.control_theta;
    g.t.control_speed_feedback_rpm = g.control_speed_feedback_rpm;
    g.t.if_iq_ref_a = g.if_iq_ref_a;
    g.t.if_raw_seed_id_a = g.handover_id_meas_avg;
    g.t.if_raw_seed_iq_a = (g.t.state == FM_STATE_RUN) ?
                           g.handover_iq_ref_avg : g.handover_iq_meas_avg;
    g.t.feedback_good_count = g.feedback_good_count;
    g.t.sling_phase = g.sling_phase;
    g.t.sling_attempt = g.sling_attempt;
    g.t.sling_axis_quadrature = g.sling_axis_quadrature;
    g.t.sling_seed_angle_rad = g.sling_hfi_seed_theta;
    g.t.sling_valid_samples = g.sling_valid_samples;
    g.t.if_pi_seeded = g.if_pi_seeded;
    g.t.coast_good_count = g.coast_good_count;
    g.t.catch_iq_limit_a = g.catch_iq_limit_a;
    g.t.observer_holdover_active = g.observer_holdover_active;
    g.t.observer_holdover_count = g.observer_holdover_count;
    g.t.last_good_observer_angle_rad = g.last_good_observer_theta;
    g.t.last_good_observer_rpm = rpm_from_omega(g.last_good_observer_omega_e);
    g.t.observer_angle_rad = g.obs.theta_meas_valid ?
                             hybrid_aligned_raw_theta() : g.control_theta;
    g.t.observer_rpm = rpm_from_omega(g.obs.flux_omega_e);
    g.t.hfi_omega_raw_rpm = rpm_from_omega(g.hfi.omega_e);
    g.t.hfi_bemf_angle_err_rad = g.obs.theta_meas_valid ?
        FM_WrapPi(hybrid_aligned_raw_theta() - hfi_foc_theta()) : 0.0f;
    g.t.hfi_to_foc_offset_rad = g.hfi_to_foc_offset;
    g.t.hfi_calibrated_angle_rad = hfi_foc_theta();
    g.t.hfi_cal_error_rad = 0.0f;
    g.t.hfi_cal_resultant = g.hfi.cal_resultant;
    g.t.hfi_cal_verify_count = g.hfi.cal_verify_count;
    g.t.vbus_v = g.vbus;
    {
        float v2 = g.t.vd_cmd_v * g.t.vd_cmd_v +
                   g.t.vq_cmd_v * g.t.vq_cmd_v;
        if (isfinite(v2) && v2 >= 0.0f)
            g.t.vfund_mag_v = sqrtf(v2);
    }

    g.t.forced_angle_rad = g.forced_theta;
    /* In the trapezoid test this is the commanded mechanical-equivalent
     * six-step commutation speed.  It is NEVER treated as measured rotor speed. */
    g.t.forced_rpm = (g.t.state == FM_STATE_OBSERVER_HANDOVER ||
                      g.t.state == FM_STATE_BEMF_BOOST ||
                      g.t.state == FM_STATE_RUN) ?
                     g.control_speed_feedback_rpm : g.active_target_rpm;
    g.t.forced_voltage_v = g.forced_voltage_v;
    g.t.forced_observer_good_count = g.forced_observer_good_count;
    g.t.forced_accel_rpm_s = g.forced_accel_rpm_s;
    g.t.forced_observer_blend = g.forced_observer_blend;

    if (g.t.active_estimator == FM_ESTIMATOR_HFI) {
        g.t.electrical_angle_rad = hfi_foc_theta();
        g.t.rpm = hfi_foc_rpm();
    } else if (g.t.active_estimator == FM_ESTIMATOR_HFI_SEED) {
        g.t.electrical_angle_rad = g.control_theta;
        g.t.rpm = g.t.sling_rpm;
    } else if (g.t.active_estimator == FM_ESTIMATOR_BLEND) {
        g.t.electrical_angle_rad = g.control_theta;
        /* The I-f trajectory is a control trajectory, not measured rotor
         * speed.  Until the observer has actually qualified, report zero
         * mechanical RPM rather than the old 1333-rpm ghost value. */
        g.t.rpm = (g.feedback_good_count >= FM_HYBRID_SYNC_LOCK_SAMPLES) ?
                  g.control_speed_feedback_rpm : 0.0f;
    } else if (g.t.active_estimator == FM_ESTIMATOR_BEMF) {
        g.t.electrical_angle_rad = g.control_theta;
        g.t.rpm = g.control_speed_feedback_rpm;
    } else {
        g.t.electrical_angle_rad = hfi_foc_theta();
        g.t.rpm = 0.0f;
    }

    g.t.isr_cycles_last = g.isr_cycles_last;
    g.t.isr_cycles_max = g.isr_cycles_max;
    g.t.isr_overrun_count = g.isr_overrun_count;
    g.t.isr_interval_cycles_last = g.isr_interval_cycles_last;
    g.t.isr_interval_cycles_max = g.isr_interval_cycles_max;
    g.t.isr_late_count = g.isr_late_count;
    g.t.observer_phase_valid = g.observer_phase_valid;
    g.t.raw_flux_rpm = rpm_from_omega(g.obs.flux_omega_e);
    g.t.active_target_rpm = g.active_target_rpm;
    g.t.rejoin_angle_error_rad = g.rejoin_angle_error_rad;
    g.t.sixstep_bemf_sync = g.trap_bemf_sync_active;
    g.t.sixstep_measured_rpm = g.trap_measured_rpm;
    g.t.sixstep_sector = g.trap_sector;
    g.t.sixstep_sector_phase = g.trap_sector_phase;
    g.t.sixstep_speed_error_rpm = g.trap_sync_speed_err_rpm;
    g.t.sixstep_phase_error_rad = g.trap_sync_phase_err_rad;
    g.t.sixstep_commutation_count = g.trap_commutation_count;
    g.t.sixstep_sync_good_count = g.trap_bemf_sync_good_count;
    g.t.sixstep_sync_bad_count = g.trap_bemf_sync_bad_count;

    g.t.trap_command_rpm = g.trap_command_rpm;
    g.t.hfi_polarity_proven = hfi_polarity_is_proven();
    g.t.foc_capture_good_count = g.foc_capture_good_count;
    g.t.foc_capture_entry_samples = g.bemf_boost_entry_sample;
    g.t.foc_capture_peak_bemf_v = g.foc_capture_peak_bemf_v;
    g.t.foc_retry_cooldown_samples = g.trap_foc_retry_cooldown_samples;
    g.t.trap_field_slip_rpm =
        (g.t.state == FM_STATE_SLINGSHOT) ? g.t.trap_field_slip_rpm : g.foc_entry_slip_rpm;
    g.t.foc_entry_rpm = g.foc_entry_rpm;
    g.t.foc_entry_command_rpm = g.foc_entry_command_rpm;
    g.t.foc_entry_slip_rpm = g.foc_entry_slip_rpm;
    g.t.foc_entry_bemf_v = g.foc_entry_bemf_v;
    g.t.foc_entry_angle_error_rad = g.foc_entry_angle_error_rad;
    g.t.foc_entry_speed_error_rpm = g.foc_entry_speed_error_rpm;
    g.t.foc_entry_phase_current_a = g.foc_entry_phase_current_a;
    g.t.neg_run_admission_ready = g.neg_run_admission_ready;
    g.t.neg_run_false_lock_count = g.neg_run_false_lock_count;
    g.t.neg_run_false_lock_active = g.neg_run_false_lock_active;
    g.t.neg_run_quality_bits = g.neg_run_quality_bits;
    g.t.neg_run_iq_limit_a = g.neg_run_iq_limit_a;
    g.t.neg_run_release_k = g.neg_run_release_k;
    g.t.neg_run_release_count = g.neg_run_release_count;

    g.t.sixstep_floating_bemf_v = g.trap_floating_bemf_v;
    g.t.sixstep_floating_current_a = g.trap_floating_current_a;
    g.t.sixstep_zc_rpm = g.trap_zc_rpm;
    g.t.sixstep_zc_holdover = g.trap_zc_holdover_sectors;

    /* Bridge-off IDLE owns no live rotor estimator. Never present the previous
     * flux state as a current RPM measurement. */
    if (g.t.state == FM_STATE_IDLE) {
        g.t.rpm = 0.0f;
        g.t.observer_rpm = 0.0f;
        g.t.raw_flux_rpm = 0.0f;
        g.t.bemf_v = 0.0f;
        g.t.phase_error_rad = 0.0f;
        g.t.observer_locked = 0U;
        g.t.observer_phase_valid = 0U;
        g.t.last_good_observer_rpm = 0.0f;
        g.t.control_speed_feedback_rpm = 0.0f;
        g.t.sixstep_measured_rpm = 0.0f;
        g.t.sixstep_bemf_sync = 0U;
    }
}

/* Compatibility path if another module explicitly routes ADC2 through HAL.
 * The normal ADC1_2 vector does not use this path. */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != NULL && hadc->Instance == ADC2)
        FastMotor_ADC2InjectedIRQ(DWT->CYCCNT);
}

void FastMotor_Service(void)
{
    if (g.t.state == FM_STATE_FAULT || g.t.state == FM_STATE_BOOT) return;
    static uint32_t last_fault_poll_ms = 0U;
    uint32_t now = HAL_GetTick();

    /* Runtime telemetry fields are now mirrored cheaply at 1 kHz inside the
     * control callback from values already computed for control.  Do not run
     * the old large foreground diagnostic rebuild at all. */

    if ((uint32_t)(now - g.last_vbus_update_ms) >= FM_VBUS_UPDATE_MS) {
        g.last_vbus_update_ms = now;
        update_vbus_measurement();
        if (g.t.vbus_adc_valid && g.vbus_uv_count >= FM_VBUS_UV_SAMPLES) {
            FastMotor_Stop(40U); /* real measured bus undervoltage */
            return;
        }
        if (g.t.vbus_adc_valid && g.vbus_ov_count >= FM_VBUS_OV_SAMPLES) {
            FastMotor_Stop(42U); /* current-hardware DC-bus overvoltage */
            return;
        }
    }

    if ((uint32_t)(now - last_fault_poll_ms) >= 10U) {
        last_fault_poll_ms = now;
        STSPIN_Status_t s = {0};
        if (STSPIN_GetStatus(&s) != HAL_OK) {
            FastMotor_Stop(20U);
        } else {
            g.t.stspin_status_raw = s.raw;
            if (s.any_fault)
                FastMotor_Stop(0x100U | s.raw);
        }
        if (g.t.state == FM_STATE_FAULT) return;
    }
    fm_v13_foreground_service();
}

void FastMotor_Stop(uint32_t fault_code)
{
    g.handover_observer_blend_start = 0.0f;
    g.t.fault = fault_code;
    g.t.state = FM_STATE_FAULT;
    g.t.active_estimator = FM_ESTIMATOR_NONE;
    g.t.polarity_probe_iq_a = 0.0f;
    g.t.sling_iq_cmd_a = 0.0f;
    g.t.iq_ref_a = 0.0f;
    g.last_v_fund.alpha = 0.0f;
    g.last_v_fund.beta = 0.0f;
    FM_PWM_Disable();
}

void FastMotor_SetTargetRPM(float rpm)
{
    /* Signed target is intentional: low-speed direction changes are routed
     * through the six-step mode, while high-speed FOC can apply signed torque
     * to decelerate toward that region. */
    if (!isfinite(rpm))
        return;

    float next = FM_Clamp(rpm, -FM_MAX_RPM, FM_MAX_RPM);
    uint8_t changed = (fabsf(next - g.t.target_rpm) >= 0.5f) ? 1U : 0U;
    g.t.target_rpm = next;

    /* A changed command always gets consumed by RUN. Re-sending the same
     * target also becomes actionable when feedback is currently invalid; this
     * is useful for recovery instead of silently accepting a command behind a
     * stale RPM estimate. */
    if (changed ||
        (g.t.state == FM_STATE_RUN && g.control_feedback_blend < 0.5f)) {
        g.speed_command_pending = 1U;
    }
}

const FM_Telemetry *FastMotor_GetTelemetry(void)
{
    return &g.t;
}
