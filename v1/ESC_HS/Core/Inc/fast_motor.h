#ifndef FAST_MOTOR_H
#define FAST_MOTOR_H

#include <stdint.h>

typedef enum {
    FM_STATE_BOOT = 0,
    FM_STATE_BEEP,
    FM_STATE_HFI_SCAN,          /* stationary HFI-only saliency-axis search */
    FM_STATE_HFI_CONVERGE,      /* stationary HFI PLL acquisition */
    FM_STATE_SLINGSHOT,         /* HFI-seeded true six-step/trapezoidal startup */
    FM_STATE_BEMF_BOOST,        /* phase-locked current-mode FOC; Id release before speed PI */
    FM_STATE_OBSERVER_HANDOVER, /* six-step -> I-f FOC -> continuous shadow-observer phase lock */
    FM_STATE_RUN,
    FM_STATE_FAULT,
    FM_STATE_IDLE = 9          /* zero-command bridge-off coast/standstill; nonzero command restarts HFI */
} FM_State;

typedef enum {
    FM_ESTIMATOR_NONE = 0,
    FM_ESTIMATOR_HFI  = 1,
    FM_ESTIMATOR_BLEND = 2,
    FM_ESTIMATOR_BEMF = 3,
    FM_ESTIMATOR_HFI_SEED = 4 /* frozen HFI-seeded angle during the few-ms slingshot impulse */
} FM_EstimatorSource;

typedef struct {
    FM_State state;
    FM_EstimatorSource active_estimator;
    float rpm;
    float target_rpm;
    float electrical_angle_rad;
    float bemf_v;
    float phase_error_rad;
    float id_a;
    float iq_a;
    float iq_ref_a;
    float iu_a;
    float iv_a;
    float iw_a;
    /* V13 Lean shadow diagnostic only (never used for control or safety). */
    float v13_iu_a, v13_iv_a, v13_iw_a;
    float v13_noevent_iu_a, v13_noevent_iv_a, v13_noevent_iw_a;
    float v13_projection_iu_a, v13_projection_iv_a, v13_projection_iw_a;
    float v13_alpha, v13_base_alpha, v13_confidence;
    float v13_voltage_evidence, v13_step_latch;
    uint32_t v13_cycles_last, v13_cycles_max, v13_samples;
    /* Estimate of non-interrupt V13 CPU cycles: DWT wall cycles less ISR cycles. */
    uint32_t v13_cpu_cycles_last, v13_cpu_cycles_max;
    uint32_t v13_rejected_samples;
    uint32_t v13_missed_samples;
    uint32_t v13_fast_overruns;
    uint32_t v13_gap_resyncs;
    uint32_t v13_reset_count, v13_continuity_invalid;
    uint32_t v13_capture_state, v13_capture_count;
    float v13_research_alpha;
    float v13_40k_alpha_current, v13_40k_beta_current;
    float v13_40k_gain;
    uint32_t v13_40k_cycles_last, v13_40k_cycles_max, v13_40k_samples, v13_40k_guard_events, v13_40k_gain_age_ticks;
    uint32_t v13_age_ticks_last, v13_age_ticks_max;
    uint32_t observer_run_cycles_last, observer_run_cycles_max;
    uint32_t observer20k_mode; /* 0 other states; 1 RUN raw 20kHz; 2 RUN raw 40kHz control */
    uint32_t observer20k_updates; /* actual observer integrations during RUN */
    uint32_t observer20k_predicts; /* 25us pure angle-prediction ISR ticks */
    uint32_t v13_fast_mode; /* 0: startup/inactive, 1: 20kHz ISR, 2: 2kHz fallback */
    float v13_observer_blend;
    uint8_t v13_voltage_valid, v13_reference_complete, v13_shadow_decimation;
    float hfi_angle_rad;
    float hfi_rpm;
    float hfi_error_a;
    float hfi_saliency_a;
    float hfi_omega_raw_rpm;
    float polarity_delta_theta_rad;
    float polarity_probe_iq_a;
    uint8_t polarity_decision; /* 0 pending, 1 direct after coast proof, 2 +pi after reverse coast, 3 stronger probe retry, 5 failed */
    float vbus_v;
    float observer_angle_rad;
    float observer_rpm;
    float hfi_bemf_angle_err_rad;
    float vd_cmd_v;
    float vq_cmd_v;
    float vfund_mag_v;
    float hfi_vq_limit_v;
    uint8_t observer_motion_coherent;
    uint8_t launch_phase; /* 0 HFI acquire, 1 six-step, 2 forced-angle FOC capture, 3 observer FOC, 5 RUN */
    /* HFI carrier-separation diagnostics.  HFI angle tracking uses only the
     * high-frequency residual i_raw - i_notched, not the fundamental current. */
    float hfi_carrier_iq_a;
    float hfi_carrier_mag_a;
    float hfi_err_norm;

    /* Legacy channel names retained for telemetry-file compatibility.  In this
     * build these carry HFI/BEMF qualification data where noted; there is no
     * forced rotor-angle startup state. */
    float forced_angle_rad;
    float forced_rpm;
    float forced_voltage_v;
    uint32_t forced_observer_good_count;
    float forced_accel_rpm_s;
    float forced_observer_blend;

    /* HFI mapping / legacy known-angle calibration diagnostics.  Runtime does
     * not command a known rotor angle; angle sign is a fixed commissioned value. */
    float align_expected_angle_rad;
    float hfi_to_foc_offset_rad;
    float hfi_calibrated_angle_rad;
    float hfi_cal_error_rad;
    float hfi_cal_resultant;
    uint32_t hfi_cal_verify_count;
    /* Legacy two-point fields remain in the wire protocol.  The live mapping is
     * theta_foc = configured_sign * theta_hfi + offset/branch correction. */
    float hfi_angle_sign;
    float hfi_cal_raw0_rad;
    float hfi_cal_raw1_rad;
    float hfi_cal_delta_raw_rad;
    float hfi_cal_model_error_rad;

    /* Power-stage diagnostics used to prove that a commanded voltage really
     * reaches TIM1 / the STSPIN gate driver during the launch. */
    uint8_t tim1_moe;
    uint16_t tim1_ccr1;
    uint16_t tim1_ccr2;
    uint16_t tim1_ccr3;
    uint8_t stspin_status_raw;

    /* CORDIC mixed-function self-test.  This specifically tests PHASE ->
     * COS/SIN switching, which was corrupting the old Park/InvPark transforms. */
    uint8_t cordic_selftest_ok;
    float cordic_test_sin45;
    float cordic_test_cos45;
    uint16_t vbus_adc_counts;
    uint8_t vbus_adc_valid;

    /* Current-sense diagnostics.  raw_* are the latest injected JDR counts;
     * offset_* are the injected-path startup median offsets. */
    uint16_t raw_iu_counts;
    uint16_t raw_iv_counts;
    uint16_t raw_iw_counts;
    uint16_t offset_iu_counts;
    uint16_t offset_iv_counts;
    uint16_t offset_iw_counts;
    float phase_current_sum_a;
    float phase_current_peak_a;
    uint32_t overcurrent_count;
    uint32_t current_sum_bad_count;

    /* HFI startup diagnostic: leaky convergence confidence counter. */
    uint32_t hfi_converge_count;

    uint8_t observer_locked;
    uint32_t observer_lock_count;
    uint32_t observer_unlock_count;
    uint8_t observer_polarity_pi;
    uint8_t observer_polarity_resolved;
    float forced_voltage_to_rotor_lag_rad;
    float handover_seed_iq_a;

    /* Build / state-6 entry proof.  These make stale-binary mistakes obvious
     * in the PC log and prove that full observer release was completed before
     * current-control handover. */
    uint32_t firmware_build_id;
    uint32_t forced_observer_good_required;
    uint32_t state6_entry_good_count;
    float state6_entry_blend;
    uint8_t full_release_qualified;

    /* Estimator-authority / HFI-start D/Q diagnostics. */
    float control_feedback_blend;
    float control_angle_rad;
    float control_speed_feedback_rpm;
    float if_iq_ref_a;
    uint32_t feedback_good_count;

    /* Legacy I-f PI-capture fields retained for protocol compatibility; unused
     * by the HFI-closed-loop startup path. */
    float if_raw_seed_id_a;
    float if_raw_seed_iq_a;
    uint8_t if_pi_seeded;

    /* Legacy coast fields plus real-observer holdover diagnostics. */
    uint32_t coast_good_count;
    float catch_iq_limit_a;
    uint8_t observer_holdover_active;
    uint32_t observer_holdover_count;
    float last_good_observer_angle_rad;
    float last_good_observer_rpm;

    /* Slingshot / zero-voltage-vector startup diagnostics. */
    uint8_t sling_phase;          /* trapezoid test: commutation sector + 1 (1..6) */
    uint8_t sling_attempt;       /* always 1 in the coherent one-direction launch */
    uint8_t sling_axis_quadrature;/* retained protocol field; 0 in this build */
    float sling_seed_angle_rad;
    float sling_iq_cmd_a;
    float sling_bemf_v;
    float sling_rpm;
    float sling_angle_rad;
    uint32_t sling_valid_samples;
    float sling_peak_phase_current_a;
    float sling_peak_iq_a;
    float sling_peak_vq_v;

    /* Hard-real-time diagnostics for the 40 kHz ADC/control callback.  Cycle
     * counts come directly from Cortex-M4 DWT->CYCCNT so UART/plotting cannot
     * hide whether the control ISR is missing its 25 us budget. */
    uint32_t isr_cycles_last;
    uint32_t isr_cycles_max;
    uint32_t isr_overrun_count;
    uint32_t isr_interval_cycles_last;
    uint32_t isr_interval_cycles_max;
    uint32_t isr_late_count;

    /* Additional acquisition diagnostics used by the drone crossover build. */
    uint8_t observer_phase_valid;
    float raw_flux_rpm;
    float active_target_rpm;
    float rejoin_angle_error_rad;
    uint8_t sixstep_bemf_sync;
    float sixstep_measured_rpm;
    uint8_t sixstep_sector;
    float sixstep_sector_phase;
    float sixstep_speed_error_rpm;
    float sixstep_phase_error_rad;
    uint32_t sixstep_commutation_count;
    uint32_t sixstep_sync_good_count;
    uint32_t sixstep_sync_bad_count;

    /* Build 26091604/05 startup/catch diagnostics.  These are intentionally
     * telemetry-only and do not participate in the control law. */
    float trap_command_rpm;
    uint8_t trap_lock_confident;
    uint32_t trap_locked_commutations;
    uint8_t early_foc_window;
    float early_foc_angle_error_rad;
    float early_foc_speed_error_rpm;
    uint8_t pos_slip_limit_active;
    float pos_slip_limit_rpm;
    uint8_t trap_flux_available;
    uint8_t trap_bemf_theta_valid;
    uint8_t hfi_polarity_proven;
    uint32_t foc_capture_good_count;
    uint32_t foc_capture_entry_samples;
    float foc_capture_peak_bemf_v;
    uint32_t foc_retry_cooldown_samples;

    /* Build 26091605 positive-catch diagnostics. */
    float trap_field_slip_rpm;
    uint8_t positive_catch_slip_ok;
    uint32_t positive_catch_block_bits;
    float foc_entry_rpm;
    float foc_entry_command_rpm;
    float foc_entry_slip_rpm;
    float foc_entry_bemf_v;
    float foc_entry_angle_error_rad;
    float foc_entry_speed_error_rpm;
    float foc_entry_phase_current_a;

    /* Build 26091607 direction-symmetric topology-boundary diagnostics. */
    uint8_t handover_signed_motion_ok;
    uint8_t handover_extra_proof_ok;
    uint8_t handover_polarity_pi_latched;
    float foc_entry_measured_rpm;
    float foc_entry_flux_rpm;
    float foc_entry_id_a;
    float foc_entry_iq_a;
    float topology_flux_phase_error_rad;
    float topology_flux_rpm;
    uint8_t first_foc_sample_valid;
    float first_foc_flux_rpm;
    float first_foc_phase_error_rad;

    /* Build 26091701 negative false-lock / release diagnostics. */
    uint8_t neg_run_admission_ready;
    uint32_t neg_run_false_lock_count;
    uint8_t neg_run_false_lock_active;
    uint32_t neg_run_quality_bits;
    float neg_run_iq_limit_a;
    float neg_run_release_k;
    uint32_t neg_run_release_count;

    /* Deprecated reconstructed floating-phase diagnostics; forced to zero in
     * build 26090310.  Closed-loop six-step uses raw_flux_rpm /
     * sixstep_measured_rpm and sixstep_bemf_sync instead. */
    float sixstep_floating_bemf_v;
    float sixstep_floating_current_a;
    float sixstep_zc_rpm;
    uint8_t sixstep_zc_holdover;

    uint32_t fault;
} FM_Telemetry;

void FastMotor_Init(void);
void FastMotor_Service(void);
void FastMotor_V13CaptureArm(void);
void FastMotor_ADC2InjectedIRQ(uint32_t irq_entry_cycles);
void FastMotor_Stop(uint32_t fault_code);
void FastMotor_SetTargetRPM(float rpm);
const FM_Telemetry *FastMotor_GetTelemetry(void);

#endif
