#ifndef FAST_MOTOR_CONFIG_H
#define FAST_MOTOR_CONFIG_H
/* Build 26101703: 20kHz full V13 SHADOW, 20kHz RAW RUN observer. */

/* BUILD 26101621: 40 kHz proven raw RUN observer and FOC are unchanged.
 * Full V13 is 20 kHz RUN-only diagnostic in a bounded foreground SPSC FIFO.
 * No fuzzy equations, observer changes, allocations, busy waits or telemetry
 * output inside the 40 kHz ADC interrupt. Drop V13 frames rather than stall. */
/* Fast-mode experiment: full original V13 at every second 40kHz ADC callback,
 * after motor reaches RUN. It NEVER touches current PI/HFI/protections.
 * Automatic fail-closed disabling after repeated >25us ISR durations; the
 * last tested 2kHz foreground mode takes over in that case. */
#define FM_V13_FAST_20KHZ_ENABLE       0U
#define FM_V13_STAGGER_OBSERVER_20KHZ  0U
#define FM_V13_FAST_DIV                2U
#define FM_V13_FAST_MAX_OVERRUNS       3U /* conservative guard; 20kHz may fall back */
#define FM_V13_FAST_BLEND_RPM_BEGIN    1500.0f
#define FM_V13_FAST_BLEND_RPM_END      6000.0f
#define FM_V13_FAST_BLEND_MAX         0.12f /* observer-only; never current PI */
#define FM_V13_FAST_WARMUP_UPDATES    200U /* 10ms to avoid takeover transient */
#define FM_V13_SHADOW_ENABLED          0U
#define FM_V13_SHADOW_DECIMATION       10U /* original V13 remains exact 4kHz continuous */
#define FM_V13_RESEARCH_NOEVENT       0U /* 0: disable duplicate research V13 computation */
#define FM_V13_40K_CACHED_OUTPUT      0U /* 40kHz RESEARCH EMA using V13's cached 4kHz gain; not frozen V13 */
#define FM_V13_40K_TIMING_DIV         128U /* DWT only once every 128 control ticks */
#define FM_V13_SNAPSHOT_PHASE          9U /* alternate 9/29, away from 40-tick speed PI */
#define FM_V13_RUN_OBSERVER_ASSIST     0U /* MUST remain shadow-only */
#define FM_V13_RUN_OBSERVER_MAX_RPM    500.0f
#define FM_V13_RUN_OBSERVER_FULL_RPM   200.0f
#define FM_V13_RUN_OBSERVER_MAX_BLEND  0.15f
#define FM_V13_MAX_SAMPLE_AGE_TICKS    60U /* 1.5 ms @ 40 kHz */
#define FM_RUN_LOW_RPM_SIXSTEP_FALLBACK 0U
#define FM_RUN_OBSERVER_LOSS_FAULT_SAMPLES 4000U /* 100 ms; gate OFF, not blind torque */

/* A/B experiment: raw-current RUN observer at 20 kHz. Startup, HFI,
 * six-step, handover and BEMF BOOST keep their proven rates. All FOC PI,
 * protection and PWM work remains at 40kHz; V13 is completely disconnected.
 * Setting this to 0 restores the baseline raw-current 40kHz RUN observer.
 * This intentionally does NOT alter any observer gain or threshold values. */
/* Full original V13 on opposite 40k ISR to RUN observer updates.
 * Strictly shadow-only. No V13 output is ever used for control/observer.
 * Automatic workload guard does not alter PWM, PI or safety protections. */
#define FM_V13_20K_SHADOW_TEST         1U
#define FM_V13_20K_MAX_OVERRUNS       4U
#define FM_RUN_RAW_OBSERVER_20KHZ       1U

/* ========================= BOARD / MOTOR ========================= */
#define FM_PWM_HZ                    40000.0f
#define FM_CONTROL_TS_S              (1.0f / FM_PWM_HZ)
#define FM_POLE_PAIRS                7.0f

/* Starting motor model for A2212/5T. Re-measure these for best observer tuning. */
/* A2212/5T 2450KV starting phase model.  The previous 60 mOhm / 30 uH
 * model was inconsistent with the successful six-step and 4 kHz HFI current
 * seen on this exact motor.  Use a deliberately conservative midpoint phase
 * resistance and the HFI-derived high-KV inductance as the observer seed.
 * Final flight calibration should replace R with a measured phase value. */
#define FM_PHASE_R_OHM               0.030f
#define FM_PHASE_L_H                  5.0e-6f

/* Observer-only effective phase model.  With the new x5 current-sense hardware
 * the previously successful x10-firmware/x5-hardware run revealed that the
 * voltage/flux observer behaved as if the current terms were half their true
 * value.  Preserve truthful current telemetry/control while reproducing that
 * observer model explicitly.  This is equivalent to 15 mOhm / 2.5 uH. */
#define FM_OBSERVER_PHASE_R_OHM      (0.50f * FM_PHASE_R_OHM)
#define FM_OBSERVER_PHASE_L_H        (0.50f * FM_PHASE_L_H)

/* A2212/5T 2450KV outrunner starting observer constants.  Published A2212
 * variants use 14 rotor poles (7 pole pairs).  2450 KV corresponds to about
 * 0.455 mWb PM flux linkage under the usual sinusoidal line-line convention.
 * R and L vary between manufacturers/clones and should still be measured on
 * the actual motor for final flight tuning. */
#define FM_MOTOR_KV_RPM_PER_V        2450.0f
#define FM_MOTOR_FLUX_LINKAGE_WB        0.000455f

/* 12-bit ADC, 3.3 V, x5 current-sense amplifier, 5 mOhm shunt.
 * Hardware was changed from x10 to x5 on 2026-08-27 to double analog current
 * headroom.  Keep this scale synchronized with the physical resistor network. */
#define FM_CURRENT_SENSE_GAIN         5.0f
#define FM_CURRENT_SHUNT_OHM          0.005f
#define FM_CURRENT_A_PER_COUNT \
    (3.3f / (4095.0f * FM_CURRENT_SENSE_GAIN * FM_CURRENT_SHUNT_OHM))
/* x5 + 5 mOhm gives about 32.23 mA/count and roughly +/-66 A analog range.
 * Telemetry/protection remain in REAL amps.  The higher limits below explicitly
 * reproduce the ~20-24 A physical torque level that the earlier mis-scaled run
 * was actually using, while keeping significant headroom below ADC saturation. */
#define FM_PHASE_CURRENT_TRIP_A             55.0f  /* higher sustained phase-current ceiling; 4 samples = 100 us */
#define FM_PHASE_CURRENT_TRIP_SAMPLES        4U    /* 100 us at 40 kHz */
#define FM_PHASE_CURRENT_HARD_TRIP_A        60.0f  /* one-sample hard ceiling; preserves margin below x5 ADC range */
#define FM_PHASE_CURRENT_HARD_TRIP_SAMPLES    1U    /* immediate real-current trip; x5 sensing leaves ample ADC headroom */
#define FM_ADC_RAIL_MARGIN_COUNTS             8U    /* catches ADC/opamp saturation */
#define FM_CURRENT_SUM_MAX_A                  3.0f  /* generous sensor-coherence gate */
#define FM_CURRENT_SUM_BAD_SAMPLES            8U

/* Real DC-bus measurement.  The original board/project uses PF0/ADC1_IN10
 * with a nominal 20k/1k divider, i.e. 21:1. */
#define FM_ADC_VREF_V                         3.3f
#define FM_ADC_FULL_SCALE_COUNTS           4095.0f
#define FM_VBUS_DIVIDER_RATIO                21.0f
#define FM_VBUS_VOLTS_PER_COUNT \
    (FM_ADC_VREF_V / FM_ADC_FULL_SCALE_COUNTS * FM_VBUS_DIVIDER_RATIO)
#define FM_VBUS_NOMINAL_V                    13.0f  /* fallback only if ADC invalid */
#define FM_VBUS_FILTER_ALPHA                  0.15f
#define FM_VBUS_UPDATE_MS                      2U
#define FM_VBUS_ADC_TIMEOUT_MS                 1U
#define FM_VBUS_STARTUP_SAMPLES               16U
#define FM_VBUS_VALID_MIN_V                    5.0f
#define FM_VBUS_VALID_MAX_V                   65.0f
#define FM_VBUS_UV_TRIP_V                      7.5f
#define FM_VBUS_UV_SAMPLES                     5U
#define FM_VBUS_OV_TRIP_V                     52.0f  /* CURRENT IRFS7530 60-V hardware: do not approach 60 VDC; raise only after HV power-stage upgrade */
#define FM_VBUS_OV_SAMPLES                      2U    /* 4 ms at 2-ms VBUS service cadence */

/* Clock configuration used by this project:
 * SYSCLK/PCLK = 170 MHz. ADC1/ADC2 use synchronous PCLK/4 = 42.5 MHz.
 * TIM1 center-aligned ARR=2124 therefore gives 40 kHz PWM/control timing. */

/* ========================= STARTUP SOUND ========================= */
#define FM_BEEP_HZ                   1200.0f
#define FM_BEEP_V                    0.05f
#define FM_BEEP_MS                   220U

/* ========================= DIRECT CLOSED-LOOP HFI START =========================
 * Runtime startup does NOT align the rotor and does NOT create a synthetic/fake
 * rotating angle.  The 4 kHz injected signal is used to estimate rotor saliency
 * angle at standstill; that HFI angle is then the actual Park/InvPark feedback
 * while the current and speed loops accelerate the motor from (near) zero.
 *
 * IMPORTANT for a drone ESC: FM_HFI_RUNTIME_ANGLE_SIGN is a fixed commissioning
 * constant, not something re-learned by forcing the rotor to known angles every
 * boot.  +1 matches the phase/current convention of this project.  If a bench
 * log proves raw HFI angle runs backwards, change it to -1 once and keep it as
 * a product/configuration constant.  This build starts from the direct HFI
 * branch and resolves the remaining pi ambiguity at standstill with a brief
 * HFI-tracked +Iq polarity probe before six-step is allowed to start.  The dedicated slingshot stage may then use a bounded
 * HFI-seeded rotating FOC frame to create enough BEMF for observer capture. */
#define FM_HFI_HZ                         4000.0f
#define FM_HFI_V                             0.18f
/* At a 30 V bus, 0.18 V is only 0.6% modulation and HFI convergence became
 * intermittent (fault 31). Keep the old 0.18 V floor at normal bus voltage,
 * but request at least 1% of VBUS at high voltage, capped at 0.30 V. */
#define FM_HFI_MIN_EFFECTIVE_DUTY             0.010f
#define FM_HFI_MAX_EFFECTIVE_V                0.30f
#define FM_HFI_DEMOD_LPF_HZ                 160.0f
#define FM_HFI_ERROR_SIGN                     1.0f
#define FM_HFI_RUNTIME_ANGLE_SIGN              1.0f
#define FM_HFI_RUNTIME_FOC_OFFSET_RAD          0.0f

/* Stationary HFI-only saliency scan.  No fundamental alignment vector is
 * applied: the rotor is free and the bridge outputs only the HF probe. */
#define FM_HFI_SCAN_ANGLE_COUNT                 8U
#define FM_HFI_SCAN_SETTLE_SAMPLES             40U
#define FM_HFI_SCAN_MEASURE_SAMPLES           120U
#define FM_HFI_SCAN_IDLE_SAMPLES               40U
#define FM_HFI_MIN_SALIENCY_A                 0.012f

/* Current feedback during HFI uses a 4 kHz notch.  Park/InvPark current control
 * therefore sees the fundamental current while the HFI demodulator sees the
 * complementary carrier residual. */
#define FM_HFI_NOTCH_COS_W0              0.8090169944f
#define FM_HFI_NOTCH_R                   0.92f
#define FM_HFI_NOTCH_R2                  (FM_HFI_NOTCH_R * FM_HFI_NOTCH_R)
#define FM_HFI_NOTCH_GAIN                0.9367554f

/* HFI PLL. */
#define FM_HFI_PLL_KP_RAD_S                 240.0f
#define FM_HFI_PLL_KI_RAD_S2              14000.0f
#define FM_HFI_OMEGA_LPF_HZ                 120.0f
#define FM_HFI_CONVERGE_MIN_MS              120U
#define FM_HFI_CONVERGE_TIMEOUT_MS         1400U
#define FM_HFI_CONVERGE_ERR_NORM              0.30f
#define FM_HFI_CONVERGE_OMEGA_RAD_S           35.0f
#define FM_HFI_CONVERGE_SAMPLES              800U  /* 20 ms at 40 kHz */
#define FM_HFI_CONVERGE_BAD_DEC                 2U
/* HFI saliency gives an electrical AXIS modulo pi.  Resolve magnetic polarity
 * before six-step by applying a tiny +Iq torque probe in the candidate FOC
 * frame while HFI continues tracking the saliency axis.  Correct polarity
 * produces a small positive electrical displacement; a pi-wrong candidate
 * produces the same displacement with the opposite sign.  This avoids using
 * BEMF (which is zero at standstill) or guessing a branch.
 *
 * The first probe is intentionally gentle.  If its displacement is below the
 * noise threshold, retry once with slightly more current.  If it is still
 * ambiguous, startup is NOT faulted: six-step starts provisionally and the first
 * trustworthy signed PM-flux/BEMF motion resolves the branch.  Global current
 * protection remains authoritative. */
#define FM_HFI_POLARITY_SETTLE_SAMPLES          40U   /* 1.0 ms quiet HFI baseline */
#define FM_HFI_POLARITY_PULSE_SAMPLES           80U   /* 2.0 ms regulated +Iq probe */
#define FM_HFI_POLARITY_COAST_SAMPLES          160U   /* 4.0 ms HFI-only displacement proof */
#define FM_HFI_POLARITY_PROBE_IQ_A               1.5f
#define FM_HFI_POLARITY_RETRY_IQ_A               2.5f
#define FM_HFI_POLARITY_MIN_DELTA_RAD             0.015f /* ~0.86 electrical degree */
#define FM_HFI_POLARITY_MOTION_MIN_RPM            300.0f /* once real BEMF exists, motion can settle theta-vs-theta+pi */
#define FM_HFI_POLARITY_MOTION_MIN_BEMF_V           0.16f


/* HFI anti-drift is strong while acquiring a stationary rotor, then relaxed
 * when HFI itself is the active motion estimator.  This avoids the old
 * 350-rpm chicken-and-egg cap that required BEMF before HFI could accelerate. */
#define FM_HFI_OMEGA_LEAK_STILL_S               8.0f
#define FM_HFI_OMEGA_LEAK_MOVING_S              0.8f
#define FM_HFI_STATIONARY_MAX_RPM              350.0f
#define FM_HFI_MAX_RPM                        3000.0f
#define FM_HFI_MOTION_BEMF_V                    0.10f
#define FM_HFI_MOTION_MIN_OBS_RPM              120.0f
#define FM_HFI_MOTION_MAX_ANGLE_ERR_RAD          0.60f
#define FM_HFI_MOTION_MAX_RPM_DIFF             300.0f
#define FM_OBSERVER_PLL_ENABLE_MIN_HFI_RPM      250.0f

/* ====================== HFI -> TRUE SIX-STEP START ======================
 * Deterministic zero-speed launcher.  HFI supplies the initial rotor sector;
 * then the bridge advances the six physical commutation states monotonically
 * in one direction: one phase PWM-high, one phase low, third phase floating.
 * The six-step stage now hands to FOC at FM_TRAP_FOC_HANDOVER_RPM.
 * forced_rpm is COMMAND speed while six-step owns the motor; it is never
 * reported as measured mechanical speed.
 */
#define FM_TRAP_TEST_ENABLE                         1U
#define FM_TRAP_START_RPM                         120.0f
#define FM_TRAP_TARGET_RPM                       1500.0f
#define FM_TRAP_FIELD_CEILING_RPM                 1650.0f /* rotating-field headroom; FOC still waits for REAL 1500 rpm */
#define FM_TRAP_ACCEL_RPM_PER_S                 12000.0f /* closed-loop speed-setpoint ramp after flux/BEMF lock */
#define FM_TRAP_BOOTSTRAP_ACCEL_RPM_PER_S           6000.0f /* proven 26090327 bootstrap ramp */
#define FM_TRAP_ACQUIRE_DECEL_RPM_PER_S          12000.0f

/* Build 26091605: positive catch reproduces the clean ~70 s handover.
 * The 26091604 log separated successful and failed positive FOC entries very
 * clearly: the clean ~70.23 s entry had measured rotor 800 rpm, field command
 * 903 rpm (103 rpm slip), 0.516 V BEMF and only 0.051 rad angle error.  The
 * long failure around 57..61 s repeatedly entered FOC with ~400..1100 rpm of
 * field/rotor slip.  Keep the field tied to real positive rotor motion even
 * AFTER BEMF sync, and never probe FOC while that slip is excessive.
 * Negative startup/catch remains unchanged from the 26090327 baseline. */
#define FM_TRAP_POS_SLIP_LIMIT_MIN_RPM             350.0f
#define FM_TRAP_POS_SLIP_LIMIT_MIN_BEMF_V            0.30f
#define FM_TRAP_POS_PRELOCK_MAX_SLIP_RPM            500.0f
#define FM_TRAP_POS_LOCKED_MAX_SLIP_RPM             300.0f
#define FM_TRAP_POS_FOC_MAX_FIELD_SLIP_RPM          300.0f
#define FM_TRAP_START_LINE_V                        1.25f
#define FM_TRAP_TARGET_LINE_V                       1.80f
/* Current-shaped six-step: a strong low-speed launch is allowed.  HFI gives
 * the initial physical sector; a bounded bootstrap creates the first motion;
 * then the qualified PM-flux/BEMF observer angle owns every six-step
 * commutation.  Current tapers toward the FOC handover. */
#define FM_TRAP_LAUNCH_CURRENT_A                    22.0f
#define FM_TRAP_HANDOVER_CURRENT_A                   8.0f
#define FM_TRAP_BEMF_SYNC_ENABLE_RPM               650.0f
#define FM_TRAP_BEMF_SYNC_MIN_V                      0.12f
#define FM_TRAP_BEMF_SYNC_MAX_RPM_ERR              900.0f
#define FM_TRAP_BEMF_SYNC_MAX_PHASE_ERR_RAD           1.0471975512f /* legacy diagnostic gate */
#define FM_TRAP_BEMF_SYNC_GOOD_SAMPLES              30U   /* 3 ms at 10-kHz observer before rotor angle can own six-step */
#define FM_TRAP_BEMF_SYNC_BAD_SAMPLES               20U   /* 2.0 ms: reject brief observer glitches without catch/release jerking */
#define FM_TRAP_FLUX_LOCK_MIN_CMD_RPM               700.0f
#define FM_TRAP_FLUX_LOCK_MIN_RPM                   500.0f
#define FM_TRAP_FLUX_LOCK_MIN_BEMF_V                  0.12f
#define FM_TRAP_FLUX_LOCK_MAX_CMD_ERR_RPM          1000.0f
#define FM_TRAP_FLUX_LOCK_ACQUIRE_MAX_ERR_RAD          0.85f
#define FM_TRAP_FLUX_LOCK_TRACK_MAX_ERR_RAD            0.65f
#define FM_TRAP_FLUX_LOCK_MAX_RPM_JUMP             1400.0f
#define FM_TRAP_FLUX_ANGLE_CORRECTION                  0.45f
#define FM_TRAP_FLUX_SPEED_ALPHA                       0.20f
#define FM_TRAP_FLUX_LOCK_BAD_DEC                       2U
#define FM_TRAP_FLUX_LOCK_EXTRA_GOOD_SAMPLES           20U  /* 2 ms extra before FOC handover */
#define FM_TRAP_FOC_MIN_LOCKED_COMMUTATIONS              6U  /* one electrical revolution of legal observer-owned sectors before FOC probe */
#define FM_TRAP_OPENLOOP_LEAD_RPM                  220.0f /* fallback only before phase pacing is established */
#define FM_TRAP_TORQUE_ADVANCE_LOW_RAD                0.4363323130f /* 25 electrical deg at low speed */
#define FM_TRAP_TORQUE_ADVANCE_HIGH_RAD               0.1745329252f /* 10 electrical deg near FOC entry */
#define FM_TRAP_PHASE_PACE_K_RPM_PER_RAD             220.0f
#define FM_TRAP_PHASE_PACE_MAX_CORR_RPM              260.0f
#define FM_TRAP_FOC_PREP_RPM                       1350.0f
#define FM_TRAP_FOC_PREP_CURRENT_A                    7.0f
#define FM_TRAP_FOC_HANDOVER_PEAK_A                  18.0f
/* Opportunistic early-FOC catch. These thresholds observe the proven
 * continuous six-step trajectory; they do not alter its torque/current/advance.
 * A failed topology probe must abort quickly and then leave six-step alone. */
#define FM_TRAP_EARLY_FOC_CATCH_RPM                   650.0f
#define FM_TRAP_EARLY_FOC_MIN_BEMF_V                    0.35f
#define FM_TRAP_EARLY_FOC_MAX_ANGLE_ERR_RAD              0.30f
#define FM_TRAP_EARLY_FOC_MAX_SPEED_ERR_RPM            250.0f
#define FM_TRAP_EARLY_FOC_MAX_PHASE_CURRENT_A           20.0f /* match proven negative topology admission; does not raise current command */
#define FM_TRAP_EARLY_FOC_RETRY_COOLDOWN_SAMPLES      3200U /* 80 ms @ 40 kHz */
/* Negative-direction catch uses physical motion evidence, not instantaneous
 * trapezoidal Park-Iq sign.  Log 26090316 showed long smooth negative rotation
 * with ~0.57 V BEMF, correct signed flux speed and many legal commutations, but
 * the old 16-A instantaneous phase-current gate rejected most valid windows.
 * Keep the positive path unchanged; allow negative FOC when the rotor evidence
 * is coherent and let the existing 6-A RPM-controlled FOC capture limit make
 * the topology transition gentle. */
#define FM_TRAP_NEG_FOC_MIN_BEMF_V                       0.40f
#define FM_TRAP_NEG_FOC_MAX_ANGLE_ERR_RAD                 0.30f
#define FM_TRAP_NEG_FOC_MAX_SPEED_ERR_RPM               300.0f
#define FM_TRAP_NEG_FOC_MIN_LOCKED_COMMUTATIONS           6U   /* one full electrical revolution */
#define FM_TRAP_NEG_FOC_MIN_SIGNED_IQ_A                   0.00f /* legacy; trapezoidal Park-Iq is no longer a catch veto */
#define FM_TRAP_NEG_FOC_REJECT_COOLDOWN_SAMPLES          200U  /* 5 ms; only used by legacy reject paths */
#define FM_TRAP_NEG_FOC_CATCH_RPM                        600.0f /* smooth negative six-step in the log was already real here */
#define FM_TRAP_NEG_FOC_MAX_PHASE_CURRENT_A               20.0f /* negative six-step naturally sits ~18-20 A; topology admission only, FOC PI remains capped to 6 A */
#define FM_TRAP_NEG_SYMMETRIC_AFTER_POLARITY_PROOF        1U   /* once theta-vs-pi is proven, use positive-quality catch timing */

#define FM_TRAP_OBSERVER_START_CMD_RPM             300.0f
#define FM_TRAP_OBSERVER_DIV                         4U   /* 10-kHz PM-flux/BEMF measurement; angle prediction/commutation remain 40 kHz */
#define FM_HYBRID_HANDOVER_OBSERVER_DIV              1U   /* 40-kHz observer/PLL restored for reliable topology handover */
#define FM_TRAP_FLUX_SPEED_MIN_SAMPLES              20U   /* reject seeded RPM until 1 ms of real flux-angle motion exists */
#define FM_TRAP_COMM_HYST_RAD                        0.0349065850f /* 2 electrical degrees; monotonic anti-chatter */
#define FM_TRAP_SOFT_CURRENT_A                     FM_TRAP_LAUNCH_CURRENT_A
#define FM_TRAP_MIN_LINE_V                          0.25f
#define FM_TRAP_VOLTAGE_DROP_PER_SAMPLE_V           0.0040f
#define FM_TRAP_VOLTAGE_RECOVER_PER_SAMPLE_V        0.00020f
#define FM_TRAP_IQ_HISTORY_ALPHA                    0.05f
#define FM_TRAP_SPEED_KP_V_PER_RPM                  0.00025f
#define FM_TRAP_SPEED_KI_V_PER_RPM_S                0.00080f
#define FM_TRAP_SPEED_BOOST_MAX_V                    0.65f
#define FM_TRAP_MAX_LINE_V                           2.60f
/* At high DC bus voltage, a fixed 1.0..1.45 V six-step request can become a
 * 3..5% PWM pulse.  Keep the requested launch voltage above a modest effective
 * duty target so dead-time/gate-driver nonlinearity does not dominate.  The
 * current-shaped voltage reduction below remains authoritative and can still
 * pull below this target when phase current is high. */
#define FM_TRAP_MIN_SOURCE_DUTY                      0.045f
#define FM_TRAP_SECTOR_CONFIRM_UPDATES                2U
#define FM_TRAP_BEMF_LOSS_RESTART_SAMPLES             0U   /* legacy: observer-loss recovery uses matched bootstrap, never zero-speed restart */
#define FM_TRAP_MODEL_MIN_FLUX_RATIO                  0.20f
/* Legacy floating-phase ZC constants retained for source/protocol compatibility.
 * Build 26090310 does NOT use reconstructed floating-phase ZC as commutation
 * authority because the PCB has no U/V/W terminal-voltage ADC measurements. */
#define FM_TRAP_ZC_MIN_BEMF_VECTOR_V                  0.060f
/* Below this startup command, reconstructed BEMF is dominated by the
 * high-current switching/model residual visible even at zero mechanical RPM.
 * HFI + the bounded bootstrap sequence own this physically sensorless region. */
#define FM_TRAP_ZC_ACQUIRE_MIN_CMD_RPM               450.0f /* bootstrap only until reconstructed floating-phase BEMF is measurable */
#define FM_TRAP_ZC_ACQUIRE_MIN_FLUX_RPM              250.0f /* first-ZC plausibility cross-check only; ZC period owns speed after acquisition */
#define FM_TRAP_ZC_ACQUIRE_TIMEOUT_MULT                2U
#define FM_TRAP_ZC_HYST_V                             0.018f
#define FM_TRAP_ZC_BLANK_ISR                          4U    /* 100 us after a sector change */
#define FM_TRAP_ZC_LOCK_EVENTS                        4U   /* four ordered floating-phase crossings before ZC owns six-step timing */
#define FM_TRAP_ZC_MAX_FLUX_RPM_ERR                900.0f /* first ZC only: broad independent-motion agreement gate */
#define FM_TRAP_ZC_CHAIN_MAX_RPM_ERR               600.0f /* later ZCs agree with the previous physical ZC period, not noisy raw-flux RPM */
#define FM_TRAP_ZC_CHAIN_MAX_FRAC_ERR                0.40f /* allow load acceleration while rejecting unrelated switching edges */
#define FM_TRAP_ZC_MIN_HALFSECTOR_ISR                 3U
#define FM_TRAP_ZC_MAX_HALFSECTOR_ISR               600U
#define FM_TRAP_ZC_TIMEOUT_SECTOR_MULT                2U
#define FM_TRAP_ZC_HOLDOVER_SECTORS                    2U   /* bridge one or two missing crossings with last measured sector period */
#define FM_TRAP_ZC_DIDT_ALPHA                         0.22f /* cheap 40-kHz floating-phase di/dt filter */
#define FM_TRAP_ZC_BEMF_ALPHA                         0.20f /* cheap 40-kHz per-phase reconstructed-BEMF filter */
#define FM_TRAP_ZC_FLOAT_CURRENT_MAX_A                 4.0f /* accept ZC only while the selected Hi-Z phase is actually near zero current */
#define FM_TRAP_ZC_FIRST_CMD_FRAC_MIN                  0.35f /* first ZC period must be physically plausible vs bootstrap speed */
#define FM_TRAP_ZC_FIRST_CMD_FRAC_MAX                  2.20f
#define FM_TRAP_HFI_REACQUIRE_RPM                   250.0f

/* ================= TRUE SIX-STEP -> FOC HANDOVER =================
 * Six-step is now the deterministic zero-speed launcher.  Once it has carried
 * the rotor into a comfortable BEMF region, switch the bridge back to normal
 * three-phase PWM and run sinusoidal current control on the continuously
 * propagated six-step rotor reference.  The BEMF observer watches this FOC
 * waveform for a short coherent raw-angle window.  When that window is valid
 * the angle authority switches directly (NO angle blend), but the current PI
 * is made bumpless by using measured Id/Iq as the first references and by
 * seeding its integrators from the already-applied alpha/beta voltage vector.
 */
#define FM_TRAP_FOC_HANDOVER_RPM                 1500.0f
#define FM_TRAP_FOC_ENTRY_IQ_MAX_A                 22.0f
#define FM_TRAP_FOC_ENTRY_ID_MAX_A                  6.0f
#define FM_TRAP_FOC_EQUIV_VQ_SCALE                  0.58f
#define FM_TRAP_FOC_CAPTURE_VOLTAGE_LIMIT_V           2.60f
#define FM_TRAP_FOC_CAPTURE_TIMEOUT_MS             80U
#define FM_TRAP_FOC_ZERO_ERROR_SAMPLES             160U   /* 4.0 ms current-tracking proof; observer cannot own angle here */
#define FM_TRAP_FOC_TOPOLOGY_SETTLE_SAMPLES         40U   /* 1.0 ms topology settling */
#define FM_TRAP_FOC_OBSERVER_WARMUP_SAMPLES          20U   /* 0.5 ms observer warmup */
/* Keep the six-step current vector unchanged through observer acquisition.
 * There is intentionally no pre-observer Id/Iq reduction. */
#define FM_TRAP_FOC_BEMF_CONSECUTIVE_SAMPLES            8U   /* 0.20 ms: deliberately minimal once six-step calibration is known */
/* The six-step voltage-vector / rotor-flux geometry is deterministic.  Do NOT
 * learn an angle offset while the rotor is slipping behind open-loop commutation. */
#define FM_TRAP_FOC_CAL_START_RPM                     FM_TRAP_OBSERVER_START_CMD_RPM
#define FM_TRAP_FOC_CAL_SAMPLES                        0U
#define FM_TRAP_FOC_CAL_MIN_BEMF_V                     0.050f
#define FM_TRAP_FOC_CAL_PHASE_MIN                       0.0f
#define FM_TRAP_FOC_CAL_PHASE_MAX                       1.0f
#define FM_TRAP_FOC_OBSERVER_PLL_ENABLE_RPM        400.0f
#define FM_TRAP_FOC_OBSERVER_PROBE_BLEND             0.00f /* PM-flux angle only during acquisition; control remains pinned to forced trajectory */
#define FM_TRAP_FOC_FLUX_MIN_RPM                      80.0f
#define FM_TRAP_FOC_FLUX_MAX_RPM_ERR                3000.0f /* do not reject a valid angle because the speed LPF is still catching up */
#define FM_TRAP_FOC_IDIQ_RAMP_SAMPLES               400U   /* 10 ms observer-FOC Id -> 0; Iq remains at captured six-step level */

/* ================= HYBRID SIX-STEP <-> SENSORLESS FOC =================
 * Drone-oriented operating split:
 *   - six-step owns startup / genuinely low speed up to 1500 mechanical rpm;
 *   - at 1500 rpm the bridge changes to sinusoidal FOC and phase-locks there;
 *   - once FOC is qualified, its first minimum operating target is 2000 rpm;
 *   - normal commanded-speed control takes over above that floor.
 *
 * IMPORTANT: loss of observer validity at high speed is NOT permission to enter
 * low-speed six-step.  Six-step re-entry requires positive low-speed evidence.
 * This prevents an already-fast propeller/motor from being commutated as if it
 * were a 120-rpm rotor. */
#define FM_HYBRID_FOC_ENTER_RPM                    1500.0f
#define FM_HYBRID_FOC_EXIT_RPM                      900.0f
#define FM_HYBRID_REVERSAL_TO_SIXSTEP_RPM           900.0f
#define FM_HYBRID_STOP_RPM                           60.0f
#define FM_HYBRID_TOPOLOGY_SETTLE_SAMPLES            20U   /* 0.5 ms: proven 26090301 topology settle */
#define FM_HYBRID_IF_STABILIZE_SAMPLES               20U   /* 0.5 ms: proven 26090301 stabilization */
#define FM_HYBRID_SYNC_RAMP_SAMPLES                  40U   /* 1.0 ms: proven 26090301 phase-lock ramp */
#define FM_HYBRID_SYNC_LOCK_SAMPLES                  16U   /* 0.4 ms: do not trap startup before RUN grace can help */
#define FM_HYBRID_NEG_SYNC_LOCK_SAMPLES              16U   /* same 0.4 ms proof as positive once signed flux is preserved */
#define FM_HYBRID_NEG_RESEED_FLUX_ON_FOC_ENTRY          0U   /* 26090325 preserves/propagates qualified six-step flux instead */
#define FM_HYBRID_NEG_RESEED_MIN_RPM                  400.0f /* only relevant to a real negative moving handover */
#define FM_HYBRID_HANDOVER_FALLBACK_SAMPLES          240U   /* 6 ms bad-probe timeout; preserve continuous six-step */
#define FM_HYBRID_SYNC_LOCK_ERR_RAD                  0.17f /* ~10 electrical deg */
#define FM_HYBRID_SYNC_MAX_FLUX_RPM_ERR             600.0f /* independent raw-flux speed must agree with the 1500-rpm bridge */
#define FM_HYBRID_SYNC_MIN_FLUX_RPM                 500.0f /* state-4 BEMF gate already proves real motion before early catch */
#define FM_HYBRID_SYNC_MIN_BEMF_V                    0.08f
#define FM_HYBRID_PLL_KP_RAD_S_PER_RAD             1330.0f /* ~150 Hz, zeta ~= 0.7: fast but far below old 800-Hz experiment */
#define FM_HYBRID_PLL_KI_RAD_S2_PER_RAD          890000.0f
#define FM_HYBRID_PLL_PHASE_CLAMP_RAD                0.15f /* ~8.6 deg correction cap */
#define FM_HYBRID_PLL_MAX_TRACK_ERR_RAD               0.60f /* >34 deg: predict, never chase the raw glitch */
#define FM_HYBRID_PLL_MAX_ACCEL_RPM_S            120000.0f
#define FM_HYBRID_PLL_MAX_MECH_RPM                36000.0f
#define FM_HYBRID_PLL_MAX_TRIM_FRACTION              0.12f /* +/-12% slip trim only */
#define FM_HYBRID_PLL_MIN_TRIM_RPM                    80.0f
#define FM_HYBRID_FOC_SPEED_FLOOR_RPM                  0.0f /* no artificial 2000 rpm RUN floor */
#define FM_HYBRID_HANDOVER_CURRENT_RAMP_SAMPLES      8000U   /* 200 ms: preserve loaded torque while the FOC observer settles */
#define FM_HYBRID_HANDOVER_CAPTURE_IQ_A                22.0f /* same torque authority as six-step; speed PI still decides actual Iq */
#define FM_HYBRID_CAPTURE_IQ_MAX_A                     22.0f /* preserve loaded torque while observer is being proven */
/* Handover is RPM-controlled FOC.  Six amps is only the saturation bound of
 * the speed PI, not a commanded capture current.  Normal RUN authority opens
 * gradually only after the observer has proven coherent FOC operation. */
#define FM_HYBRID_CAPTURE_LOAD_IQ_MAX_A                22.0f /* fixed capture authority; no load-adaptive unlock */
#define FM_HYBRID_CAPTURE_LOAD_ERR_RPM                 450.0f
#define FM_HYBRID_CAPTURE_LOAD_RELEASE_ERR_RPM         180.0f
#define FM_HYBRID_CAPTURE_LOAD_SAT_FRACTION              0.95f
#define FM_HYBRID_CAPTURE_LOAD_BOOST_SAMPLES          1000U /* 25 ms to unlock heavy-load controller */
#define FM_HYBRID_CAPTURE_LOSS_HOLD_SAMPLES           1600U /* 40 ms, heavy mode only */
#define FM_HEAVY_LOAD_VOLTAGE_LIMIT_RATIO                0.57f
#define FM_CAPTURE_HEAVY_SPEED_KP_A_PER_RPM             0.00900f
#define FM_CAPTURE_HEAVY_SPEED_KI_A_PER_RPM_S            0.18000f
#define FM_HYBRID_CAPTURE_RAMP_RPM_PER_S             5000.0f /* gentle RPM-loop slew after an opportunistic early catch */
#define FM_HYBRID_NEG_CAPTURE_RAMP_RPM_PER_S         5000.0f /* gentler negative capture; positive path unchanged */
#define FM_HYBRID_CAPTURE_MIN_SAFE_RPM                500.0f /* early BEMF-qualified FOC may prove itself below 1000 rpm */
#define FM_HYBRID_CAPTURE_STABLE_SAMPLES              400U   /* 10 ms: stay in full-torque FOC capture until the observer is persistently coherent */
#define FM_HYBRID_NEG_CAPTURE_STABLE_SAMPLES          400U   /* 10 ms: same persistent proof for negative FOC */
#define FM_HYBRID_CAPTURE_GOOD_DECAY_SAMPLES             4U   /* one bad 40-kHz sample erases 0.10 ms of proof instead of all 10 ms */
#define FM_HYBRID_NEG_CAPTURE_CLASSIFY_SAMPLES        2400U   /* 60 ms @40 kHz */
#define FM_HYBRID_NEG_CAPTURE_MIN_PEAK_BEMF_V            0.27f
#define FM_HYBRID_NEG_CAPTURE_WEAK_RETRY_COOLDOWN_SAMPLES 2400U /* 60 ms six-step recovery */
#define FM_HYBRID_CAPTURE_DROPOUT_HOLD_SAMPLES       500U   /* 12.5 ms: ride through brief topology/observer glitches before destructive six-step fallback */
#define FM_HYBRID_CAPTURE_DROPOUT_IQ_A                12.0f /* retain useful torque through a brief observer dropout */
#define FM_HYBRID_CAPTURE_INCOHERENT_GRACE_SAMPLES  12000U   /* 300 ms: do not starve/fallback a loaded FOC capture while the new topology observer settles */
#define FM_HYBRID_CAPTURE_STALL_SAMPLES             4000U   /* 100 ms at 40 kHz: saturated weak-BEMF capture must make real progress */
#define FM_HYBRID_CAPTURE_STALL_ERR_RPM             1200.0f
#define FM_HYBRID_CAPTURE_STALL_IQ_A                   5.5f
#define FM_HYBRID_CAPTURE_STALL_MAX_BEMF_V              0.28f
#define FM_HYBRID_CAPTURE_ABSOLUTE_MAX_SAMPLES      12000U   /* 300 ms: state 5 is a transition, never a multi-second operating mode */
#define FM_HYBRID_RUN_REACQUIRE_SAMPLES                40U   /* 1 ms coherent raw-flux proof before restoring torque */
#define FM_HYBRID_RUN_REACQUIRE_MAX_ANGLE_ERR_RAD       0.45f
#define FM_HYBRID_RUN_REACQUIRE_MAX_RPM_ERR          1500.0f
/* Fresh RUN entry gets a slightly longer bounded holdover window.  The motor
 * has just demonstrated 20+ ms of coherent handover/capture, so a brief
 * topology/observer hiccup should not immediately throw it back to six-step.
 * Torque is clamped to the capture-level current while this grace is active. */
#define FM_HYBRID_RUN_ENTRY_GRACE_SAMPLES            1200U   /* 30 ms */
#define FM_HYBRID_NEG_RUN_ENTRY_GRACE_SAMPLES        2400U   /* 60 ms negative observer grace */
#define FM_HYBRID_RUN_ENTRY_HOLD_IQ_A                  12.0f
#define FM_HYBRID_RUN_IQ_UNLOCK_SAMPLES               4000U   /* 100 ms: speed PI authority ramps 6 A -> normal RUN limit */
#define FM_HYBRID_NEG_RUN_IQ_UNLOCK_SAMPLES           8000U   /* 200 ms negative authority unlock */
#define FM_HYBRID_HOLDOVER_IQ_SLEW_A_PER_S           5000.0f
/* This is a SPEED-SETPOINT slew only.  It never advances electrical angle.
 * Once phase lock completes, theta/omega come exclusively from the observer PLL. */
#define FM_HYBRID_RUN_ENTRY_SPEED_RAMP_RPM_PER_S     5000.0f /* first 100 ms after FOC acquisition: keep RPM PI gentle in both directions */
#define FM_HYBRID_NEG_RUN_ENTRY_SPEED_RAMP_RPM_PER_S 3000.0f /* negative only; positive behavior preserved */
#define FM_HYBRID_POS_FOC_RUN_MIN_BEMF_V              0.48f /* 26091705: stable positive RUN entries were >=~0.44 V; failed entries were <=~0.45 V */
#define FM_HYBRID_POS_FOC_RUN_MIN_RPM                 900.0f /* remain in full-torque FOC capture below this; does not add six-step time */
#define FM_HYBRID_POS_FOC_RUN_MAX_SPEED_ERR_RPM       350.0f /* stronger physical-speed agreement before declaring RUN */
#define FM_HYBRID_NEG_FOC_RUN_MIN_BEMF_V              0.48f /* 26091705: successful negative entries were >=0.50 V; failed entries stayed <=0.43 V */
#define FM_HYBRID_NEG_FOC_RUN_MIN_RPM                 750.0f /* do not declare negative RUN on the old ~600-rpm false-lock plateau */
#define FM_HYBRID_NEG_FALSE_LOCK_MAX_RPM              750.0f /* RUN watchdog: below this, demand independent physical progress */
#define FM_HYBRID_NEG_FALSE_LOCK_MAX_BEMF_V             0.30f /* false-lock signature in 26091607 was ~0.21..0.26 V */
#define FM_HYBRID_NEG_FALSE_LOCK_MIN_ERR_RPM          1200.0f /* only intervene when the command is materially faster than feedback */
#define FM_HYBRID_NEG_FALSE_LOCK_MIN_IQ_A                7.0f /* old false lock sat against the -8 A safe ceiling */
#define FM_HYBRID_NEG_FALSE_LOCK_SAMPLES               1200U /* 30 ms at 40 kHz: genuine starts clear this quickly */
#define FM_HYBRID_NEG_RELEASE_DECAY_TICKS                 2U /* one marginal 1-kHz sample must not reset all released authority */
#define FM_HYBRID_NEG_FOC_RUN_MAX_PHASE_ERR_RAD         0.08f /* require a settled negative FOC angle before normal RUN */
#define FM_HYBRID_NEG_FOC_RUN_MAX_SPEED_ERR_RPM        200.0f
#define FM_HYBRID_NEG_RUN_SAFE_IQ_A                     8.0f /* post-catch RPM-PI ceiling while the negative observer builds margin */
#define FM_HYBRID_NEG_RUN_SAFE_RAMP_RPM_PER_S        7500.0f /* faster than old 3000-rpm/s entry, but without the 30k-rpm/s step */
#define FM_HYBRID_NEG_RUN_RELEASE_MIN_RPM             1600.0f
#define FM_HYBRID_NEG_RUN_RELEASE_MIN_BEMF_V             0.52f
#define FM_HYBRID_NEG_RUN_RELEASE_MAX_PHASE_ERR_RAD       0.10f
#define FM_HYBRID_NEG_RUN_RELEASE_GOOD_SPEED_TICKS       10U /* 10 ms at 1-kHz speed loop */
#define FM_HYBRID_NEG_RUN_RELEASE_RAMP_SPEED_TICKS       80U /* 100 ms at 1-kHz speed loop */
#define FM_HYBRID_SPEED_SETPOINT_RAMP_RPM_PER_S    30000.0f
#define FM_HYBRID_RUN_RAW_LOSS_SAMPLES              800U   /* 20 ms high-speed observer-rejoin grace; never directly triggers six-step */
#define FM_HYBRID_RUN_TO_SIXSTEP_CONFIRM_SAMPLES    400U   /* 10 ms independent proof below FOC-exit speed before low-speed mode change */
#define FM_HYBRID_RUN_REJOIN_MAX_ERR_RAD               0.45f /* raw flux may pull the PLL back only within ~69 electrical deg */
#define FM_HYBRID_RUN_REJOIN_STEP_RAD                  0.004f /* <=0.57 electrical deg per 25 us ISR */
#define FM_HYBRID_RUN_REJOIN_OMEGA_ALPHA               0.01f  /* bounded raw-flux speed pull during phase reacquisition */
#define FM_HYBRID_ID_RELEASE_SAMPLES                 120U   /* 3 ms Id -> 0 after real phase lock */

/* ====================== HFI-SEEDED ROTATING-FIELD / ZVV START ======================
 * HFI is trusted only long enough to provide the initial standstill electrical
 * axis.  Once it has converged, HFI is disabled and the FOC frame is advanced
 * monotonically in the requested positive direction.  This is deliberately an
 * open-loop ROTATING current vector, not a set of unrelated static kicks:
 *
 *   HFI seed -> slow forward electrical-angle ramp -> short hold -> ZVV -> BEMF
 *
 * Positive Iq is current controlled throughout the forced rotation.  The angle
 * ramp is intentionally slow enough for the low-inertia A2212 rotor to follow,
 * while the existing 0.75 V absolute slingshot clamp and all global current /
 * ADC-rail protection remain authoritative.
 *
 * The forced mechanical-equivalent speed is only a command/diagnostic; it is
 * NEVER treated as measured rotor speed.  sling_rpm remains zero until the ZVV
 * estimator proves real BEMF motion after the bridge is released.
 */
#define FM_SLING_KICK_IQ_A                      8.0f
#define FM_SLING_KICK_START_IQ_A                1.0f
#define FM_SLING_KICK_RAMP_SAMPLES            320U   /* 8.00 ms current slew at 40 kHz */
#define FM_SLING_ROTATE_START_RPM               20.0f
#define FM_SLING_ROTATE_TARGET_RPM             450.0f
#define FM_SLING_ROTATE_RAMP_SAMPLES          3200U   /* 80 ms fallback ramp; early observer capture still starts at 45 rpm */
#define FM_SLING_ROTATE_HOLD_SAMPLES           800U   /* 20 ms fallback window; normally observer wins much earlier */
#define FM_SLING_KICK_SAMPLES \
    (FM_SLING_ROTATE_RAMP_SAMPLES + FM_SLING_ROTATE_HOLD_SAMPLES)
#define FM_SLING_ZVV_BLANK_SAMPLES             12U   /* 0.30 ms switching/current-step settle */
#define FM_SLING_ZVV_OBSERVE_SAMPLES          160U   /* 4.00 ms BEMF/current-response observation */
#define FM_SLING_ZVV_MIN_VALID_SAMPLES         28U
#define FM_SLING_ZVV_DIDT_LPF_HZ             8000.0f
#define FM_SLING_ZVV_BEMF_LPF_HZ             3500.0f
#define FM_SLING_MIN_BEMF_V                     0.080f
#define FM_SLING_MIN_DIRECTION_RPM             80.0f
#define FM_SLING_ACCEPT_RPM                   140.0f
#define FM_SLING_MAX_ATTEMPTS                   1U    /* one coherent forward launch; do not cancel momentum */
#define FM_SLING_CURRENT_KP_V_PER_A              0.10f
#define FM_SLING_CURRENT_KI_STEP_V_PER_A          0.0020f
#define FM_SLING_INITIAL_INT_Q_V                 0.0f
/* Absolute slingshot voltage ceiling retained from the successful 2026-08-26
 * current-limited test.  Do not convert this back to a bus-voltage fraction. */
#define FM_SLING_VOLTAGE_LIMIT_V                  0.75f
#define FM_SLING_EARLY_RELEASE_CURRENT_A          14.0f

/* Early driven-observer capture.  The voltage-model observer runs while the
 * rotating FOC field is still energized; ZVV is fallback only.  The first
 * trustworthy BEMF angle starts a short forced->observer blend immediately. */
#define FM_SLING_OBSERVER_ENABLE_RPM               45.0f
#define FM_SLING_OBSERVER_VALID_BEMF_V              0.030f
#define FM_SLING_OBSERVER_GOOD_BEMF_V               0.040f
#define FM_SLING_OBSERVER_MAX_PHASE_ERR_RAD          0.70f
#define FM_SLING_OBSERVER_GOOD_SAMPLES              24U   /* legacy telemetry alias; raw-angle window below is authoritative */
#define FM_SLING_OBSERVER_BLEND_SAMPLES             40U   /* 1.00 ms bumpless angle/authority blend */
#define FM_SLING_OBSERVER_PROBE_BLEND                0.10f
#define FM_SLING_OBSERVER_FULL_BLEND                 0.98f

/* Raw BEMF-vector acquisition.  Qualification is based on a multi-sample
 * angle residual window, not the instantaneous seed-biased PLL phase error. */
#define FM_SLING_ACQUIRE_BEMF_V                       0.10f
#define FM_SLING_ACQUIRE_SAMPLES                     FM_TRAP_FOC_BEMF_CONSECUTIVE_SAMPLES
#define FM_SLING_ACQUIRE_MAX_RESIDUAL_RAD              0.70f
#define FM_SLING_ACQUIRE_MAX_INNOVATION_RAD            0.55f
#define FM_SLING_ACQUIRE_MAX_VARIANCE_RAD2              0.10f

/* Bumpless PI transfer.  0.5 mixes the live PI-state history with a seed
 * reconstructed from averaged applied Vd/Vq and averaged Id/Iq error. */
#define FM_HANDOVER_PI_HISTORY_BLEND                    0.50f
#define FM_BEMF_ENTRY_IQ_RAMP_SAMPLES                  400U  /* 10 ms entry-Iq -> boost-Iq ramp */

/* After the slingshot has created measurable BEMF, use the real BEMF observer
 * for FOC immediately.  A bounded torque command carries the motor from the
 * acquisition speed to a comfortably observable region, then the normal speed
 * PI takes over. */
#define FM_BEMF_BOOST_IQ_A                       4.0f  /* legacy nominal only; build 26082611 holds captured six-step Iq through BEMF-FOC */
#define FM_BEMF_BOOST_TO_SPEED_RPM             250.0f
#define FM_BEMF_BOOST_TIMEOUT_MS                450U
#define FM_BEMF_BOOST_MIN_BEMF_V                 0.030f
#define FM_BEMF_BOOST_MAX_PHASE_ERR_RAD           1.20f
#define FM_BEMF_BOOST_BAD_SAMPLES               400U  /* 10 ms: closed-loop recovers without an abrupt stop */

/* Observer/handover constants retained for the normal RUN estimator and legacy
 * telemetry.  HFI is not used as the torque-angle source after the slingshot. */
#define FM_HFI_TO_BEMF_TARGET_RPM                500.0f
#define FM_HFI_RAMP_RPM_PER_S                   1800.0f
#define FM_HFI_START_IQ_MAX_A                      4.0f
#define FM_HFI_START_TIMEOUT_MS                 4200U
#define FM_HFI_START_MIN_TRACK_SALIENCY_RATIO      0.25f
#define FM_HFI_START_BAD_TRACK_SAMPLES           800U
#define FM_HFI_OBSERVER_ENABLE_RPM               250.0f
#define FM_HFI_OBSERVER_MIN_BEMF_V                 0.07f
#define FM_HFI_OBSERVER_MAX_RPM_ERR              300.0f
#define FM_HFI_OBSERVER_MAX_ANGLE_ERR_RAD          0.45f
#define FM_HFI_OBSERVER_PHASE_ERR_RAD              1.00f
#define FM_HFI_OBSERVER_GOOD_SAMPLES             1200U
#define FM_HFI_OBSERVER_RELEASE_UP_S                6.0f
#define FM_HFI_OBSERVER_RELEASE_DOWN_S             12.0f
#define FM_HFI_OBSERVER_POLARITY_BLEND_MIN           0.35f
#define FM_FULL_RELEASE_BLEND_MIN                     0.999f
#define FM_HFI_OBSERVER_HANDOVER_MS                 40U
#define FM_HFI_OBSERVER_HANDOVER_TIMEOUT_MS        600U
#define FM_HFI_HANDOVER_MAX_ANGLE_ERR_RAD             0.45f
#define FM_HFI_HANDOVER_MAX_RPM_ERR                 450.0f
#define FM_HFI_HANDOVER_MIN_BEMF_V                    0.08f

/* Launch current-loop tuning. */
#define FM_PI_CAPTURE_KP_V_PER_A                   0.025f
#define FM_PI_CAPTURE_KI_STEP_V_PER_A              0.00025f
#define FM_LAUNCH_CURRENT_KP_V_PER_A               0.045f
#define FM_LAUNCH_CURRENT_KI_STEP_V_PER_A           0.0010f

/* Unique build signature for the HFI-closed-loop zero-start architecture. */
#define FM_FIRMWARE_BUILD_ID                    26101703U /* raw RUN observer 20k + genuine V13 20k SHADOW */

/* ========================= HIGH-SPEED TARGET ========================= */
#define FM_TARGET_RPM                 2000.0f
#define FM_MAX_RPM                   32000.0f
#define FM_TARGET_RAMP_RPM_PER_S     8000.0f
#define FM_HANDOVER_MS                80U
#define FM_HANDOVER_MAX_ANGLE_ERR_RAD 0.35f

/* ========================= SPEED PI ========================= */
#define FM_SPEED_LOOP_DIV            40U       /* 40 kHz / 40 = 1 kHz */

/* Hard-real-time CPU-load controls.  These do not change the 40 kHz control
 * mathematics; they only decimate diagnostics that are not needed every PWM
 * sample. */
#define FM_TRAP_DQ_HISTORY_DIV                      8U   /* 5 kHz D/Q history; handover seeding only, not control */
#define FM_TRAP_IQ_HISTORY_ALPHA_DECIMATED          0.18549375f /* 1-(1-0.05)^4 */

#define FM_SPEED_KP_A_PER_RPM        0.00450f
#define FM_SPEED_KI_A_PER_RPM_S      0.08000f
#define FM_SPEED_IQ_MAX_A           40.0f  /* increased low/mid-speed RUN torque authority */
#define FM_SPEED_IQ_HIGH_RPM_A      30.0f  /* retain strong torque authority at high rpm while hard OCP stays active */
#define FM_SPEED_IQ_TAPER_START_RPM 5000.0f
#define FM_SPEED_IQ_TAPER_END_RPM   10000.0f
#define FM_RUN_UNLOCK_SAMPLES        4000U  /* 100 ms sustained gross observer-loss at 40 kHz */
#define FM_RUN_OBSERVER_HEALTH_GRACE_MS 500U
#define FM_RUN_HEALTH_MIN_BEMF_V       0.04f
#define FM_RUN_OBSERVER_MIN_RPM        220.0f
#define FM_RUN_HEALTH_MAX_PHASE_ERR_RAD 0.35f  /* ~20 deg: do not let a bad observer frame reach FOC */
#define FM_RUN_MAX_OBSERVER_ANGLE_INNOVATION_RAD 0.25f /* ~14 deg vs one-step prediction */

/* Drone-oriented brief estimator holdover.  Never freeze a stale angle at
 * speed: propagate the last trusted real angle with its last trusted omega for
 * only a few milliseconds.  If the observer does not rejoin close to that
 * prediction, stop rather than applying torque at an unknown angle. */
#define FM_OBSERVER_HOLDOVER_SAMPLES              400U  /* 10 ms SEARCH window; no longer aborts an active recapture blend */
#define FM_OBSERVER_HOLDOVER_MAX_RPM_JUMP          800.0f

/* Direct RUN rejoin: once the observer is clearly healthy again, give it angle
 * authority immediately.  This deliberately bypasses the older multi-sample
 * recapture/blend path that could keep a recovered observer trapped in
 * holdover until fault 34. */
#define FM_OBSERVER_DIRECT_REJOIN_MIN_BEMF_V          0.080f
#define FM_OBSERVER_DIRECT_REJOIN_MIN_RPM            180.0f
#define FM_OBSERVER_DIRECT_REJOIN_MAX_PHASE_ERR_RAD    0.35f  /* ~20 deg */
#define FM_OBSERVER_DIRECT_REJOIN_SLEW_SAMPLES        40U     /* 1.0 ms at 40 kHz; starts from one healthy sample */
#define FM_OBSERVER_REJOIN_REF_RECOVERY_SAMPLES       40U     /* 1.0 ms current-vector cleanup after theta rejoin */

/* Coherence-based RUN observer re-capture.  The raw PM-flux angle may be far from
 * the propagated trajectory after a PLL upset, but it must move coherently on
 * the already-resolved magnetic-polarity branch before FOC authority returns. */
#define FM_OBSERVER_RECATCH_MIN_BEMF_V               0.080f
#define FM_OBSERVER_RECATCH_MIN_RPM                 180.0f
#define FM_OBSERVER_RECATCH_MAX_RPM_DELTA          1600.0f
#define FM_OBSERVER_RECATCH_SAMPLES                  24U   /* 0.60 ms */
#define FM_OBSERVER_RECATCH_MAX_INNOVATION_RAD        0.45f
#define FM_OBSERVER_RECATCH_MAX_VARIANCE_RAD2          0.030f
#define FM_OBSERVER_RECATCH_MAX_OFFSET_RAD             2.75f /* reject near-pi branch flips, accept the ~133 deg best-run recovery */
#define FM_OBSERVER_RECATCH_BLEND_SAMPLES             80U   /* 2.0 ms bumpless frame blend */
#define FM_OBSERVER_RECATCH_BLEND_TIMEOUT_SAMPLES     120U   /* 3.0 ms watchdog; direct rejoin slew is only 1.0 ms */

/* ========================= CURRENT PI ========================= */
#define FM_CURRENT_KP_V_PER_A        0.040f
#define FM_CURRENT_KI_STEP_V_PER_A   0.0009f
#define FM_RUN_PI_GAIN_RAMP_MS        400U  /* after observer blend is complete, smoothly restore normal current-PI gains */
#define FM_VOLTAGE_LIMIT_RATIO       0.54f
#define FM_MIN_DUTY                  0.03f
#define FM_MAX_DUTY                  0.97f

/* ========================= FAST BEMF OBSERVER ========================= */
#define FM_DIDT_LPF_HZ               6000.0f
#define FM_BEMF_LPF_HZ               2500.0f
#define FM_BEMF_VALID_MIN_V          0.06f
#define FM_BEMF_LOCK_MIN_V           0.10f
#define FM_LOCK_MIN_RPM              300.0f
#define FM_LOCK_MAX_PHASE_ERR_RAD    1.00f
#define FM_LOCK_SAMPLES              200U
/* Flux-linkage angle tracker.  The old +pi/2 BEMF phase offset is deliberately
 * gone: rotor angle is atan2(psi_pm_beta, psi_pm_alpha) directly. */
#define FM_OBSERVER_BEMF_ANGLE_OFFSET_RAD            0.0f /* legacy/ZVV only */
#define FM_OBSERVER_FORWARD_ONLY                       0U  /* hybrid six-step path supports signed direction changes */
#define FM_FLUX_SPEED_LPF_HZ                         800.0f /* raw-flux speed diagnostic / startup calibration only */
#define FM_FLUX_RADIAL_CORRECTION                      0.0025f
#define FM_FLUX_COMPONENT_LIMIT_RATIO                  1.80f
#define FM_FLUX_MIN_MAG_RATIO                          0.20f
#define FM_FLUX_ANGLE_CORRECTION                       0.55f /* legacy, retained for compatibility */
#define FM_FLUX_MAX_INNOVATION_RAD                     1.00f /* legacy, retained for compatibility */
#define FM_FLUX_MAX_SAMPLE_DELTA_RAD                   0.75f
#define FM_FLUX_MAX_MECH_ACCEL_RPM_S               60000.0f /* raw-flux speed diagnostic only */

/* High-bandwidth type-II PM-flux angle PLL.  These gains correspond to about
 * an 800-Hz natural frequency with ~0.7 damping at the 40-kHz control rate.
 * The PLL predicts theta from omega each ISR, so high rotor speed does not
 * create filter lag.  The phase-detector clamp prevents a single voltage-model
 * glitch from rotating the FOC frame by tens of degrees. */
#define FM_PLL_KP_RAD_S_PER_RAD                      7100.0f
#define FM_PLL_KI_RAD_S2_PER_RAD                 25000000.0f
#define FM_PLL_PHASE_DETECTOR_CLAMP_RAD                0.25f  /* 14.3 electrical deg */
#define FM_PLL_MAX_MECH_ACCEL_RPM_S                400000.0f
#define FM_PLL_MAX_MECH_RPM                         36000.0f
#define FM_OBSERVER_EXTRA_LEAD_RAD                     0.0f

/* ========================= STARTUP ADC OFFSET ========================= */
#define FM_OFFSET_MEDIAN_SAMPLES     127U
#define FM_OFFSET_DISCARD_SAMPLES    32U
#define FM_OFFSET_CAL_TIMEOUT_MS     50U

#endif
