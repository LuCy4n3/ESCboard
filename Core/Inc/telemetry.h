#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "fast_motor.h"

/*
 * PC protocol expected by the supplied SerialReader:
 *   [HEADER ...] [ADDR 1 byte] [payload N bytes]
 *
 * The supplied reader does not hard-code the header.  This project uses the
 * example header AA 55 and makes it explicit here so it can be changed in one
 * place if the PC configuration uses a different value.
 *
 * Every channel below is 4-byte little-endian.  Channels that can be negative
 * use offset-binary on the wire because the current PC logger parses the four
 * payload bytes as uint32_t.  The matching 26082846 JSON subtracts the sign
 * bias.  The UART speed command remains signed 16-bit big-endian two's complement.
 */
#define TELEMETRY_HEADER0                 0xAAU
#define TELEMETRY_HEADER1                 0x55U
/* Low-rate telemetry + offline 4kHz RAM capture (~10.85 kB/s on wire at 460800 baud).
 * Switch to 0 to restore 37-channel production capture. */
#define TELEMETRY_V13_SHADOW_CAPTURE       0U
#if TELEMETRY_V13_SHADOW_CAPTURE
#define TELEMETRY_PERIOD_MS               50U /* 20Hz compact DMA diagnostics */
#else
#define TELEMETRY_PERIOD_MS               50U
#endif

/* RUN raw 20kHz observer A/B diagnostic channels. */
#define TLM_ADDR_OBSERVER20K_MODE          0xF2U
#define TLM_ADDR_OBSERVER20K_UPDATES       0xF3U
#define TLM_ADDR_OBSERVER20K_PREDICTS      0xF4U

/* Address map. */
#define TLM_ADDR_STATE                    0x01U  /* integer enum */
#define TLM_ADDR_RPM                      0x02U  /* rpm */
#define TLM_ADDR_TARGET_RPM               0x03U  /* rpm */
#define TLM_ADDR_ELECTRICAL_ANGLE         0x04U  /* mrad */
#define TLM_ADDR_IU                       0x05U  /* mA */
#define TLM_ADDR_IV                       0x06U  /* mA */
#define TLM_ADDR_IW                       0x07U  /* mA */
#define TLM_ADDR_ID                       0x08U  /* mA */
#define TLM_ADDR_IQ                       0x09U  /* mA */
#define TLM_ADDR_IQ_REF                   0x0AU  /* mA */
#define TLM_ADDR_HFI_ANGLE                0x0BU  /* mrad */
#define TLM_ADDR_HFI_SPEED                0x0CU  /* rpm */
#define TLM_ADDR_HFI_ERROR                0x0DU  /* uA-equivalent demod error */
#define TLM_ADDR_HFI_SALIENCY             0x0EU  /* uA demod magnitude */
#define TLM_ADDR_BEMF                     0x0FU  /* mV */
#define TLM_ADDR_OBSERVER_PHASE_ERROR     0x10U  /* mrad */
#define TLM_ADDR_OBSERVER_LOCKED          0x11U  /* 0/1 */
#define TLM_ADDR_ACTIVE_ESTIMATOR         0x12U  /* enum */
#define TLM_ADDR_VBUS                     0x13U  /* mV */
#define TLM_ADDR_FAULT                    0x14U  /* integer fault code */
#define TLM_ADDR_POLARITY_DELTA_THETA     0x15U  /* legacy mrad field; direct-HFI build does not pre-probe */
#define TLM_ADDR_RAW_JDR1                 0x16U  /* ADC counts */
#define TLM_ADDR_RAW_JDR2                 0x17U  /* ADC counts */
#define TLM_ADDR_RAW_JDR3                 0x18U  /* ADC counts */
#define TLM_ADDR_OFFSET_U                 0x19U  /* ADC counts */
#define TLM_ADDR_OFFSET_V                 0x1AU  /* ADC counts */
#define TLM_ADDR_OFFSET_W                 0x1BU  /* ADC counts */
#define TLM_ADDR_PHASE_CURRENT_SUM        0x1CU  /* mA */
#define TLM_ADDR_HFI_CONVERGE_COUNT       0x1DU  /* leaky confidence count, 0..FM_HFI_CONVERGE_SAMPLES */
#define TLM_ADDR_POLARITY_PROBE_IQ         0x1EU  /* mA */
#define TLM_ADDR_POLARITY_DECISION         0x1FU  /* 0 pending,1 direct,2 +pi,3 stronger-probe retry,5 failed */
#define TLM_ADDR_VBUS_ADC_COUNTS           0x20U  /* PF0 / ADC1_IN10 raw counts */
#define TLM_ADDR_VBUS_ADC_VALID            0x21U  /* 0/1 */
#define TLM_ADDR_PHASE_CURRENT_PEAK        0x22U  /* mA, raw instantaneous phase peak */
#define TLM_ADDR_OVERCURRENT_COUNT         0x23U  /* debounced software OCP counter */
#define TLM_ADDR_CURRENT_SUM_BAD_COUNT     0x24U  /* current-sense coherence counter */
#define TLM_ADDR_HFI_OMEGA_RAW_RPM         0x25U  /* raw HFI PLL speed before output LPF, rpm */
#define TLM_ADDR_OBSERVER_RPM              0x26U  /* BEMF observer mechanical rpm */
#define TLM_ADDR_OBSERVER_ANGLE            0x27U  /* BEMF observer angle, mrad */
#define TLM_ADDR_HFI_BEMF_ANGLE_ERR        0x28U  /* observer theta - HFI theta, mrad */
#define TLM_ADDR_VD_CMD                    0x29U  /* applied fundamental Vd, mV */
#define TLM_ADDR_VQ_CMD                    0x2AU  /* applied fundamental Vq, mV */
#define TLM_ADDR_VFUND_MAG                 0x2BU  /* applied fundamental vector magnitude, mV */
#define TLM_ADDR_LAUNCH_PHASE              0x2CU  /* 0 HFI acquire,1 slingshot kick,2 ZVV BEMF read,3 BEMF boost,4 retry/brake,5 RUN */
#define TLM_ADDR_OBS_MOTION_COHERENT        0x2DU  /* 0/1: BEMF/speed/angle agree with HFI */
#define TLM_ADDR_HFI_VQ_LIMIT               0x2EU  /* active HFI Vq ceiling, mV */
#define TLM_ADDR_ALIGN_EXPECTED_ANGLE       0x2FU  /* legacy: no commanded alignment in this build */
#define TLM_ADDR_HFI_TO_FOC_OFFSET          0x30U  /* configured HFI->FOC offset plus runtime branch correction, mrad */
#define TLM_ADDR_HFI_CAL_ANGLE              0x31U  /* mapped HFI angle, mrad */
#define TLM_ADDR_HFI_CAL_ERROR              0x32U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_HFI_CAL_RESULTANT          0x33U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_HFI_CAL_VERIFY_COUNT       0x34U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_HFI_ANGLE_SIGN             0x35U  /* +1000 or -1000 -> sign +1/-1 */
#define TLM_ADDR_HFI_CAL_RAW0               0x36U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_HFI_CAL_RAW1               0x37U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_HFI_CAL_DELTA_RAW          0x38U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_HFI_CAL_MODEL_ERROR        0x39U  /* legacy calibration diagnostic, unused */
#define TLM_ADDR_TIM1_MOE                   0x3AU  /* TIM1 main-output-enable bit */
#define TLM_ADDR_TIM1_CCR1                  0x3BU  /* raw PWM compare */
#define TLM_ADDR_TIM1_CCR2                  0x3CU  /* raw PWM compare */
#define TLM_ADDR_TIM1_CCR3                  0x3DU  /* raw PWM compare */
#define TLM_ADDR_STSPIN_STATUS              0x3EU  /* raw STSPIN STATUS register */
#define TLM_ADDR_CORDIC_SELFTEST_OK         0x3FU  /* 1 = PHASE->COS/SIN self-test passed */
#define TLM_ADDR_CORDIC_SIN45               0x40U  /* sin(pi/4), x1e6 */
#define TLM_ADDR_CORDIC_COS45               0x41U  /* cos(pi/4), x1e6 */
#define TLM_ADDR_HFI_CARRIER_IQ             0x42U  /* carrier-only q current, uA */
#define TLM_ADDR_HFI_CARRIER_MAG            0x43U  /* carrier residual magnitude, uA */
#define TLM_ADDR_HFI_ERR_NORM               0x44U  /* normalized HFI PLL error, x1e6 */
#define TLM_ADDR_FORCED_ANGLE               0x45U  /* legacy protocol field */
#define TLM_ADDR_FORCED_RPM                 0x46U  /* rotating-launch commanded mechanical-equivalent rpm (not measured rotor rpm) */
#define TLM_ADDR_FORCED_VOLTAGE             0x47U  /* reused as latched |Vq| diagnostic during slingshot */
#define TLM_ADDR_FORCED_OBS_GOOD            0x48U  /* observer/HFI qualification count */
#define TLM_ADDR_FORCED_ACCEL_RPM_S         0x49U  /* legacy protocol field; unused */
#define TLM_ADDR_FORCED_OBSERVER_BLEND      0x4AU  /* BEMF observer release/proof blend, x1000 */
#define TLM_ADDR_OBSERVER_LOCK_COUNT         0x4BU  /* internal BEMF lock counter */
#define TLM_ADDR_OBSERVER_UNLOCK_COUNT       0x4CU  /* RUN unlock debounce counter */
#define TLM_ADDR_OBSERVER_POLARITY_PI          0x4DU  /* 0/1: BEMF angle branch shifted by pi */
#define TLM_ADDR_OBSERVER_POLARITY_RESOLVED    0x4EU  /* 0/1: BEMF pi-branch choice against HFI complete */
#define TLM_ADDR_FORCED_VOLTAGE_ROTOR_LAG      0x4FU  /* HFI angle - BEMF observer angle, mrad */
#define TLM_ADDR_HANDOVER_SEED_IQ              0x50U  /* observer-frame Iq captured at current-control takeover, mA */
#define TLM_ADDR_FIRMWARE_BUILD_ID              0x51U  /* exact firmware build signature (26090301 expected) */
#define TLM_ADDR_FORCED_OBS_GOOD_REQUIRED       0x52U  /* compile-time HFI/BEMF qualification requirement */
#define TLM_ADDR_STATE6_ENTRY_GOOD_COUNT        0x53U  /* observer/HFI good-count latched at handover entry */
#define TLM_ADDR_STATE6_ENTRY_BLEND             0x54U  /* observer release blend at handover entry, x1000 */
#define TLM_ADDR_FULL_RELEASE_QUALIFIED         0x55U  /* 1 only after full-release proof */
#define TLM_ADDR_CONTROL_FEEDBACK_BLEND          0x56U  /* 0 HFI authority, 0..1000 estimator blend, 1000 BEMF */
#define TLM_ADDR_CONTROL_ANGLE                   0x57U  /* actual Park/InvPark angle, mrad */
#define TLM_ADDR_CONTROL_SPEED_FEEDBACK          0x58U  /* speed feedback represented by current frame, rpm */
#define TLM_ADDR_IF_IQ_REF                       0x59U  /* legacy name: current HFI-start Iq reference, mA */
#define TLM_ADDR_FEEDBACK_GOOD_COUNT             0x5AU  /* reserved/legacy qualification counter */
#define TLM_ADDR_IF_RAW_SEED_ID                  0x5BU  /* legacy I-f field, unused */
#define TLM_ADDR_IF_RAW_SEED_IQ                  0x5CU  /* legacy I-f field, unused */
#define TLM_ADDR_IF_PI_SEEDED                    0x5DU  /* legacy I-f field, unused */
#define TLM_ADDR_COAST_GOOD_COUNT                 0x5EU  /* legacy coast field, unused */
#define TLM_ADDR_CATCH_IQ_LIMIT                   0x5FU  /* legacy catch field, unused */
#define TLM_ADDR_OBSERVER_HOLDOVER_ACTIVE         0x60U  /* 1 while using propagated last trustworthy real angle */
#define TLM_ADDR_OBSERVER_HOLDOVER_COUNT          0x61U  /* holdover samples */
#define TLM_ADDR_LAST_GOOD_OBSERVER_ANGLE         0x62U  /* mrad */
#define TLM_ADDR_LAST_GOOD_OBSERVER_RPM           0x63U  /* rpm */
#define TLM_ADDR_SLING_PHASE                      0x64U  /* 0 idle,1 forward rotating launch,2 blank,3 ZVV observe,5 accepted */
#define TLM_ADDR_SLING_ATTEMPT                    0x65U  /* coherent launch index; 1 in this build */
#define TLM_ADDR_SLING_AXIS_QUADRATURE            0x66U  /* retained protocol field; 0 in this build */
#define TLM_ADDR_SLING_SEED_ANGLE                 0x67U  /* initial converged HFI seed angle, mrad */
#define TLM_ADDR_SLING_IQ_CMD                     0x68U  /* slingshot current command, mA */
#define TLM_ADDR_SLING_BEMF                       0x69U  /* ZVV-derived BEMF magnitude, mV */
#define TLM_ADDR_SLING_RPM                        0x6AU  /* ZVV-derived mechanical rpm */
#define TLM_ADDR_SLING_ANGLE                      0x6BU  /* ZVV-derived electrical rotor angle, mrad */
#define TLM_ADDR_SLING_VALID_SAMPLES              0x6CU  /* accepted ZVV angle samples */
#define TLM_ADDR_SLING_PEAK_PHASE_CURRENT         0x6DU  /* latched max phase-current magnitude during kick, mA */
#define TLM_ADDR_SLING_PEAK_IQ                    0x6EU  /* latched max |Iq| during kick, mA */
#define TLM_ADDR_SLING_PEAK_VQ                    0x6FU  /* latched max |Vq| command during kick, mV */

/* 40-kHz real-time timing / crossover diagnostics added in build 26082730; ISR implementation optimized in 26082731. */
#define TLM_ADDR_ISR_CYCLES_LAST                   0x70U  /* DWT cycles spent in previous control ISR */
#define TLM_ADDR_ISR_CYCLES_MAX                    0x71U  /* max DWT cycles seen since boot */
#define TLM_ADDR_ISR_OVERRUN_COUNT                 0x72U  /* ISR execution longer than one 25 us budget */
#define TLM_ADDR_ISR_INTERVAL_CYCLES               0x73U  /* cycles between ADC callback entries */
#define TLM_ADDR_ISR_INTERVAL_MAX                  0x74U  /* max callback-entry interval */
#define TLM_ADDR_ISR_LATE_COUNT                    0x75U  /* callback interval > 1.5 nominal periods */
#define TLM_ADDR_OBSERVER_PHASE_VALID              0x76U  /* 1 only when angle+flux-speed coherence is safe for torque */
#define TLM_ADDR_RAW_FLUX_RPM                      0x77U  /* independent PM-flux mechanical rpm */
#define TLM_ADDR_ACTIVE_TARGET_RPM                 0x78U  /* ramped internal speed-loop target */
#define TLM_ADDR_REJOIN_ANGLE_ERROR                0x79U  /* raw flux angle - control angle, mrad */
#define TLM_ADDR_SIXSTEP_BEMF_SYNC                 0x7AU  /* 1 once six-step commutation follows measured BEMF angle */
#define TLM_ADDR_SIXSTEP_MEASURED_RPM              0x7BU  /* real flux rpm while six-step owns bridge */
#define TLM_ADDR_SIXSTEP_SECTOR                    0x7CU  /* physical commutation sector 0..5 */
#define TLM_ADDR_SIXSTEP_SECTOR_PHASE              0x7DU  /* phase through present 60-degree sector, x1000 */
#define TLM_ADDR_SIXSTEP_SPEED_ERROR               0x7EU  /* measured rpm - six-step command rpm */
#define TLM_ADDR_SIXSTEP_PHASE_ERROR               0x7FU  /* raw flux angle - open-loop rotor reference, mrad */
#define TLM_ADDR_SIXSTEP_COMMUTATION_COUNT         0x80U  /* monotonic +/-1 sector transitions */
#define TLM_ADDR_SIXSTEP_SYNC_GOOD_COUNT           0x81U  /* consecutive qualified observer samples */
#define TLM_ADDR_SIXSTEP_SYNC_BAD_COUNT            0x82U  /* consecutive invalid samples after sync */
#define TLM_ADDR_SIXSTEP_FLOATING_BEMF             0x83U  /* reconstructed Hi-Z phase BEMF, mV */
#define TLM_ADDR_SIXSTEP_FLOATING_CURRENT          0x84U  /* current in selected Hi-Z phase, mA signed */
#define TLM_ADDR_SIXSTEP_ZC_RPM                    0x85U  /* RPM measured directly from floating-phase ZC period */
#define TLM_ADDR_SIXSTEP_ZC_HOLDOVER               0x86U  /* predicted sectors since last real ZC */

/* Build 26091604/05 startup/catch diagnostics. */
#define TLM_ADDR_TRAP_COMMAND_RPM                   0x87U  /* signed six-step field-speed command, rpm */
#define TLM_ADDR_TRAP_LOCK_CONFIDENT                0x88U  /* 1 once strict 26090327 six-step lock proof is complete */
#define TLM_ADDR_TRAP_LOCKED_COMMUTATIONS           0x89U  /* legal commutations since BEMF/flux lock */
#define TLM_ADDR_EARLY_FOC_WINDOW                   0x8AU  /* 1 on the exact ISR where all early-FOC gates are satisfied */
#define TLM_ADDR_EARLY_FOC_ANGLE_ERROR              0x8BU  /* direct-BEMF angle vs six-step rotor angle, mrad */
#define TLM_ADDR_EARLY_FOC_SPEED_ERROR              0x8CU  /* raw flux rpm vs qualified six-step rpm */
#define TLM_ADDR_POS_SLIP_LIMIT_ACTIVE              0x8DU  /* positive bootstrap slip limiter active */
#define TLM_ADDR_POS_SLIP_LIMIT_RPM                 0x8EU  /* current positive field-speed cap, rpm */
#define TLM_ADDR_TRAP_FLUX_AVAILABLE                0x8FU  /* physical PM-flux speed sample available */
#define TLM_ADDR_TRAP_BEMF_THETA_VALID              0x90U  /* direct BEMF rotor-angle sample valid */
#define TLM_ADDR_HFI_POLARITY_PROVEN                0x91U  /* HFI/motion branch resolved */
#define TLM_ADDR_FOC_CAPTURE_GOOD_COUNT             0x92U  /* consecutive coherent FOC-capture samples */
#define TLM_ADDR_FOC_CAPTURE_ENTRY_SAMPLES          0x93U  /* samples spent in current FOC capture attempt */
#define TLM_ADDR_FOC_CAPTURE_PEAK_BEMF              0x94U  /* peak BEMF seen in current capture, mV */
#define TLM_ADDR_FOC_RETRY_COOLDOWN                 0x95U  /* six-step samples before another FOC probe is allowed */
#define TLM_ADDR_TRAP_FIELD_SLIP_RPM                 0x96U  /* signed field-command minus measured rotor rpm */
#define TLM_ADDR_POS_CATCH_SLIP_OK                   0x97U  /* 1 when positive field/rotor slip is safe for FOC */
#define TLM_ADDR_POS_CATCH_BLOCK_BITS                0x98U  /* positive early-FOC gate bitmask */
#define TLM_ADDR_FOC_ENTRY_RPM                       0x99U  /* latched measured rpm at last FOC probe */
#define TLM_ADDR_FOC_ENTRY_COMMAND_RPM               0x9AU  /* latched six-step field command at FOC probe */
#define TLM_ADDR_FOC_ENTRY_SLIP_RPM                  0x9BU  /* latched field/rotor slip at FOC probe */
#define TLM_ADDR_FOC_ENTRY_BEMF                      0x9CU  /* latched BEMF magnitude at FOC probe, mV */
#define TLM_ADDR_FOC_ENTRY_ANGLE_ERROR               0x9DU  /* latched BEMF-vs-sector angle error, mrad */
#define TLM_ADDR_FOC_ENTRY_SPEED_ERROR               0x9EU  /* latched flux-vs-measured speed error, rpm */
#define TLM_ADDR_FOC_ENTRY_PHASE_CURRENT             0x9FU  /* latched phase-current peak at FOC probe, mA */

/* Build 26091607 direction-symmetric topology-boundary diagnostics. */
#define TLM_ADDR_HANDOVER_SIGNED_MOTION_OK           0xA0U
#define TLM_ADDR_HANDOVER_EXTRA_PROOF_OK             0xA1U
#define TLM_ADDR_HANDOVER_POLARITY_PI                0xA2U
#define TLM_ADDR_FOC_ENTRY_MEASURED_RPM              0xA3U
#define TLM_ADDR_FOC_ENTRY_FLUX_RPM                  0xA4U
#define TLM_ADDR_FOC_ENTRY_ID                        0xA5U
#define TLM_ADDR_FOC_ENTRY_IQ                        0xA6U
#define TLM_ADDR_TOPOLOGY_FLUX_PHASE_ERROR           0xA7U
#define TLM_ADDR_TOPOLOGY_FLUX_RPM                   0xA8U
#define TLM_ADDR_FIRST_FOC_SAMPLE_VALID              0xA9U
#define TLM_ADDR_FIRST_FOC_FLUX_RPM                  0xAAU
#define TLM_ADDR_FIRST_FOC_PHASE_ERROR               0xABU

/* Build 26091701 negative false-lock / release diagnostics. */
#define TLM_ADDR_NEG_RUN_ADMISSION_READY              0xACU
#define TLM_ADDR_NEG_RUN_FALSE_LOCK_COUNT             0xADU
#define TLM_ADDR_NEG_RUN_FALSE_LOCK_ACTIVE            0xAEU
#define TLM_ADDR_NEG_RUN_QUALITY_BITS                 0xAFU
#define TLM_ADDR_NEG_RUN_IQ_LIMIT                     0xB0U
#define TLM_ADDR_NEG_RUN_RELEASE_K                    0xB1U
#define TLM_ADDR_NEG_RUN_RELEASE_COUNT                0xB2U

/* Stage-B V13 shadow-only telemetry. Existing channel addresses remain unchanged.
 * 0xB3+ are intentionally outside all existing legacy addresses. */
#define TLM_ADDR_V13_IU                         0xB3U /* offset binary mA */
#define TLM_ADDR_V13_IV                         0xB4U
#define TLM_ADDR_V13_IW                         0xB5U
#define TLM_ADDR_V13_ALPHA                      0xB6U /* alpha * 1e6 */
#define TLM_ADDR_V13_BASE_ALPHA                 0xB7U /* alpha * 1e6 */
#define TLM_ADDR_V13_CONFIDENCE                 0xB8U /* confidence * 1e6 */
#define TLM_ADDR_V13_VOLTAGE_EVIDENCE           0xB9U /* evidence * 1e6 */
#define TLM_ADDR_V13_CYCLES_LAST                0xBAU
#define TLM_ADDR_V13_CYCLES_MAX                 0xBBU
#define TLM_ADDR_V13_SAMPLE_INDEX                0xBCU
#define TLM_ADDR_V13_REFERENCE_COMPLETE         0xBDU /* 1: full current activity + lean body ported */
#define TLM_ADDR_V13_VOLTAGE_VALID               0xBEU /* 0: Hi-Z/not driving, voltage evidence masked */
#define TLM_ADDR_V13_DECIMATION                  0xBFU
#define TLM_ADDR_V13_REJECTED_SAMPLES            0xC0U
#define TLM_ADDR_V13_MISSED_SAMPLES          0xC3U
#define TLM_ADDR_V13_OBSERVER_BLEND          0xC4U /* micro-units, 0..150000 */
#define TLM_ADDR_V13_STEP_LATCH                  0xC1U /* * 1e6 */
#define TLM_ADDR_V13_CPU_CYCLES_LAST             0xC5U /* ISR-subtracted estimate */
#define TLM_ADDR_V13_CPU_CYCLES_MAX              0xC6U /* ISR-subtracted estimate */
#define TLM_ADDR_V13_FAST_MODE                    0xC7U /* 0 inactive, 1 fast, 2 fallback */
#define TLM_ADDR_V13_GAP_RESYNCS 0xCBU /* background sequence resets after dropped frames */
#define TLM_ADDR_V13_RESEARCH_ALPHA          0xCCU
#define TLM_ADDR_V13_SESSION_RESETS         0xCDU
#define TLM_ADDR_V13_CONTINUITY_INVALID     0xCEU
#define TLM_ADDR_V13_CAPTURE_STATUS         0xCFU
#define TLM_ADDR_V13_CAPTURE_COUNT          0xD0U
#define TLM_ADDR_CAPTURE_SEQ                0xD1U /* index 0..127 */
#define TLM_ADDR_CAPTURE_WORD0              0xD2U /* 16 words @ D2..E1 */
#define TLM_ADDR_V13_NOEVENT_IU              0xE2U
#define TLM_ADDR_V13_NOEVENT_IV              0xE3U
#define TLM_ADDR_V13_NOEVENT_IW              0xE4U
#define TLM_ADDR_V13_PROJECTION_IU           0xE5U
#define TLM_ADDR_V13_PROJECTION_IV           0xE6U
#define TLM_ADDR_V13_PROJECTION_IW           0xE7U
#define TLM_ADDR_V13_AGE_TICKS_LAST          0xE8U /* foreground completion latency, units 25us */
#define TLM_ADDR_V13_AGE_TICKS_MAX           0xE9U /* max same RUN */
#define TLM_ADDR_V13_40K_GAIN                0xEAU /* alpha*1e6 */
#define TLM_ADDR_V13_40K_ALPHA               0xEBU /* alpha current, mA */
#define TLM_ADDR_V13_40K_BETA                0xECU /* beta current, mA */
#define TLM_ADDR_V13_40K_CYCLES              0xEDU /* sampled every 128th ISR */
#define TLM_ADDR_V13_40K_CYCLES_MAX          0xEEU
#define TLM_ADDR_V13_40K_SAMPLES             0xEFU
#define TLM_ADDR_V13_40K_GUARD_EVENTS        0xF0U
#define TLM_ADDR_V13_40K_GAIN_AGE           0xF1U
#define TLM_ADDR_V13_FAST_OVERRUNS                0xC8U /* fast updates exceeding 25us */
#define TLM_ADDR_RUN_OBSERVER_CYCLES_LAST        0xC9U /* 20kHz RUN observer+PLL DWT */
#define TLM_ADDR_RUN_OBSERVER_CYCLES_MAX         0xCAU /* peak DWT observer+PLL */

void Telemetry_Init(void);
void Telemetry_Service(const FM_Telemetry *t);

#endif
