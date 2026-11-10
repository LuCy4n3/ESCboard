#ifndef V13_LEAN_H
#define V13_LEAN_H

#include <stdint.h>

/* Full original v13 lean + supplied v12_current_activity.m, causal 1 kHz
 * streaming implementation.  Reference-complete for valid voltage samples;
 * six-step floating-leg voltage innovation is intentionally invalidated. */
#define V13_REFERENCE_COMPLETE 1U

typedef struct {
    float alpha, early_added, voltage_added, voltage_evidence;
    float base_alpha, confidence, step_latch, event_latch;
    uint8_t input_valid;
} V13_Diagnostics;

typedef struct {
    float out[2], fast[2], drift[2];
    float activity_fast[2], activity_slow[2], activity_energy;
    uint8_t activity_initialized;
    float prev_residual[2], prev_voltage[2];
    float step_sum[2], groups[4][2];
    float confidence, step_latch, severity, early_latch, event_latch;
    float step_decay, early_decay, event_decay;
    uint8_t history, initialized, step_count, group_ready, voltage_history;
    uint32_t rejected_samples;
    V13_Diagnostics diag;
} V13_State;

void V13_Init(V13_State *s, float fs_hz);
/* ISR-safe session reset; preserves precomputed exponential decays. */
void V13_ResetSession(V13_State *s);
/* duty_abc must represent previous-cycle latched TIM1 duty (0..1).
 * If voltage_valid==0, electrical-event evidence is inhibited, and voltage
 * history is restarted at the first subsequent valid interval.
 * Currents in real A, bus in V. Per-sample call at the actual ISR frequency.
 * The output is independent of motor control; never feed to safety checks. */
void V13_Update(V13_State *s, const float current_abc[3],
                const float duty_abc[3], float bus_v,
                uint8_t voltage_valid, float filtered_abc[3]);

/* Research-only: identical to Lean except the PWM voltage-event branch is
 * completely disabled. NEVER claim this is the frozen reference. */
void V13_UpdateNoVoltageResearch(V13_State *s, const float current_abc[3],
    const float duty_abc[3], float bus_v, uint8_t voltage_valid, float filtered_abc[3]);

#endif
