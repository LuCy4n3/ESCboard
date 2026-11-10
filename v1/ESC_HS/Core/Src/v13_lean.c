#include "v13_lean.h"
#include <math.h>
#include <string.h>

#define V13_INV_SQRT3   0.5773502691896258f
#define V13_SQRT3_HALF 0.8660254037844386f

static inline float clip(float x, float a, float b)
{
    return (x < a) ? a : ((x > b) ? b : x);
}
/* Constant reciprocals avoid run-time FP division on Cortex-M4F.
 * Thresholds are fixed in the frozen MATLAB V13 algorithm. */
static inline float membership_inv(float x, float lo, float inv_span)
{
    float f = clip((x - lo) * inv_span, 0.0f, 1.0f);
    return f * f * (3.0f - 2.0f * f);
}
static inline float vmin(float a, float b) { return (a < b) ? a : b; }
static inline float vmax(float a, float b) { return (a > b) ? a : b; }
static inline float dot2(const float a[2], const float b[2])
{
    return a[0] * b[0] + a[1] * b[1];
}
static void reconstruction(const V13_State *s, float abc[3])
{
    abc[0] = s->out[0];
    abc[1] = -0.5f * s->out[0] + V13_SQRT3_HALF * s->out[1];
    abc[2] = -0.5f * s->out[0] - V13_SQRT3_HALF * s->out[1];
}
void V13_Init(V13_State *s, float fs_hz)
{
    if (s == 0) return;
    memset(s, 0, sizeof(*s));
    if (!isfinite(fs_hz) || fs_hz < 1.0f) fs_hz = 40000.0f;
    s->step_decay  = expf(-1.0f / (fs_hz * 0.008f));
    s->early_decay = expf(-1.0f / (fs_hz * 0.00025f));
    s->event_decay = expf(-1.0f / (fs_hz * 0.002f));
}
void V13_ResetSession(V13_State *s)
{
    if (!s) return;
    float d0 = s->step_decay, d1 = s->early_decay, d2 = s->event_decay;
    memset(s, 0, sizeof(*s));
    s->step_decay = d0;
    s->early_decay = d1;
    s->event_decay = d2;
}

/* Target-side O3 speed optimization, without fast-math reassociation.
 * Cortex-M4F FPU handles the scalar operations; preserve V13 mathematics. */
#if defined(__GNUC__)
__attribute__((hot,optimize("O3")))
#endif
static void V13_UpdateCore(V13_State *s, const float current_abc[3],
                const float duty_abc[3], float bus_v,
                uint8_t voltage_valid, uint8_t event_enabled, float filtered_abc[3])
{
    if (!s || !current_abc || !duty_abc || !filtered_abc) return;
    if (!isfinite(current_abc[0]) || !isfinite(current_abc[1]) ||
        !isfinite(current_abc[2]) || !isfinite(bus_v) || bus_v <= 0.0f ||
        (voltage_valid && (!isfinite(duty_abc[0]) ||
          !isfinite(duty_abc[1]) || !isfinite(duty_abc[2])))) {
        ++s->rejected_samples;
        s->diag.input_valid = 0U;
        if (s->initialized) reconstruction(s, filtered_abc);
        else filtered_abc[0] = filtered_abc[1] = filtered_abc[2] = 0.0f;
        return;
    }
    const float z[2] = {
        (2.0f * current_abc[0] - current_abc[1] - current_abc[2]) * 0.3333333333333333f,
        (current_abc[1] - current_abc[2]) * V13_INV_SQRT3
    };
    if (!s->initialized) {
        s->out[0] = s->fast[0] = z[0];
        s->out[1] = s->fast[1] = z[1];
        s->initialized = 1U;
    }
    /* v12_current_activity.m: fully causal independent activity schedule.
     * All coefficients copied verbatim from the original MATLAB. */
    if (!s->activity_initialized) {
        s->activity_fast[0] = s->activity_slow[0] = z[0];
        s->activity_fast[1] = s->activity_slow[1] = z[1];
        s->activity_initialized = 1U;
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        s->activity_fast[i] += 0.28f * (z[i] - s->activity_fast[i]);
        s->activity_slow[i] += 0.02f * (z[i] - s->activity_slow[i]);
    }
    float activity_d0 = s->activity_fast[0] - s->activity_slow[0];
    float activity_d1 = s->activity_fast[1] - s->activity_slow[1];
    s->activity_energy += 0.02f * ((activity_d0 * activity_d0 +
                                  activity_d1 * activity_d1) - s->activity_energy);
    const float activity_weight = membership_inv(s->activity_energy, 0.002025f, 8.30047727744f);
    const float base = 0.025f + 0.70f * activity_weight;
    const float fast_alpha = vmin(0.97f, base + (1.0f - base) * 0.357142857142857f);
    float delta[2];
    for (unsigned i = 0; i != 2U; ++i) {
        s->fast[i] += 0.32f * (z[i] - s->fast[i]);
        delta[i] = s->fast[i] - s->out[i];
        s->drift[i] += 0.012f * (delta[i] - s->drift[i]);
    }
    float drift2 = dot2(s->drift, s->drift);
    float evidence = 0.0f;
    if (drift2 > 0.015f * 0.015f) {
        float dot_move = dot2(delta, s->drift);
        float delta2 = dot2(delta, delta);
        if (dot_move > 0.0f && delta2 > 1e-12f) {
            float alignment2 = (dot_move * dot_move) / (delta2 * drift2 + 1e-12f);
            float m = membership_inv((drift2 - 0.000225f) * 296.2962962963f, 0.0f, 1.0f);
            evidence = m * (0.4f + 0.6f * clip(alignment2, 0.0f, 1.0f));
        }
    }
    float attack = (evidence > s->confidence) ? 0.30f : 0.04f;
    s->confidence += attack * (evidence - s->confidence);
    float alpha = base + (fast_alpha - base) * s->confidence;

    /* V13 'lean' confirmed eight-sample group path. Exactly one update per 8. */
    s->step_latch *= s->step_decay;
    s->severity *= s->step_decay;
    ++s->step_count;
    s->step_sum[0] += z[0]; s->step_sum[1] += z[1];
    if (s->step_count == 8U) {
        float grp[2] = {s->step_sum[0] * 0.125f, s->step_sum[1] * 0.125f};
        for (unsigned j = 0U; j != 3U; ++j) {
            s->groups[j][0] = s->groups[j + 1U][0];
            s->groups[j][1] = s->groups[j + 1U][1];
        }
        s->groups[3][0] = grp[0]; s->groups[3][1] = grp[1];
        if (s->group_ready < 4U) ++s->group_ready;
        s->step_count = 0U;
        s->step_sum[0] = s->step_sum[1] = 0.0f;
        if (s->group_ready == 4U) {
            float before[2], after[2], jump[2], aa[2], bb[2];
            for (unsigned i = 0; i != 2U; ++i) {
                before[i] = 0.5f * (s->groups[0][i] + s->groups[1][i]);
                after[i]  = 0.5f * (s->groups[2][i] + s->groups[3][i]);
                jump[i] = after[i] - before[i];
                aa[i] = s->groups[2][i] - before[i];
                bb[i] = s->groups[3][i] - before[i];
            }
            float j2 = dot2(jump, jump);
            float pers = dot2(aa, bb);
            if (pers > 0.0f) {
                float pers2 = (pers * pers) / (dot2(aa, aa) * dot2(bb, bb) + 1e-12f);
                float snr2 = j2 * 341.912127799f; /* reciprocal sigmaShort^2 */
                float step_ev = membership_inv(snr2, 4.84f, 0.089605734767f) * pers2;
                if (step_ev > 0.35f) {
                    s->step_latch = vmax(s->step_latch, 0.70f * step_ev);
                    s->severity = vmax(s->severity,
                        membership_inv(snr2, 11.56f, 0.094966761633f));
                }
            }
        }
    }
    alpha += (0.55f + 0.40f * s->severity) * s->step_latch * (fast_alpha - alpha);

    float residual[2] = {z[0] - s->out[0], z[1] - s->out[1]};
    float early_extra = 0.0f;
    if (s->history) {
        float r2 = dot2(residual, residual);
        float last2 = dot2(s->prev_residual, s->prev_residual);
        float coherence = dot2(residual, s->prev_residual);
        float level_ev = 0.0f;
        if (coherence > 0.0f && coherence * coherence > 0.64f * r2 * last2)
            level_ev = membership_inv(vmin(r2, last2) * 208.333333333333f, 4.0f, 0.121212121212f);
        float alpha_before = alpha;
        s->early_latch = vmax(s->early_latch * s->early_decay, level_ev);
        if (s->early_latch > 0.0f) {
            float quick_cap = vmin(0.97f, base + (1.0f - base) * 0.23f);
            float quick_alpha = vmin(quick_cap,
                base + 0.55f * s->early_latch * (fast_alpha - base));
            alpha = vmax(alpha, quick_alpha);
        }
        early_extra = vmax(0.0f, alpha - alpha_before);
    }

    /* MATLAB voltage innovation. Invalid floating-leg topology masks voltage
     * evidence rather than inventing a 3-phase duty for true six-step. */
    float event = 0.0f;
    if (event_enabled && voltage_valid) {
        float v_now[2] = {
            bus_v * (2.0f * duty_abc[0] - duty_abc[1] - duty_abc[2]) * 0.3333333333333333f,
            bus_v * (duty_abc[1] - duty_abc[2]) * V13_INV_SQRT3
        };
        if (s->voltage_history) {
            float dv0 = v_now[0] - s->prev_voltage[0];
            float dv1 = v_now[1] - s->prev_voltage[1];
            event = membership_inv(dv0 * dv0 + dv1 * dv1, 0.000625f, 304.761904762f);
        }
        s->prev_voltage[0] = v_now[0];
        s->prev_voltage[1] = v_now[1];
        s->voltage_history = 1U;
    } else {
        s->voltage_history = 0U;
    }
    s->event_latch = event_enabled ?
        vmax(s->event_latch * s->event_decay, event) : 0.0f;
    float before_event = alpha;
    alpha = vmax(alpha, base + (0.92f - base) * s->event_latch);
    float voltage_extra = vmax(0.0f, alpha - before_event);

    alpha = clip(alpha, 0.008f, 0.97f);
    s->out[0] += alpha * (z[0] - s->out[0]);
    s->out[1] += alpha * (z[1] - s->out[1]);
    reconstruction(s, filtered_abc);
    s->prev_residual[0] = residual[0];
    s->prev_residual[1] = residual[1];
    s->history = 1U;
    s->diag.alpha = alpha;
    s->diag.base_alpha = base;
    s->diag.early_added = early_extra;
    s->diag.voltage_added = voltage_extra;
    s->diag.voltage_evidence = event;
    s->diag.confidence = s->confidence;
    s->diag.step_latch = s->step_latch;
    s->diag.event_latch = s->event_latch;
    s->diag.input_valid = 1U;
}

/* Frozen reference: all V13 mathematical branches intact. */
#if defined(__GNUC__)
__attribute__((hot,optimize("O3")))
#endif
void V13_Update(V13_State *s, const float current_abc[3],
                const float duty_abc[3], float bus_v, uint8_t voltage_valid,
                float filtered_abc[3])
{
    V13_UpdateCore(s, current_abc, duty_abc, bus_v, voltage_valid, 1U, filtered_abc);
}
/* Modified A/B research comparator: disable voltage innovation only. */
#if defined(__GNUC__)
__attribute__((hot,optimize("O3")))
#endif
void V13_UpdateNoVoltageResearch(V13_State *s, const float current_abc[3],
                const float duty_abc[3], float bus_v, uint8_t voltage_valid,
                float filtered_abc[3])
{
    V13_UpdateCore(s, current_abc, duty_abc, bus_v, voltage_valid, 0U, filtered_abc);
}
