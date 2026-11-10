#ifndef V13_CAPTURE_H
#define V13_CAPTURE_H
#include <stdint.h>
/* Build 26101703: 256 consecutive 20kHz updates = 12.8ms, 8192 B SRAM.
 * Captured only after explicit UART command 15 07 BC 00 00.
 * Actual V13 shadow sample and its matched input are paired in one ISR.
 * Quantization: 4 mA/count, duty 0..65535, bus 1 mV/count,
 * alpha/event 1/65535, CPU cycles raw. Projection is computed offline.
 * TIM1 CCRs represent command registers, not measured inverter voltages.
 */
#define V13_CAPTURE_LENGTH 256U
#define V13_CAPTURE_SCALE_A 0.004f
#define V13_CAPTURE_FORMAT 0x13200301U
/* exactly eight 32-bit little-endian words */
typedef struct {
    uint32_t isr_tick;
    int16_t raw[3];
    int16_t frozen[3];
    uint16_t duty[3];
    uint16_t bus_mv;
    uint16_t alpha;
    uint16_t voltage_event;
    uint16_t cpu_cycles;
    uint16_t flags; /* b0 PWM valid, b1 invalid input; b2 observer20k active */
} V13_CaptureFrame;
typedef struct {
    volatile uint32_t status; /* 0 idle, 1 collecting, 2 ready */
    volatile uint32_t count;
    volatile uint32_t session;
    volatile uint32_t reset_count;
    volatile uint32_t discontinuities;
    V13_CaptureFrame frames[V13_CAPTURE_LENGTH];
} V13_CaptureBuffer;
extern V13_CaptureBuffer g_v13_ram_capture;
uint8_t V13_CaptureRead(uint32_t index, const V13_CaptureFrame **out);
#endif
