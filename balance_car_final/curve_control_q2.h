#ifndef CURVE_CONTROL_Q2_H
#define CURVE_CONTROL_Q2_H

#include <stdint.h>

#include "curve_params_q2.h"
#include "pid_q2.h"

typedef enum
{
    CURVE_STATE_WAIT = 0,
    CURVE_STATE_STARTING,
    CURVE_STATE_RUNNING,
    CURVE_STATE_FINISH_ARMED,
    CURVE_STATE_STOPPED,
    CURVE_STATE_FAULT
} CurveRaceState;

typedef enum
{
    CURVE_FAULT_NONE = 0,
    CURVE_FAULT_EMERGENCY_STOP,
    CURVE_FAULT_HARD_TIMEOUT
} Q2CurveControlFault;

typedef enum
{
    CURVE_FINISH_TRIGGER_NONE = 0,
    CURVE_FINISH_TRIGGER_RAW_FAST,
    CURVE_FINISH_TRIGGER_FILTERED_FALLBACK
} CurveFinishTrigger;

typedef struct
{
    CurveRaceState state;
    Q2CurveControlFault fault;
    uint32_t elapsed_ms;
    uint8_t raw_mask;
    uint8_t filtered_mask;
    uint8_t start_line_cleared;
    uint8_t finish_confirm_count;
    uint8_t raw_finish_count;
    uint8_t approach_stage;
    CurveFinishTrigger finish_trigger;
    uint8_t gap_hold;
    int16_t error_mm_q8;
    int16_t turn_pwm;
    int16_t left_target_pwm;
    int16_t right_target_pwm;
    int16_t left_pwm;
    int16_t right_pwm;
} Q2CurveControlOutput;

typedef struct
{
    Q2_PID_t line_pid;
    Q2CurveControlOutput output;
    uint8_t history[CURVE_SENSOR_COUNT];
    uint8_t sample_count;
    uint8_t clear_count;
} Q2CurveControl;

void Q2CurveControl_Init(Q2CurveControl *control);
uint8_t Q2CurveControl_Start(Q2CurveControl *control);
void Q2CurveControl_PushSample1ms(Q2CurveControl *control, uint8_t raw_mask);
Q2CurveControlOutput Q2CurveControl_Step5ms(Q2CurveControl *control);
void Q2CurveControl_StopImmediate(
    Q2CurveControl *control,
    Q2CurveControlFault fault);
Q2CurveControlOutput Q2CurveControl_GetOutput(const Q2CurveControl *control);
const char *Q2CurveControl_StateName(CurveRaceState state);

/* Pure helpers exposed for deterministic host tests. */
uint8_t Q2CurveControl_FilteredMask(const Q2CurveControl *control);
int16_t Q2CurveControl_ErrorMmQ8(uint8_t filtered_mask);

#endif
