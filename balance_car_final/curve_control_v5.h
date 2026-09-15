#ifndef CURVE_CONTROL_V5_H
#define CURVE_CONTROL_V5_H

#include <stdint.h>

#include "curve_params_v5.h"
#include "curve_pid.h"

typedef enum
{
    CURVE_STATE_WAIT = 0,
    CURVE_STATE_STARTING,
    CURVE_STATE_RUNNING,
    CURVE_STATE_FINISH_ARMED,
    CURVE_STATE_STOP_DELAY,
    CURVE_STATE_STOPPED,
    CURVE_STATE_FAULT
} CurveRaceState;

typedef enum
{
    CURVE_FAULT_NONE = 0,
    CURVE_FAULT_EMERGENCY_STOP,
    CURVE_FAULT_HARD_TIMEOUT
} CurveControlV5Fault;

typedef enum
{
    CURVE_FINISH_TRIGGER_NONE = 0,
    CURVE_FINISH_TRIGGER_RAW_FAST,
    CURVE_FINISH_TRIGGER_FILTERED_FALLBACK
} CurveFinishTrigger;

typedef struct
{
    CurveRaceState state;
    CurveControlV5Fault fault;
    uint32_t elapsed_ms;
    uint32_t stop_delay_start_ms;
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
} CurveControlV5Output;

typedef struct
{
    Curve_PID_t line_pid;
    CurveControlV5Output output;
    uint8_t history[CURVE_SENSOR_COUNT];
    uint8_t sample_count;
    uint8_t clear_count;
} CurveControlV5;

void CurveControlV5_Init(CurveControlV5 *control);
uint8_t CurveControlV5_Start(CurveControlV5 *control);
void CurveControlV5_PushSample1ms(CurveControlV5 *control, uint8_t raw_mask);
CurveControlV5Output CurveControlV5_Step5ms(CurveControlV5 *control);
void CurveControlV5_StopImmediate(
    CurveControlV5 *control,
    CurveControlV5Fault fault);
CurveControlV5Output CurveControlV5_GetOutput(const CurveControlV5 *control);
const char *CurveControlV5_StateName(CurveRaceState state);

/* Pure helpers exposed for deterministic host tests. */
uint8_t CurveControlV5_FilteredMask(const CurveControlV5 *control);
int16_t CurveControlV5_ErrorMmQ8(uint8_t filtered_mask);

#endif
