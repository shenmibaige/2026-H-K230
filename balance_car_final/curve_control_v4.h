#ifndef CURVE_CONTROL_V4_H
#define CURVE_CONTROL_V4_H

#include <stdint.h>

#include "curve_params_v4.h"
#include "curve_pid.h"

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
} CurveControlV4Fault;

typedef enum
{
    CURVE_FINISH_TRIGGER_NONE = 0,
    CURVE_FINISH_TRIGGER_RAW_FAST,
    CURVE_FINISH_TRIGGER_FILTERED_FALLBACK
} CurveFinishTrigger;

typedef struct
{
    CurveRaceState state;
    CurveControlV4Fault fault;
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
} CurveControlV4Output;

typedef struct
{
    Curve_PID_t line_pid;
    CurveControlV4Output output;
    uint8_t history[CURVE_SENSOR_COUNT];
    uint8_t sample_count;
    uint8_t clear_count;
} CurveControlV4;

void CurveControlV4_Init(CurveControlV4 *control);
uint8_t CurveControlV4_Start(CurveControlV4 *control);
void CurveControlV4_PushSample1ms(CurveControlV4 *control, uint8_t raw_mask);
CurveControlV4Output CurveControlV4_Step5ms(CurveControlV4 *control);
void CurveControlV4_StopImmediate(
    CurveControlV4 *control,
    CurveControlV4Fault fault);
CurveControlV4Output CurveControlV4_GetOutput(const CurveControlV4 *control);
const char *CurveControlV4_StateName(CurveRaceState state);

/* Pure helpers exposed for deterministic host tests. */
uint8_t CurveControlV4_FilteredMask(const CurveControlV4 *control);
int16_t CurveControlV4_ErrorMmQ8(uint8_t filtered_mask);

#endif
