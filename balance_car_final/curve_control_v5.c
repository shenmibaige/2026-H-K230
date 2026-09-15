#include "curve_control_v5.h"

#include <stddef.h>

static void CurveControlV5_StopForFinish(
    CurveControlV5 *control,
    CurveFinishTrigger trigger);

static uint8_t CurveControlV5_PopCount(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 1U));
        value = (uint8_t)(value >> 1U);
    }
    return count;
}

static int16_t CurveControlV5_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static int16_t CurveControlV5_Clamp(
    int16_t value,
    int16_t minimum,
    int16_t maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static int16_t CurveControlV5_Slew(
    int16_t current,
    int16_t target)
{
    if (target > current)
    {
        int16_t difference = (int16_t)(target - current);
        if (difference > CURVE_PWM_RISE_PER_5MS)
        {
            return (int16_t)(current + CURVE_PWM_RISE_PER_5MS);
        }
    }
    else if (target < current)
    {
        int16_t difference = (int16_t)(current - target);
        if (difference > CURVE_PWM_FALL_PER_5MS)
        {
            return (int16_t)(current - CURVE_PWM_FALL_PER_5MS);
        }
    }
    return target;
}

static uint8_t CurveControlV5_IsRunState(CurveRaceState state)
{
    return ((state == CURVE_STATE_STARTING) ||
            (state == CURVE_STATE_RUNNING) ||
            (state == CURVE_STATE_FINISH_ARMED) ||
            (state == CURVE_STATE_STOP_DELAY))
               ? 1U
               : 0U;
}

static uint8_t CurveControlV5_IsFinishCandidate(uint8_t filtered_mask)
{
    return ((filtered_mask & CURVE_FINISH_BCD_MASK) ==
            CURVE_FINISH_BCD_MASK)
               ? 1U
               : 0U;
}

static void CurveControlV5_EnterStopDelay(
    CurveControlV5 *control,
    CurveFinishTrigger trigger)
{
    control->output.finish_trigger = trigger;
    control->output.stop_delay_start_ms =
        control->output.elapsed_ms;
    control->output.state = CURVE_STATE_STOP_DELAY;
}

static uint8_t CurveControlV5_SelectApproachStage(uint32_t elapsed_ms)
{
    if (elapsed_ms >= CURVE_FINISH_APPROACH_STAGE2_MS)
    {
        return 2U;
    }
    if (elapsed_ms >= CURVE_FINISH_APPROACH_STAGE1_MS)
    {
        return 1U;
    }
    return 0U;
}

static int16_t CurveControlV5_SelectBasePwm(
    int16_t error_mm_q8,
    uint8_t approach_stage)
{
    int16_t magnitude = CurveControlV5_Abs16(error_mm_q8);
    int16_t straight_pwm;
    int16_t mid_pwm;
    int16_t outer_pwm;

    if (approach_stage >= 2U)
    {
        straight_pwm = CURVE_APPROACH2_STRAIGHT_PWM;
        mid_pwm = CURVE_APPROACH2_MID_PWM;
        outer_pwm = CURVE_APPROACH2_OUTER_PWM;
    }
    else if (approach_stage == 1U)
    {
        straight_pwm = CURVE_APPROACH1_STRAIGHT_PWM;
        mid_pwm = CURVE_APPROACH1_MID_PWM;
        outer_pwm = CURVE_APPROACH1_OUTER_PWM;
    }
    else
    {
        straight_pwm = CURVE_BASE_STRAIGHT_PWM;
        mid_pwm = CURVE_BASE_MID_PWM;
        outer_pwm = CURVE_BASE_OUTER_PWM;
    }

    if (magnitude < CURVE_ERROR_NEAR_Q8)
    {
        return straight_pwm;
    }
    if (magnitude <= CURVE_ERROR_MID_Q8)
    {
        return mid_pwm;
    }
    return outer_pwm;
}

static void CurveControlV5_ResetDynamicControl(CurveControlV5 *control)
{
    Curve_PID_Reset(&control->line_pid);
    control->line_pid.Kp = CURVE_LINE_PID_KP;
    control->line_pid.Ki = CURVE_LINE_PID_KI;
    control->line_pid.Kd = CURVE_LINE_PID_KD;
    control->line_pid.OutMax = CURVE_LINE_PID_OUT_LIMIT;
    control->line_pid.OutMin = -CURVE_LINE_PID_OUT_LIMIT;
    control->line_pid.ErrorIntMax =
        CURVE_LINE_PID_I_OUT_LIMIT / CURVE_LINE_PID_KI;
    control->line_pid.ErrorIntMin =
        -CURVE_LINE_PID_I_OUT_LIMIT / CURVE_LINE_PID_KI;
    control->line_pid.OutRiseMax = 0.0f;
    control->line_pid.OutFallMax = 0.0f;

    control->output.error_mm_q8 = 0;
    control->output.turn_pwm = 0;
    control->output.left_target_pwm = 0;
    control->output.right_target_pwm = 0;
    control->output.left_pwm = 0;
    control->output.right_pwm = 0;
    control->output.gap_hold = 0U;
}

void CurveControlV5_Init(CurveControlV5 *control)
{
    uint8_t index;

    if (control == NULL)
    {
        return;
    }

    for (index = 0U; index < CURVE_SENSOR_COUNT; index++)
    {
        control->history[index] = 0U;
    }
    control->sample_count = 0U;
    control->clear_count = 0U;
    control->output.state = CURVE_STATE_WAIT;
    control->output.fault = CURVE_FAULT_NONE;
    control->output.elapsed_ms = 0U;
    control->output.stop_delay_start_ms = 0U;
    control->output.raw_mask = 0U;
    control->output.filtered_mask = 0U;
    control->output.start_line_cleared = 0U;
    control->output.finish_confirm_count = 0U;
    control->output.raw_finish_count = 0U;
    control->output.approach_stage = 0U;
    control->output.finish_trigger =
        CURVE_FINISH_TRIGGER_NONE;
    CurveControlV5_ResetDynamicControl(control);
}

uint8_t CurveControlV5_Start(CurveControlV5 *control)
{
    if ((control == NULL) ||
        (control->sample_count < CURVE_HISTORY_SAMPLES))
    {
        return 0U;
    }

    if ((control->output.state != CURVE_STATE_WAIT) &&
        (control->output.state != CURVE_STATE_STOPPED) &&
        (control->output.state != CURVE_STATE_FAULT))
    {
        return 0U;
    }

    control->clear_count = 0U;
    control->output.state = CURVE_STATE_STARTING;
    control->output.fault = CURVE_FAULT_NONE;
    control->output.elapsed_ms = 0U;
    control->output.stop_delay_start_ms = 0U;
    control->output.start_line_cleared = 0U;
    control->output.finish_confirm_count = 0U;
    control->output.raw_finish_count = 0U;
    control->output.approach_stage = 0U;
    control->output.finish_trigger =
        CURVE_FINISH_TRIGGER_NONE;
    CurveControlV5_ResetDynamicControl(control);
    return 1U;
}

void CurveControlV5_PushSample1ms(
    CurveControlV5 *control,
    uint8_t raw_mask)
{
    uint8_t index;

    if (control == NULL)
    {
        return;
    }

    raw_mask = (uint8_t)(raw_mask & CURVE_SENSOR_MASK);
    control->output.raw_mask = raw_mask;
    for (index = 0U; index < CURVE_SENSOR_COUNT; index++)
    {
        uint8_t bit =
            ((raw_mask & (uint8_t)(1U << index)) != 0U) ? 1U : 0U;
        control->history[index] =
            (uint8_t)(((control->history[index] << 1U) | bit) &
                      CURVE_HISTORY_MASK);
    }

    if (control->sample_count < CURVE_HISTORY_SAMPLES)
    {
        control->sample_count++;
    }

    if (CurveControlV5_IsRunState(control->output.state) != 0U)
    {
        control->output.elapsed_ms++;
    }

    /*
     * Fast finish path: only the armed final lap may use raw B/C/D.
     * Two consecutive 1 ms samples reject a single-sample glitch while
     * removing the majority-filter plus three-control-cycle latency.
     */
    if (control->output.state == CURVE_STATE_FINISH_ARMED)
    {
        if (CurveControlV5_IsFinishCandidate(raw_mask) != 0U)
        {
            if (control->output.raw_finish_count < 255U)
            {
                control->output.raw_finish_count++;
            }
        }
        else
        {
            control->output.raw_finish_count = 0U;
        }

        if (control->output.raw_finish_count >=
            CURVE_FINISH_RAW_CONFIRM_SAMPLES)
        {
            CurveControlV5_EnterStopDelay(
                control,
                CURVE_FINISH_TRIGGER_RAW_FAST);
        }
    }
    else
    {
        control->output.raw_finish_count = 0U;
    }
}

uint8_t CurveControlV5_FilteredMask(const CurveControlV5 *control)
{
    uint8_t mask = 0U;
    uint8_t threshold;
    uint8_t index;

    if ((control == NULL) || (control->sample_count == 0U))
    {
        return 0U;
    }

    threshold = (uint8_t)((control->sample_count / 2U) + 1U);
    for (index = 0U; index < CURVE_SENSOR_COUNT; index++)
    {
        if (CurveControlV5_PopCount(control->history[index]) >= threshold)
        {
            mask = (uint8_t)(mask | (uint8_t)(1U << index));
        }
    }
    return mask;
}

int16_t CurveControlV5_ErrorMmQ8(uint8_t filtered_mask)
{
    static const int8_t positions_mm[CURVE_SENSOR_COUNT] = {
        CURVE_SENSOR_A_MM,
        CURVE_SENSOR_B_MM,
        CURVE_SENSOR_C_MM,
        CURVE_SENSOR_D_MM,
        CURVE_SENSOR_E_MM};
    int16_t sum_mm = 0;
    uint8_t count = 0U;
    uint8_t index;

    filtered_mask = (uint8_t)(filtered_mask & CURVE_SENSOR_MASK);
    for (index = 0U; index < CURVE_SENSOR_COUNT; index++)
    {
        if ((filtered_mask & (uint8_t)(1U << index)) != 0U)
        {
            sum_mm = (int16_t)(sum_mm + positions_mm[index]);
            count++;
        }
    }

    if (count == 0U)
    {
        return 0;
    }
    return (int16_t)(
        ((int32_t)sum_mm * CURVE_Q8_SCALE) / (int32_t)count);
}

static void CurveControlV5_RunLinePid(CurveControlV5 *control)
{
    int16_t base_pwm;
    int16_t turn_pwm;
    int16_t left_target;
    int16_t right_target;

    if (control->output.filtered_mask == 0U)
    {
        /*
         * An 18 mm line can fall between adjacent digital sensors.  Preserve
         * the complete previous control state until a sensor sees the line.
         */
        control->output.gap_hold = 1U;
        return;
    }

    control->output.gap_hold = 0U;
    control->output.error_mm_q8 =
        CurveControlV5_ErrorMmQ8(control->output.filtered_mask);
    control->output.approach_stage =
        CurveControlV5_SelectApproachStage(
            control->output.elapsed_ms);
    base_pwm =
        CurveControlV5_SelectBasePwm(
            control->output.error_mm_q8,
            control->output.approach_stage);

    control->line_pid.Target = 0.0f;
    control->line_pid.Actual =
        (float)control->output.error_mm_q8 / (float)CURVE_Q8_SCALE;
    Curve_PID_Update(&control->line_pid);
    turn_pwm = (int16_t)control->line_pid.Out;
    turn_pwm = CurveControlV5_Clamp(
        turn_pwm,
        (int16_t)(-CURVE_LINE_PID_OUT_LIMIT),
        (int16_t)CURVE_LINE_PID_OUT_LIMIT);

    left_target = (int16_t)(base_pwm - turn_pwm);
    right_target = (int16_t)(base_pwm + turn_pwm);

    if (control->output.elapsed_ms < CURVE_START_RAMP_MS)
    {
        left_target = (int16_t)(
            ((int32_t)left_target *
             (int32_t)control->output.elapsed_ms) /
            (int32_t)CURVE_START_RAMP_MS);
        right_target = (int16_t)(
            ((int32_t)right_target *
             (int32_t)control->output.elapsed_ms) /
            (int32_t)CURVE_START_RAMP_MS);
    }

    left_target =
        CurveControlV5_Clamp(left_target, 0, CURVE_PWM_LIMIT);
    right_target =
        CurveControlV5_Clamp(right_target, 0, CURVE_PWM_LIMIT);

    control->output.turn_pwm = turn_pwm;
    control->output.left_target_pwm = left_target;
    control->output.right_target_pwm = right_target;
    control->output.left_pwm =
        CurveControlV5_Slew(control->output.left_pwm, left_target);
    control->output.right_pwm =
        CurveControlV5_Slew(control->output.right_pwm, right_target);
}

void CurveControlV5_StopImmediate(
    CurveControlV5 *control,
    CurveControlV5Fault fault)
{
    if (control == NULL)
    {
        return;
    }

    CurveControlV5_ResetDynamicControl(control);
    control->output.finish_confirm_count = 0U;
    control->output.raw_finish_count = 0U;
    control->output.finish_trigger =
        CURVE_FINISH_TRIGGER_NONE;
    control->output.stop_delay_start_ms = 0U;
    control->output.fault = fault;
    control->output.state =
        (fault == CURVE_FAULT_NONE)
            ? CURVE_STATE_STOPPED
            : CURVE_STATE_FAULT;
}

static void CurveControlV5_StopForFinish(
    CurveControlV5 *control,
    CurveFinishTrigger trigger)
{
    CurveControlV5_StopImmediate(
        control,
        CURVE_FAULT_NONE);
    control->output.finish_trigger = trigger;

    if (trigger == CURVE_FINISH_TRIGGER_RAW_FAST)
    {
        control->output.raw_finish_count =
            CURVE_FINISH_RAW_CONFIRM_SAMPLES;
    }
    else if (
        trigger ==
        CURVE_FINISH_TRIGGER_FILTERED_FALLBACK)
    {
        control->output.finish_confirm_count =
            CURVE_FINISH_CONFIRM_CYCLES;
    }
}

CurveControlV5Output CurveControlV5_Step5ms(CurveControlV5 *control)
{
    uint8_t finish_candidate;

    if (control == NULL)
    {
        CurveControlV5Output empty = {0};
        return empty;
    }

    control->output.filtered_mask =
        CurveControlV5_FilteredMask(control);

    /* After finish is confirmed, keep tracking for 2 s before stopping. */
    if ((control->output.state == CURVE_STATE_STOP_DELAY) &&
        ((control->output.elapsed_ms -
          control->output.stop_delay_start_ms) >=
         CURVE_FINISH_STOP_DELAY_MS))
    {
        CurveControlV5_StopForFinish(
            control,
            control->output.finish_trigger);
        return control->output;
    }

    if ((CurveControlV5_IsRunState(control->output.state) != 0U) &&
        (control->output.state != CURVE_STATE_STOP_DELAY) &&
        (control->output.elapsed_ms >= CURVE_HARD_TIMEOUT_MS))
    {
        CurveControlV5_StopImmediate(
            control,
            CURVE_FAULT_HARD_TIMEOUT);
        return control->output;
    }

    if (CurveControlV5_IsRunState(control->output.state) == 0U)
    {
        control->output.left_pwm = 0;
        control->output.right_pwm = 0;
        control->output.left_target_pwm = 0;
        control->output.right_target_pwm = 0;
        return control->output;
    }

    finish_candidate =
        CurveControlV5_IsFinishCandidate(
            control->output.filtered_mask);

    if (control->output.state == CURVE_STATE_STARTING)
    {
        control->output.finish_confirm_count = 0U;
        if (finish_candidate == 0U)
        {
            if (control->clear_count < 255U)
            {
                control->clear_count++;
            }
        }
        else
        {
            control->clear_count = 0U;
        }

        if (control->clear_count >= CURVE_START_CLEAR_CYCLES)
        {
            control->output.start_line_cleared = 1U;
            control->output.state = CURVE_STATE_RUNNING;
        }
    }
    else if ((control->output.state == CURVE_STATE_RUNNING) &&
             (control->output.start_line_cleared != 0U) &&
             (control->output.elapsed_ms >=
              CURVE_FINISH_ARM_MIN_MS))
    {
        control->output.state = CURVE_STATE_FINISH_ARMED;
        control->output.finish_confirm_count = 0U;
    }

    if (control->output.state == CURVE_STATE_FINISH_ARMED)
    {
        if (finish_candidate != 0U)
        {
            if (control->output.finish_confirm_count < 255U)
            {
                control->output.finish_confirm_count++;
            }
        }
        else
        {
            control->output.finish_confirm_count = 0U;
        }

        if (control->output.finish_confirm_count >=
            CURVE_FINISH_CONFIRM_CYCLES)
        {
            CurveControlV5_EnterStopDelay(
                control,
                CURVE_FINISH_TRIGGER_FILTERED_FALLBACK);
        }
    }

    CurveControlV5_RunLinePid(control);
    return control->output;
}

CurveControlV5Output CurveControlV5_GetOutput(
    const CurveControlV5 *control)
{
    CurveControlV5Output empty = {0};

    if (control == NULL)
    {
        return empty;
    }
    return control->output;
}

const char *CurveControlV5_StateName(CurveRaceState state)
{
    switch (state)
    {
        case CURVE_STATE_WAIT:
            return "WAIT    ";
        case CURVE_STATE_STARTING:
            return "START   ";
        case CURVE_STATE_RUNNING:
            return "RUN     ";
        case CURVE_STATE_FINISH_ARMED:
            return "FIN_ARM ";
        case CURVE_STATE_STOP_DELAY:
            return "STOP2S  ";
        case CURVE_STATE_STOPPED:
            return "STOP    ";
        case CURVE_STATE_FAULT:
        default:
            return "FAULT   ";
    }
}
