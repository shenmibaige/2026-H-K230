#include "ball_controller.h"

#include <math.h>
#include <string.h>

typedef enum {
    BALL_QUIET_CONTROL_NOT_HANDLED = 0,
    BALL_QUIET_CONTROL_HANDLED,
    BALL_QUIET_CONTROL_EXIT,
    BALL_QUIET_CONTROL_SAFETY_BRAKE
} BallQuietControlResult;

static float Ball_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Ball_Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int16_t Ball_RoundInt16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint16_t Ball_RoundUInt16(float value)
{
    if (value <= 0.0f) {
        return 0u;
    }
    if (value >= 65535.0f) {
        return 65535u;
    }
    return (uint16_t)(value + 0.5f);
}

static int8_t Ball_RoundPidOutput2(float value_deg)
{
    float scaled = value_deg * 2.0f;
    if (scaled > 127.0f) {
        return 127;
    }
    if (scaled < -128.0f) {
        return -128;
    }
    return (int8_t)((scaled >= 0.0f) ? (scaled + 0.5f) :
                                            (scaled - 0.5f));
}

static uint8_t Ball_RoundWeight255(float weight)
{
    float scaled = Ball_Clamp(weight, 0.0f, 1.0f) * 255.0f;
    return (uint8_t)(scaled + 0.5f);
}

static float Ball_HardClampServo(float angle_deg)
{
    return Ball_Clamp(angle_deg,
                      BALL_SERVO_HARD_MIN_DEG,
                      BALL_SERVO_HARD_MAX_DEG);
}

static uint8_t Ball_IsTerminalBControlState(BallState state)
{
    return ((state == BALL_STATE_B_SETTLE) ||
            (state == BALL_STATE_DONE_HOLD) ||
            (state == BALL_STATE_TIMEOUT)) ? 1u : 0u;
}

static void Ball_RecordServoTransition(BallController *controller,
                                       float new_angle_deg)
{
    uint8_t old_noncenter =
        (Ball_Abs(controller->servo_command_deg -
                  BALL_SERVO_CENTER_DEG) > 0.05f) ? 1u : 0u;
    uint8_t new_noncenter =
        (Ball_Abs(new_angle_deg - BALL_SERVO_CENTER_DEG) > 0.05f) ? 1u : 0u;

    if ((old_noncenter == 0u) && (new_noncenter != 0u) &&
        (Ball_IsTerminalBControlState(controller->state) != 0u)) {
        if (controller->diagnostics.servo_event_count < 65535u) {
            controller->diagnostics.servo_event_count++;
        }
        if ((controller->quiet_event_window_active == 0u) ||
            ((uint32_t)(controller->control_now_ms -
                        controller->quiet_event_window_start_ms) >=
             BALL_QUIET_SERVO_EVENT_WINDOW_MS)) {
            controller->quiet_event_window_active = 1u;
            controller->quiet_event_window_start_ms =
                controller->control_now_ms;
            controller->quiet_event_window_count = 0u;
        }
        if (controller->quiet_event_window_count < 255u) {
            controller->quiet_event_window_count++;
        }
        if (controller->quiet_event_window_count >
            controller->diagnostics.max_servo_events_in_1s) {
            controller->diagnostics.max_servo_events_in_1s =
                controller->quiet_event_window_count;
        }
    }
}

static void Ball_SetServoCenter(BallController *controller)
{
    float angle_deg = Ball_HardClampServo(BALL_SERVO_CENTER_DEG);
    Ball_RecordServoTransition(controller, angle_deg);
    controller->servo_command_deg = angle_deg;
}

static void Ball_SetServoAutomaticOffset(BallController *controller,
                                         float offset_deg)
{
    float angle_deg = BALL_SERVO_CENTER_DEG + offset_deg;
    angle_deg = Ball_Clamp(angle_deg,
                          BALL_SERVO_AUTO_MIN_DEG,
                          BALL_SERVO_AUTO_MAX_DEG);
    angle_deg = Ball_HardClampServo(angle_deg);
    Ball_RecordServoTransition(controller, angle_deg);
    controller->servo_command_deg = angle_deg;
}

static void Ball_SetServoPhaseOffset(BallController *controller,
                                     float base_offset_deg)
{
    float trim_deg = Ball_Clamp(controller->pid_bank.fused_output_deg,
                               -BALL_PID_PHASE_TRIM_MAX_DEG,
                               BALL_PID_PHASE_TRIM_MAX_DEG);
    Ball_SetServoAutomaticOffset(controller, base_offset_deg + trim_deg);
}

static void Ball_ResetQuietRuntime(BallController *controller)
{
    BallQuietObserver_Reset(&controller->quiet_observer);
    controller->quiet_position_cm = controller->position_cm;
    controller->quiet_error_cm = BALL_TARGET_B_CM - controller->position_cm;
    controller->quiet_spread_cm = 0.0f;
    controller->quiet_anchor_cm = controller->position_cm;
    controller->quiet_position_confirm_start_ms = 0u;
    controller->quiet_bias_start_ms = 0u;
    controller->quiet_last_pulse_end_ms = 0u;
    controller->quiet_event_window_start_ms = 0u;
    controller->quiet_estimate_valid = 0u;
    controller->quiet_positive_error_count = 0u;
    controller->quiet_negative_error_count = 0u;
    controller->quiet_active = 0u;
    controller->quiet_enter_frames = 0u;
    controller->quiet_position_confirm_active = 0u;
    controller->quiet_speed_exit_frames = 0u;
    controller->quiet_hard_error_frames = 0u;
    controller->quiet_bias_active = 0u;
    controller->quiet_reacquire_wait_frames = 0u;
    controller->quiet_warmup_hold = 0u;
    controller->quiet_pulse_sequence_active = 0u;
    controller->quiet_has_last_pulse_end = 0u;
    controller->quiet_event_window_active = 0u;
    controller->quiet_event_window_count = 0u;
}

static uint8_t Ball_QuietFlags(const BallController *controller)
{
    uint8_t flags = 0u;
    if (controller->quiet_active != 0u) {
        flags |= 0x01u;
    }
    if (controller->quiet_estimate_valid != 0u) {
        flags |= 0x02u;
    }
    if (controller->quiet_bias_active != 0u) {
        flags |= 0x04u;
    }
    if (controller->quiet_position_confirm_active != 0u) {
        flags |= 0x08u;
    }
    if (controller->quiet_pulse_sequence_active != 0u) {
        flags |= 0x10u;
    }
    if ((controller->quiet_has_last_pulse_end != 0u) &&
        ((uint32_t)(controller->control_now_ms -
                    controller->quiet_last_pulse_end_ms) <
         BALL_QUIET_PULSE_REARM_MS)) {
        flags |= 0x20u;
    }
    if (controller->quiet_reacquire_wait_frames != 0u) {
        flags |= 0x40u;
    }
    if (controller->quiet_warmup_hold != 0u) {
        flags |= 0x80u;
    }
    return flags;
}

static void Ball_ResetPulse(BallController *controller)
{
    controller->pulse_phase = BALL_PULSE_IDLE;
    controller->settle_mode = BALL_SETTLE_IDLE;
    controller->pulse_phase_start_ms = 0u;
    controller->pulse_start_cm = 0.0f;
    controller->pulse_direction = 0;
    controller->pulse_level = 0u;
    controller->pulse_no_move_count = 0u;
    controller->settle_hold_active = 0u;
    controller->settle_hold_start_ms = 0u;
    controller->settle_moving = 0u;
    controller->settle_still_frames = 0u;
    controller->settle_fast_sign = 0;
    controller->settle_fast_sign_frames = 0u;
    controller->settle_pulse_ready_frames = 0u;
    controller->b_settle_brake_guard_active = 0u;
}

static void Ball_ResetMotionHistory(BallController *controller)
{
    BallEstimator_Reset(&controller->estimator);
    BallFastEstimator_Reset(&controller->fast_estimator);
    BallPidBank_ResetDynamics(&controller->pid_bank);
    controller->velocity_cm_s = 0.0f;
    controller->velocity_fast_cm_s = 0.0f;
    controller->velocity_guard_cm_s = 0.0f;
    controller->velocity_fast_valid = 0u;
    controller->predicted_position_cm = controller->position_cm;
    controller->x_guard_cm = controller->position_cm;
    controller->d_remaining_cm = controller->position_cm - BALL_TARGET_B_CM;
    controller->d_stop_cm = BALL_B_STOP_MARGIN_CM;
    controller->v_limit_cm_s = BALL_B_ENVELOPE_MAX_CM_S;
    controller->adjacent_velocity_cm_s = 0.0f;
    controller->has_last_position = 0u;
    controller->has_adjacent_velocity = 0u;
    controller->drive_motion_detected = 0u;
    controller->a_pulse_active = 0u;
    controller->a_overrun_recovery = 0u;
    controller->a_peak_cm = controller->position_cm;
    controller->a_peak_valid = 0u;
    controller->a_reverse_confirm_frames = 0u;
    controller->b_min_cm = controller->position_cm;
    controller->b_last_side = 0;
    controller->b_side_valid = 0u;
    controller->b_rebound_confirm_frames = 0u;
    controller->b_brake_release_confirm_frames = 0u;
    controller->b_first_overshoot_latched = 0u;
    controller->b_approach_confirm_frames = 0u;
    controller->b_approach_candidate_reason = BALL_APPROACH_REASON_NONE;
    controller->b_approach_command = BALL_APPROACH_COMMAND_COAST;
    controller->b_approach_command_valid = 0u;
    controller->b_approach_command_start_ms = 0u;
    controller->last_transition_reason = BALL_TRANSITION_NONE;
    Ball_ResetPulse(controller);
    Ball_ResetQuietRuntime(controller);
}

static void Ball_ResetCenterWindow(BallController *controller)
{
    controller->center_count = 0u;
    controller->center_ready = 0u;
    controller->center_median_dx_px = 0;
    memset(controller->center_dx_px, 0, sizeof(controller->center_dx_px));
    memset(controller->center_time_ms, 0, sizeof(controller->center_time_ms));
}

static void Ball_ClearLog(BallController *controller)
{
    controller->log_head = 0u;
    controller->log_count = 0u;
    controller->log_frozen = 0u;
    controller->has_log_time = 0u;
    controller->last_logged_state = BALL_STATE_IDLE;
    controller->last_log_ms = 0u;
    memset(controller->logs, 0, sizeof(controller->logs));
}

static void Ball_Log(BallController *controller,
                     uint32_t now_ms,
                     uint16_t data_age_ms)
{
    uint8_t i;

    if (controller->log_frozen != 0u) {
        return;
    }
    if ((controller->has_log_time != 0u) &&
        (data_age_ms == 0u) &&
        (controller->state == controller->last_logged_state) &&
        ((uint32_t)(now_ms - controller->last_log_ms) <
         BALL_LOG_MIN_PERIOD_MS)) {
        return;
    }
    BallLogRecord *record = &controller->logs[controller->log_head];
    record->time_ms = now_ms;
    record->dx_px = controller->last_dx_px;
    record->x100 = Ball_RoundInt16(controller->position_cm * 100.0f);
    record->v100 = Ball_RoundInt16(controller->velocity_cm_s * 100.0f);
    record->xpred100 = Ball_RoundInt16(controller->predicted_position_cm * 100.0f);
    record->servo10 = Ball_RoundUInt16(controller->servo_command_deg * 10.0f);
    record->data_age = data_age_ms;
    record->result_flags = controller->result_flags;
    record->state = (uint8_t)controller->state;
    record->detail = (uint8_t)(((uint8_t)controller->settle_mode & 0x0Fu) |
        (((uint8_t)controller->last_transition_reason & 0x0Fu) << 4));
    for (i = 0u; i < BALL_PID_COUNT; ++i) {
        record->pid_output2[i] =
            Ball_RoundPidOutput2(controller->pid_bank.lane[i].output_deg);
        record->pid_weight255[i] =
            Ball_RoundWeight255(controller->pid_bank.lane[i].weight);
    }
    record->pid_fused10 =
        Ball_RoundInt16(controller->pid_bank.fused_output_deg * 10.0f);
    record->pid_update_mask = controller->pid_bank.update_mask;
    record->pid_dominant = controller->pid_bank.dominant_index;
    record->vfast100 = Ball_RoundInt16(controller->velocity_fast_cm_s * 100.0f);
    record->vguard100 = Ball_RoundInt16(controller->velocity_guard_cm_s * 100.0f);
    record->xguard100 = Ball_RoundInt16(controller->x_guard_cm * 100.0f);
    record->drem100 = Ball_RoundInt16(controller->d_remaining_cm * 100.0f);
    record->dstop100 = Ball_RoundInt16(controller->d_stop_cm * 100.0f);
    record->vlimit100 = Ball_RoundInt16(controller->v_limit_cm_s * 100.0f);
    record->approach_reason = controller->diagnostics.b_approach_reason;
    record->brake_reason = controller->diagnostics.b_brake_reason;
    record->vB_enter100 = Ball_RoundInt16(
        controller->diagnostics.b_band_entry_velocity_cm_s * 100.0f);
    record->quiet_error100 =
        Ball_RoundInt16(controller->quiet_error_cm * 100.0f);
    record->quiet_flags = Ball_QuietFlags(controller);
    record->servo_event_count =
        (controller->diagnostics.servo_event_count > 255u) ?
            255u : (uint8_t)controller->diagnostics.servo_event_count;

    controller->log_head = (uint16_t)((controller->log_head + 1u) % BALL_LOG_CAPACITY);
    if (controller->log_count < BALL_LOG_CAPACITY) {
        controller->log_count++;
    }
    controller->has_log_time = 1u;
    controller->last_logged_state = controller->state;
    controller->last_log_ms = now_ms;
}

static uint32_t Ball_TaskElapsed(const BallController *controller,
                                 uint32_t now_ms)
{
    if (controller->task_started == 0u) {
        return 0u;
    }
    return (uint32_t)(now_ms - controller->task_start_ms);
}

static void Ball_UpdateBudgetFlags(BallController *controller,
                                   uint32_t elapsed_ms)
{
    if ((controller->diagnostics.t_a_ms == 0u) &&
        (elapsed_ms > BALL_BUDGET_A_TURN_MS)) {
        controller->result_flags |= BALL_RESULT_A_LATE;
    }
    if ((controller->diagnostics.t_b_approach_ms == 0u) &&
        (elapsed_ms > BALL_BUDGET_B_APPROACH_MS)) {
        controller->result_flags |= BALL_RESULT_B_APPROACH_LATE;
    }
    if ((controller->diagnostics.t_b_enter_ms == 0u) &&
        (elapsed_ms > BALL_BUDGET_B_BRAKE_MS)) {
        controller->result_flags |= BALL_RESULT_B_BRAKE_LATE;
    }
    if ((controller->diagnostics.t_b_band_ms == 0u) &&
        (elapsed_ms > BALL_BUDGET_B_BAND_MS)) {
        controller->result_flags |= BALL_RESULT_B_BAND_LATE;
    }
    if ((controller->diagnostics.t_done_ms == 0u) &&
        (elapsed_ms > BALL_BUDGET_DONE_MS)) {
        controller->result_flags |= BALL_RESULT_DONE_LATE;
    }
}

static void Ball_CenterWindowPush(BallController *controller,
                                  int16_t dx_px,
                                  uint32_t now_ms)
{
    if (controller->center_count > 0u) {
        uint32_t dt_ms = (uint32_t)(now_ms -
            controller->center_time_ms[controller->center_count - 1u]);
        if ((dt_ms < BALL_ESTIMATOR_MIN_DT_MS) ||
            (dt_ms > BALL_ESTIMATOR_MAX_DT_MS)) {
            Ball_ResetCenterWindow(controller);
        }
    }

    if (controller->center_count >= BALL_CENTER_SAMPLE_COUNT) {
        uint8_t i;
        for (i = 1u; i < BALL_CENTER_SAMPLE_COUNT; ++i) {
            controller->center_dx_px[i - 1u] = controller->center_dx_px[i];
            controller->center_time_ms[i - 1u] = controller->center_time_ms[i];
        }
        controller->center_count = BALL_CENTER_SAMPLE_COUNT - 1u;
    }

    controller->center_dx_px[controller->center_count] = dx_px;
    controller->center_time_ms[controller->center_count] = now_ms;
    controller->center_count++;
}

static uint8_t Ball_CenterWindowReady(const BallController *controller,
                                      int16_t *median_dx_px)
{
    int16_t sorted[BALL_CENTER_SAMPLE_COUNT];
    int16_t median;
    float mean_t_s = 0.0f;
    float mean_x_cm = 0.0f;
    float numerator = 0.0f;
    float denominator = 0.0f;
    float velocity_cm_s;
    uint32_t origin_ms;
    uint8_t i;

    if (controller->center_count < BALL_CENTER_SAMPLE_COUNT) {
        return 0u;
    }
    if ((uint32_t)(controller->center_time_ms[BALL_CENTER_SAMPLE_COUNT - 1u] -
                   controller->center_time_ms[0]) < BALL_CENTER_WINDOW_MIN_MS) {
        return 0u;
    }

    memcpy(sorted, controller->center_dx_px, sizeof(sorted));
    for (i = 1u; i < BALL_CENTER_SAMPLE_COUNT; ++i) {
        int16_t item = sorted[i];
        uint8_t j = i;
        while ((j > 0u) && (sorted[j - 1u] > item)) {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = item;
    }
    median = sorted[BALL_CENTER_SAMPLE_COUNT / 2u];

    if (Ball_Abs((float)median - (float)BALL_IMAGE_CENTER_DX_PX) >
        (float)BALL_CENTER_MEDIAN_LIMIT_PX) {
        return 0u;
    }
    for (i = 0u; i < BALL_CENTER_SAMPLE_COUNT; ++i) {
        float excursion_cm = Ball_Abs((float)(controller->center_dx_px[i] - median)) /
                             BALL_PX_PER_CM;
        if (excursion_cm > BALL_CENTER_MAX_SWING_CM) {
            return 0u;
        }
    }

    origin_ms = controller->center_time_ms[0];
    for (i = 0u; i < BALL_CENTER_SAMPLE_COUNT; ++i) {
        float t_s = (float)(uint32_t)(controller->center_time_ms[i] - origin_ms) * 0.001f;
        float x_cm = -(float)(controller->center_dx_px[i] - median) / BALL_PX_PER_CM;
        mean_t_s += t_s;
        mean_x_cm += x_cm;
    }
    mean_t_s /= (float)BALL_CENTER_SAMPLE_COUNT;
    mean_x_cm /= (float)BALL_CENTER_SAMPLE_COUNT;
    for (i = 0u; i < BALL_CENTER_SAMPLE_COUNT; ++i) {
        float t_s = (float)(uint32_t)(controller->center_time_ms[i] - origin_ms) * 0.001f;
        float x_cm = -(float)(controller->center_dx_px[i] - median) / BALL_PX_PER_CM;
        float dt_s = t_s - mean_t_s;
        numerator += dt_s * (x_cm - mean_x_cm);
        denominator += dt_s * dt_s;
    }
    velocity_cm_s = (denominator > 0.0000001f) ? (numerator / denominator) : 0.0f;
    if (Ball_Abs(velocity_cm_s) > BALL_CENTER_MAX_SPEED_CM_S) {
        return 0u;
    }

    *median_dx_px = median;
    return 1u;
}

static void Ball_EnterDriveB(BallController *controller, uint32_t now_ms)
{
    controller->state = BALL_STATE_DRIVE_B;
    controller->stage_start_ms = now_ms;
    controller->drive_reference_cm = controller->position_cm;
    controller->drive_motion_detected = 0u;
    controller->a_pulse_active = 0u;
    controller->a_overrun_recovery = 0u;
    BallPidBank_ResetIntegrals(&controller->pid_bank);
    Ball_ResetPulse(controller);
    controller->b_min_cm = controller->position_cm;
    controller->diagnostics.b_min_cm = controller->position_cm;
    controller->b_last_side = (controller->position_cm >= BALL_TARGET_B_CM) ? 1 : -1;
    controller->b_side_valid = 1u;
    controller->b_rebound_confirm_frames = 0u;
    controller->b_brake_release_confirm_frames = 0u;
    controller->b_first_overshoot_latched = 0u;
    controller->b_approach_confirm_frames = 0u;
    controller->b_approach_candidate_reason = BALL_APPROACH_REASON_NONE;
    controller->b_approach_command = BALL_APPROACH_COMMAND_COAST;
    controller->b_approach_command_valid = 0u;
    controller->velocity_fast_cm_s = 0.0f;
    controller->velocity_fast_valid = 0u;
    BallFastEstimator_Seed(&controller->fast_estimator,
                           controller->position_cm,
                           now_ms);
    /* Do not reuse the final A-target PID sign on the target-switch frame.
     * Issue the proven -30 degree B drive immediately; the six B-target lanes
     * resume normalized trimming on the next measurement. */
    Ball_SetServoAutomaticOffset(controller, BALL_DRIVE_B_OFFSET_DEG);
}

static void Ball_UpdateBGuardMetrics(BallController *controller)
{
    float v_guard_cm_s = (controller->velocity_cm_s < 0.0f) ?
        controller->velocity_cm_s : 0.0f;
    float v_in_cm_s;
    float envelope_distance_cm;

    if ((controller->velocity_fast_valid != 0u) &&
        (controller->velocity_fast_cm_s < v_guard_cm_s)) {
        v_guard_cm_s = controller->velocity_fast_cm_s;
    }
    controller->velocity_guard_cm_s = v_guard_cm_s;
    controller->x_guard_cm = controller->position_cm +
        BALL_B_GUARD_POSITION_LEAD_S * v_guard_cm_s;
    v_in_cm_s = -v_guard_cm_s;
    controller->d_remaining_cm = controller->x_guard_cm - BALL_TARGET_B_CM;
    controller->d_stop_cm = BALL_B_STOP_DELAY_S * v_in_cm_s +
        (v_in_cm_s * v_in_cm_s) / (2.0f * BALL_B_STOP_DECEL_CM_S2) +
        BALL_B_STOP_MARGIN_CM;
    envelope_distance_cm = controller->d_remaining_cm -
        BALL_B_ENVELOPE_MARGIN_CM;
    if (envelope_distance_cm < 0.0f) {
        envelope_distance_cm = 0.0f;
    }
    controller->v_limit_cm_s = sqrtf(2.0f *
        BALL_B_ENVELOPE_DECEL_CM_S2 * envelope_distance_cm);
    controller->v_limit_cm_s = Ball_Clamp(controller->v_limit_cm_s,
        BALL_B_ENVELOPE_MIN_CM_S, BALL_B_ENVELOPE_MAX_CM_S);
}

static BallApproachReason Ball_GetApproachReason(
    const BallController *controller)
{
    if (controller->d_remaining_cm <=
        (controller->d_stop_cm + BALL_B_APPROACH_MARGIN_CM)) {
        return BALL_APPROACH_REASON_STOP_MARGIN;
    }
    if (-controller->velocity_guard_cm_s >=
        BALL_B_APPROACH_HIGH_SPEED_CM_S) {
        return BALL_APPROACH_REASON_HIGH_SPEED;
    }
    if ((controller->position_cm + BALL_B_APPROACH_PHASE_LEAD_S *
         controller->velocity_guard_cm_s) <= BALL_BRAKE_B_TRIGGER_CM) {
        return BALL_APPROACH_REASON_PHASE;
    }
    return BALL_APPROACH_REASON_NONE;
}

static BallBrakeReason Ball_GetBrakeReason(const BallController *controller)
{
    float v_in_cm_s = -controller->velocity_guard_cm_s;

    if (controller->d_remaining_cm <= controller->d_stop_cm) {
        return BALL_BRAKE_REASON_STOP_DISTANCE;
    }
    if ((controller->position_cm + BALL_B_APPROACH_PHASE_LEAD_S *
         controller->velocity_guard_cm_s) <= BALL_BRAKE_B_TRIGGER_CM) {
        return BALL_BRAKE_REASON_PHASE;
    }
    if ((controller->x_guard_cm <= BALL_B_HARD_BRAKE_X_GUARD_CM) &&
        (v_in_cm_s >= BALL_B_HARD_BRAKE_MIN_SPEED_CM_S)) {
        return BALL_BRAKE_REASON_HARD_GUARD;
    }
    return BALL_BRAKE_REASON_NONE;
}

static void Ball_EnterBApproach(BallController *controller,
                                uint32_t now_ms,
                                BallApproachReason reason)
{
    controller->state = BALL_STATE_B_APPROACH;
    controller->stage_start_ms = now_ms;
    controller->diagnostics.t_b_approach_ms = Ball_TaskElapsed(controller,
                                                               now_ms);
    controller->diagnostics.b_approach_reason = (uint8_t)reason;
    controller->b_approach_command = BALL_APPROACH_COMMAND_COAST;
    controller->b_approach_command_valid = 0u;
    controller->b_approach_command_start_ms = now_ms;
    Ball_SetServoCenter(controller);
}

static void Ball_EnterBrakeB(BallController *controller,
                             uint32_t now_ms,
                             BallBrakeReason reason)
{
    uint32_t elapsed_ms = Ball_TaskElapsed(controller, now_ms);

    controller->state = BALL_STATE_BRAKE_B;
    controller->stage_start_ms = now_ms;
    controller->diagnostics.t_b_brake_ms = elapsed_ms;
    controller->diagnostics.t_b_enter_ms = elapsed_ms;
    controller->diagnostics.b_brake_start_cm = controller->position_cm;
    controller->diagnostics.b_brake_reason = (uint8_t)reason;
    controller->b_brake_release_confirm_frames = 0u;
    controller->b_rebound_confirm_frames = 0u;
    /* Hard brake is deliberately independent of the fused PID trim. */
    Ball_SetServoAutomaticOffset(controller, BALL_BRAKE_B_OFFSET_DEG);
}

static void Ball_OutputApproachCommand(BallController *controller,
                                       BallApproachCommand command)
{
    if (command == BALL_APPROACH_COMMAND_PREBRAKE) {
        Ball_SetServoAutomaticOffset(controller,
            BALL_B_APPROACH_PREBRAKE_OFFSET_DEG);
    } else if (command == BALL_APPROACH_COMMAND_REDRIVE) {
        Ball_SetServoAutomaticOffset(controller,
            BALL_B_APPROACH_REDRIVE_OFFSET_DEG);
    } else {
        Ball_SetServoCenter(controller);
    }
}

static void Ball_ControlBApproach(BallController *controller,
                                  uint32_t now_ms)
{
    BallBrakeReason brake_reason = Ball_GetBrakeReason(controller);
    BallApproachCommand desired = BALL_APPROACH_COMMAND_COAST;
    float v_in_cm_s = -controller->velocity_guard_cm_s;

    /* A hard-brake predicate always overrides the 40 ms command latch. */
    if (brake_reason != BALL_BRAKE_REASON_NONE) {
        Ball_EnterBrakeB(controller, now_ms, brake_reason);
        return;
    }

    if (v_in_cm_s > (controller->v_limit_cm_s +
                     BALL_B_APPROACH_ENVELOPE_BAND_CM_S)) {
        desired = BALL_APPROACH_COMMAND_PREBRAKE;
    } else if ((v_in_cm_s < (controller->v_limit_cm_s -
                             BALL_B_APPROACH_REDRIVE_MARGIN_CM_S)) &&
               (controller->d_remaining_cm >
                BALL_B_APPROACH_REDRIVE_MIN_DISTANCE_CM)) {
        desired = BALL_APPROACH_COMMAND_REDRIVE;
    }

    if ((controller->b_approach_command_valid != 0u) &&
        ((uint32_t)(now_ms - controller->b_approach_command_start_ms) <
         BALL_B_APPROACH_COMMAND_HOLD_MS)) {
        Ball_OutputApproachCommand(controller,
            (BallApproachCommand)controller->b_approach_command);
        return;
    }

    if (desired == BALL_APPROACH_COMMAND_COAST) {
        controller->b_approach_command_valid = 0u;
        controller->b_approach_command = BALL_APPROACH_COMMAND_COAST;
    } else if ((controller->b_approach_command_valid == 0u) ||
               (controller->b_approach_command != (uint8_t)desired)) {
        controller->b_approach_command_valid = 1u;
        controller->b_approach_command = (uint8_t)desired;
        controller->b_approach_command_start_ms = now_ms;
    }
    Ball_OutputApproachCommand(controller, desired);
}

static void Ball_StartAEarlyPulse(BallController *controller, uint32_t now_ms)
{
    controller->state = BALL_STATE_A_CAPTURE;
    controller->stage_start_ms = now_ms;
    controller->a_pulse_active = 1u;
    controller->a_pulse_start_ms = now_ms;
    Ball_SetServoAutomaticOffset(controller, BALL_A_EARLY_PULSE_OFFSET_DEG);
}

static void Ball_HandleATurn(BallController *controller,
                             float turn_cm,
                             uint32_t now_ms,
                             BallTransitionReason reason)
{
    uint32_t elapsed_ms = Ball_TaskElapsed(controller, now_ms);

    controller->diagnostics.a_turn_cm = turn_cm;
    controller->diagnostics.a_transition_reason = (uint8_t)reason;
    controller->last_transition_reason = reason;
    if (turn_cm < BALL_A_TURN_MIN_CM) {
        controller->result_flags |= BALL_RESULT_A_RANGE;
        Ball_StartAEarlyPulse(controller, now_ms);
        return;
    }
    if (turn_cm > BALL_A_TURN_MAX_CM) {
        controller->result_flags |= BALL_RESULT_A_RANGE;
        if (controller->diagnostics.t_a_ms == 0u) {
            controller->diagnostics.t_a_ms = elapsed_ms;
        }
        controller->a_overrun_recovery = 1u;
        controller->state = BALL_STATE_A_CAPTURE;
        Ball_SetServoPhaseOffset(controller, BALL_BRAKE_A_OFFSET_DEG);
        return;
    }

    controller->result_flags &= (uint16_t)~BALL_RESULT_A_RANGE;
    controller->diagnostics.t_a_ms = elapsed_ms;
    Ball_EnterDriveB(controller, now_ms);
}

static void Ball_UpdateAPeak(BallController *controller)
{
    if ((controller->a_peak_valid == 0u) ||
        (controller->position_cm > controller->a_peak_cm)) {
        controller->a_peak_cm = controller->position_cm;
        controller->a_peak_valid = 1u;
        controller->a_reverse_confirm_frames = 0u;
    }
}

static uint8_t Ball_AReverseFallbackReady(BallController *controller)
{
    if ((controller->a_peak_valid != 0u) &&
        ((controller->a_peak_cm - controller->position_cm) >=
         BALL_A_REVERSE_DROP_CM) &&
        (controller->velocity_cm_s <= -BALL_A_REVERSE_SPEED_CM_S)) {
        if (controller->a_reverse_confirm_frames < 255u) {
            controller->a_reverse_confirm_frames++;
        }
    } else {
        controller->a_reverse_confirm_frames = 0u;
    }

    return (controller->a_reverse_confirm_frames >=
            BALL_A_REVERSE_CONFIRM_FRAMES) ? 1u : 0u;
}

static int8_t Ball_BSide(float position_cm, int8_t previous_side)
{
    if (position_cm > (BALL_TARGET_B_CM + BALL_B_CROSS_HYST_CM)) {
        return 1;
    }
    if (position_cm < (BALL_TARGET_B_CM - BALL_B_CROSS_HYST_CM)) {
        return -1;
    }
    return previous_side;
}

static void Ball_LatchFirstBOvershoot(BallController *controller)
{
    float overshoot_cm;
    if (controller->b_first_overshoot_latched != 0u) {
        return;
    }
    overshoot_cm = BALL_TARGET_B_CM - controller->b_min_cm;
    if (overshoot_cm < 0.0f) {
        overshoot_cm = 0.0f;
    }
    controller->diagnostics.b_first_overshoot_cm = overshoot_cm;
    controller->b_first_overshoot_latched = 1u;
}

static void Ball_UpdateBDiagnostics(BallController *controller)
{
    int8_t side;

    if (controller->position_cm < controller->b_min_cm) {
        controller->b_min_cm = controller->position_cm;
        controller->diagnostics.b_min_cm = controller->position_cm;
    }

    side = Ball_BSide(controller->position_cm, controller->b_last_side);
    if (controller->b_side_valid == 0u) {
        controller->b_last_side = side;
        controller->b_side_valid = 1u;
        return;
    }
    if (side != controller->b_last_side) {
        if (controller->diagnostics.b_cross_count < 255u) {
            controller->diagnostics.b_cross_count++;
        }
        if ((controller->b_last_side < 0) && (side > 0)) {
            Ball_LatchFirstBOvershoot(controller);
        }
        controller->b_last_side = side;
    }
}

static void Ball_UpdateQuietObserver(BallController *controller,
                                     uint32_t now_ms)
{
    BallQuietEstimate estimate;

    BallQuietObserver_Add(&controller->quiet_observer,
                          controller->position_cm,
                          BALL_TARGET_B_CM,
                          now_ms,
                          &estimate);
    if (estimate.gap_reset != 0u) {
        uint8_t was_quiet = controller->quiet_active;
        controller->quiet_estimate_valid = 0u;
        controller->quiet_positive_error_count = 0u;
        controller->quiet_negative_error_count = 0u;
        controller->quiet_enter_frames = 0u;
        controller->quiet_position_confirm_active = 0u;
        controller->quiet_speed_exit_frames = 0u;
        controller->quiet_hard_error_frames = 0u;
        controller->quiet_bias_active = 0u;
        controller->quiet_active = 0u;
        controller->quiet_pulse_sequence_active = 0u;
        controller->pulse_phase = BALL_PULSE_IDLE;
        controller->pulse_level = 0u;
        controller->pulse_no_move_count = 0u;
        if ((was_quiet != 0u) ||
            (controller->state == BALL_STATE_DONE_HOLD) ||
            ((controller->state == BALL_STATE_TIMEOUT) &&
             (controller->timeout_settled != 0u))) {
            controller->quiet_warmup_hold = 1u;
        }
        BallPidBank_ResetIntegrals(&controller->pid_bank);
        return;
    }
    if (estimate.valid == 0u) {
        controller->quiet_estimate_valid = 0u;
        return;
    }

    controller->quiet_position_cm = estimate.median_position_cm;
    controller->quiet_error_cm = estimate.error_cm;
    controller->quiet_spread_cm = estimate.spread_cm;
    controller->quiet_positive_error_count =
        estimate.positive_error_count;
    controller->quiet_negative_error_count =
        estimate.negative_error_count;
    controller->quiet_estimate_valid = 1u;
    controller->quiet_warmup_hold = 0u;

    {
        int16_t error100 =
            Ball_RoundInt16(Ball_Abs(estimate.error_cm) * 100.0f);
        if (error100 > controller->diagnostics.max_quiet_error100) {
            controller->diagnostics.max_quiet_error100 = error100;
        }
    }
}

static float Ball_DynamicMagnitude(float u_raw_deg)
{
    float magnitude = Ball_Abs(u_raw_deg);
    if (magnitude < BALL_DYNAMIC_MIN_OFFSET_DEG) {
        magnitude = BALL_DYNAMIC_MIN_OFFSET_DEG;
    }
    if (magnitude > BALL_DYNAMIC_MAX_OFFSET_DEG) {
        magnitude = BALL_DYNAMIC_MAX_OFFSET_DEG;
    }
    return magnitude;
}

static void Ball_StartSettlePulse(BallController *controller,
                                  uint32_t now_ms,
                                  int8_t direction)
{
    float magnitude = BALL_PULSE_INITIAL_OFFSET_DEG +
        (float)controller->pulse_level * BALL_PULSE_STEP_OFFSET_DEG;
    if (magnitude > BALL_PULSE_MAX_OFFSET_DEG) {
        magnitude = BALL_PULSE_MAX_OFFSET_DEG;
    }
    controller->pulse_phase = BALL_PULSE_ACTIVE;
    controller->settle_mode = BALL_SETTLE_PULSE;
    controller->pulse_phase_start_ms = now_ms;
    controller->pulse_start_cm = controller->position_cm;
    controller->pulse_direction = direction;
    Ball_SetServoAutomaticOffset(controller, (float)direction * magnitude);
}

static void Ball_CompleteAtB(BallController *controller,
                             uint32_t now_ms,
                             uint8_t timed_out)
{
    Ball_LatchFirstBOvershoot(controller);
    if (controller->diagnostics.t_done_ms == 0u) {
        controller->diagnostics.t_done_ms = Ball_TaskElapsed(controller, now_ms);
    }
    controller->settle_hold_active = 0u;
    Ball_ResetPulse(controller);
    if (controller->quiet_estimate_valid != 0u) {
        controller->quiet_active = 1u;
        controller->quiet_anchor_cm = controller->quiet_position_cm;
        controller->quiet_warmup_hold = 0u;
    } else {
        controller->quiet_active = 0u;
        controller->quiet_warmup_hold = 1u;
    }
    controller->quiet_bias_active = 0u;
    controller->quiet_pulse_sequence_active = 0u;
    controller->settle_mode = BALL_SETTLE_QUIET;
    if (controller->diagnostics.t_quiet_enter_ms == 0u) {
        controller->diagnostics.t_quiet_enter_ms =
            Ball_TaskElapsed(controller, now_ms);
    }
    BallPidBank_ResetIntegrals(&controller->pid_bank);
    Ball_SetServoCenter(controller);

    if (timed_out != 0u) {
        controller->timeout_settled = 1u;
    } else {
        controller->state = BALL_STATE_DONE_HOLD;
        controller->result_flags &= (uint16_t)~BALL_RESULT_RUN;
        controller->result_flags |= BALL_RESULT_PASS;
    }
}

static void Ball_UpdateSettleMotion(BallController *controller)
{
    float speed_abs_cm_s = Ball_Abs(controller->velocity_cm_s);

    if (controller->settle_moving != 0u) {
        if (speed_abs_cm_s <= BALL_SETTLE_MOVE_EXIT_CM_S) {
            if (controller->settle_still_frames < 255u) {
                controller->settle_still_frames++;
            }
            if (controller->settle_still_frames >=
                BALL_SETTLE_MOVE_EXIT_FRAMES) {
                controller->settle_moving = 0u;
                controller->settle_still_frames = 0u;
            }
        } else {
            controller->settle_still_frames = 0u;
        }
    } else if (speed_abs_cm_s >= BALL_SETTLE_MOVE_ENTER_CM_S) {
        controller->settle_moving = 1u;
        controller->settle_still_frames = 0u;
    }
}

static uint8_t Ball_UpdateSettleFastSign(BallController *controller)
{
    int8_t sign = 0;

    if (controller->has_adjacent_velocity != 0u) {
        if (controller->adjacent_velocity_cm_s >=
            BALL_SETTLE_MOVE_ENTER_CM_S) {
            sign = 1;
        } else if (controller->adjacent_velocity_cm_s <=
                   -BALL_SETTLE_MOVE_ENTER_CM_S) {
            sign = -1;
        }
    }
    if (sign == 0) {
        controller->settle_fast_sign_frames = 0u;
        return 0u;
    }
    if (sign == controller->settle_fast_sign) {
        if (controller->settle_fast_sign_frames < 255u) {
            controller->settle_fast_sign_frames++;
        }
    } else {
        controller->settle_fast_sign = sign;
        controller->settle_fast_sign_frames = 1u;
    }
    return (controller->settle_fast_sign_frames >= 2u) ? 1u : 0u;
}

static float Ball_DampingMagnitude(float speed_abs_cm_s)
{
    float excess_cm_s = speed_abs_cm_s - BALL_DYNAMIC_SPEED_CM_S;
    float magnitude;

    if (excess_cm_s < 0.0f) {
        excess_cm_s = 0.0f;
    }
    magnitude = BALL_SETTLE_DAMP_BASE_DEG +
        BALL_SETTLE_DAMP_KV_DEG_PER_CM_S * excess_cm_s;
    return Ball_Clamp(magnitude,
                      BALL_SETTLE_DAMP_BASE_DEG,
                      BALL_SETTLE_DAMP_MAX_DEG);
}

static void Ball_ApplyVelocityBrake(BallController *controller,
                                    float speed_abs_cm_s)
{
    float direction = (controller->velocity_cm_s > 0.0f) ? -1.0f : 1.0f;
    float magnitude = Ball_DampingMagnitude(speed_abs_cm_s);
    float pid_magnitude = Ball_Abs(controller->pid_bank.fused_output_deg);

    if (pid_magnitude > magnitude) {
        magnitude = pid_magnitude;
    }
    magnitude = Ball_Clamp(magnitude,
                           BALL_SETTLE_DAMP_BASE_DEG,
                           BALL_SETTLE_DAMP_MAX_DEG);
    controller->settle_mode = BALL_SETTLE_DAMP;
    Ball_SetServoAutomaticOffset(controller, direction * magnitude);
}

static uint8_t Ball_PredictedTargetCrossing(float measured_error_cm,
                                            float predicted_error_cm,
                                            float velocity_cm_s)
{
    if ((measured_error_cm * velocity_cm_s > 0.0f) &&
        (measured_error_cm * predicted_error_cm <= 0.0f)) {
        return 1u;
    }
    return 0u;
}

static uint8_t Ball_QuietDirectionStable(const BallController *controller)
{
    if (controller->quiet_error_cm > 0.0f) {
        return (controller->quiet_positive_error_count >=
                BALL_QUIET_BIAS_DIRECTION_COUNT) ? 1u : 0u;
    }
    if (controller->quiet_error_cm < 0.0f) {
        return (controller->quiet_negative_error_count >=
                BALL_QUIET_BIAS_DIRECTION_COUNT) ? 1u : 0u;
    }
    return 0u;
}

static uint8_t Ball_QuietMovingAway(const BallController *controller)
{
    return ((controller->quiet_error_cm * controller->velocity_cm_s) <
            0.0f) ? 1u : 0u;
}

static uint8_t Ball_QuietOutwardPrediction(const BallController *controller)
{
    float quiet_predicted_cm;

    /* B lies in the negative direction.  This immediate safety channel is
     * intentionally narrower than ordinary "moving away": it only fires
     * when the predicted position is already beyond B and velocity continues
     * in the outward (more-negative) direction.  Other speed evidence still
     * requires the specified three-frame confirmation. */
    quiet_predicted_cm = controller->quiet_position_cm +
        BALL_SETTLE_PREDICT_S * controller->velocity_cm_s;
    if ((controller->quiet_estimate_valid == 0u) ||
        (Ball_Abs(controller->velocity_cm_s) <
         BALL_QUIET_PULSE_END_SPEED_CM_S) ||
        (Ball_QuietMovingAway(controller) == 0u) ||
        (controller->quiet_position_cm >= BALL_TARGET_B_CM) ||
        (quiet_predicted_cm >= BALL_TARGET_B_CM)) {
        return 0u;
    }
    return 1u;
}

static uint8_t Ball_QuietPulseRearmed(const BallController *controller,
                                      uint32_t now_ms)
{
    if (controller->quiet_has_last_pulse_end == 0u) {
        return 1u;
    }
    return ((uint32_t)(now_ms - controller->quiet_last_pulse_end_ms) >=
            BALL_QUIET_PULSE_REARM_MS) ? 1u : 0u;
}

static void Ball_EnterQuiet(BallController *controller, uint32_t now_ms)
{
    Ball_ResetPulse(controller);
    controller->quiet_active = 1u;
    controller->quiet_anchor_cm = controller->quiet_position_cm;
    controller->quiet_enter_frames = 0u;
    controller->quiet_position_confirm_active = 0u;
    controller->quiet_speed_exit_frames = 0u;
    controller->quiet_hard_error_frames = 0u;
    controller->quiet_bias_active = 0u;
    controller->quiet_reacquire_wait_frames = 0u;
    controller->quiet_warmup_hold = 0u;
    controller->quiet_pulse_sequence_active = 0u;
    controller->settle_mode = BALL_SETTLE_QUIET;
    controller->quiet_event_window_active = 1u;
    controller->quiet_event_window_start_ms = now_ms;
    controller->quiet_event_window_count = 0u;
    if (controller->diagnostics.t_quiet_enter_ms == 0u) {
        controller->diagnostics.t_quiet_enter_ms =
            Ball_TaskElapsed(controller, now_ms);
    }
    BallPidBank_ResetIntegrals(&controller->pid_bank);
    Ball_SetServoCenter(controller);
}

static void Ball_ExitQuiet(BallController *controller,
                           uint32_t now_ms,
                           BallQuietExitReason reason)
{
    if (controller->quiet_pulse_sequence_active != 0u) {
        controller->quiet_last_pulse_end_ms = now_ms;
        controller->quiet_has_last_pulse_end = 1u;
    }
    controller->quiet_active = 0u;
    controller->quiet_position_confirm_active = 0u;
    controller->quiet_speed_exit_frames = 0u;
    controller->quiet_hard_error_frames = 0u;
    controller->quiet_bias_active = 0u;
    controller->quiet_pulse_sequence_active = 0u;
    controller->quiet_reacquire_wait_frames =
        BALL_QUIET_RECAPTURE_WAIT_FRAMES;
    controller->pulse_phase = BALL_PULSE_IDLE;
    controller->pulse_level = 0u;
    controller->pulse_no_move_count = 0u;
    controller->settle_mode = BALL_SETTLE_QUIET_RECAPTURE;
    if (controller->diagnostics.quiet_exit_count < 65535u) {
        controller->diagnostics.quiet_exit_count++;
    }
    controller->diagnostics.quiet_exit_reason = (uint8_t)reason;
    BallPidBank_ResetIntegrals(&controller->pid_bank);
}

static void Ball_StartQuietPulse(BallController *controller,
                                 uint32_t now_ms)
{
    float magnitude = BALL_PULSE_INITIAL_OFFSET_DEG +
        (float)controller->pulse_level * BALL_PULSE_STEP_OFFSET_DEG;
    int8_t direction = (controller->quiet_error_cm >= 0.0f) ? 1 : -1;

    magnitude = Ball_Clamp(magnitude,
                           BALL_PULSE_INITIAL_OFFSET_DEG,
                           BALL_PULSE_MAX_OFFSET_DEG);
    controller->pulse_phase = BALL_PULSE_ACTIVE;
    controller->settle_mode = BALL_SETTLE_QUIET_PULSE;
    controller->pulse_phase_start_ms = now_ms;
    controller->pulse_start_cm = controller->quiet_position_cm;
    controller->pulse_direction = direction;
    controller->quiet_pulse_sequence_active = 1u;
    controller->quiet_bias_active = 0u;
    if (controller->diagnostics.quiet_pulse_count < 65535u) {
        controller->diagnostics.quiet_pulse_count++;
    }
    Ball_SetServoAutomaticOffset(controller,
        (float)direction * magnitude);
}

static void Ball_StartQuietObserve(BallController *controller,
                                   uint32_t now_ms)
{
    controller->pulse_phase = BALL_PULSE_OBSERVE;
    controller->pulse_phase_start_ms = now_ms;
    controller->quiet_last_pulse_end_ms = now_ms;
    controller->quiet_has_last_pulse_end = 1u;
    controller->settle_mode = BALL_SETTLE_QUIET_OBSERVE;
    Ball_SetServoCenter(controller);
}

static void Ball_FinishQuietObserve(BallController *controller)
{
    float effective_move_cm = 0.0f;

    if (controller->quiet_estimate_valid != 0u) {
        effective_move_cm =
            (float)controller->pulse_direction *
            (controller->quiet_position_cm - controller->pulse_start_cm);
    }
    if ((controller->quiet_estimate_valid != 0u) &&
        (controller->quiet_spread_cm <= BALL_QUIET_BIAS_SPREAD_CM) &&
        (effective_move_cm < BALL_PULSE_EFFECTIVE_MOVE_CM)) {
        if (controller->pulse_no_move_count < 255u) {
            controller->pulse_no_move_count++;
        }
        if (controller->pulse_no_move_count >=
            BALL_PULSE_NO_MOVE_REPEATS) {
            controller->pulse_no_move_count = 0u;
            if ((BALL_PULSE_INITIAL_OFFSET_DEG +
                 (float)controller->pulse_level *
                 BALL_PULSE_STEP_OFFSET_DEG) <
                BALL_PULSE_MAX_OFFSET_DEG) {
                controller->pulse_level++;
            }
        }
    } else if (effective_move_cm >= BALL_PULSE_EFFECTIVE_MOVE_CM) {
        controller->pulse_no_move_count = 0u;
        controller->pulse_level = 0u;
    }

    controller->pulse_phase = BALL_PULSE_IDLE;
    controller->quiet_pulse_sequence_active = 0u;
    controller->quiet_bias_active = 0u;
    controller->quiet_position_confirm_active = 0u;
    controller->quiet_speed_exit_frames = 0u;
    controller->quiet_hard_error_frames = 0u;
    controller->quiet_anchor_cm = controller->quiet_position_cm;
    controller->settle_mode = BALL_SETTLE_QUIET;
    Ball_SetServoCenter(controller);
}

static BallQuietControlResult Ball_ControlQuiet(
    BallController *controller,
    uint32_t now_ms,
    uint8_t predicted_crossing,
    uint8_t allow_bias_pulse)
{
    float error_abs_cm = Ball_Abs(controller->quiet_error_cm);
    float speed_abs_cm_s = Ball_Abs(controller->velocity_cm_s);
    uint8_t moving_away = Ball_QuietMovingAway(controller);

    if (controller->quiet_reacquire_wait_frames != 0u) {
        controller->quiet_reacquire_wait_frames--;
        controller->settle_mode = BALL_SETTLE_QUIET_RECAPTURE;
        Ball_SetServoCenter(controller);
        return BALL_QUIET_CONTROL_HANDLED;
    }

    if (controller->quiet_warmup_hold != 0u) {
        if (controller->quiet_estimate_valid == 0u) {
            controller->settle_mode = BALL_SETTLE_QUIET;
            Ball_SetServoCenter(controller);
            return BALL_QUIET_CONTROL_HANDLED;
        }
        controller->quiet_warmup_hold = 0u;
    }

    if (controller->quiet_pulse_sequence_active != 0u) {
        if (Ball_QuietOutwardPrediction(controller) != 0u) {
            Ball_ExitQuiet(controller, now_ms,
                           BALL_QUIET_EXIT_OUTWARD_PREDICTION);
            return BALL_QUIET_CONTROL_SAFETY_BRAKE;
        }
        if (controller->pulse_phase == BALL_PULSE_ACTIVE) {
            uint8_t reversed = ((controller->has_adjacent_velocity != 0u) &&
                ((float)controller->pulse_direction *
                 controller->adjacent_velocity_cm_s <=
                 -BALL_SETTLE_MOVE_EXIT_CM_S)) ? 1u : 0u;
            if ((predicted_crossing != 0u) ||
                (speed_abs_cm_s >=
                 BALL_QUIET_PULSE_END_SPEED_CM_S) ||
                (reversed != 0u) ||
                ((uint32_t)(now_ms -
                            controller->pulse_phase_start_ms) >=
                 BALL_QUIET_PULSE_ACTIVE_MS)) {
                Ball_StartQuietObserve(controller, now_ms);
            } else {
                float magnitude = BALL_PULSE_INITIAL_OFFSET_DEG +
                    (float)controller->pulse_level *
                    BALL_PULSE_STEP_OFFSET_DEG;
                magnitude = Ball_Clamp(magnitude,
                    BALL_PULSE_INITIAL_OFFSET_DEG,
                    BALL_PULSE_MAX_OFFSET_DEG);
                controller->settle_mode = BALL_SETTLE_QUIET_PULSE;
                Ball_SetServoAutomaticOffset(controller,
                    (float)controller->pulse_direction * magnitude);
            }
            return BALL_QUIET_CONTROL_HANDLED;
        }
        if (controller->pulse_phase == BALL_PULSE_OBSERVE) {
            controller->settle_mode = BALL_SETTLE_QUIET_OBSERVE;
            Ball_SetServoCenter(controller);
            if ((uint32_t)(now_ms -
                           controller->pulse_phase_start_ms) >=
                BALL_QUIET_PULSE_OBSERVE_MS) {
                Ball_FinishQuietObserve(controller);
            }
            return BALL_QUIET_CONTROL_HANDLED;
        }
        controller->quiet_pulse_sequence_active = 0u;
    }

    if (controller->quiet_active != 0u) {
        if (Ball_QuietOutwardPrediction(controller) != 0u) {
            Ball_ExitQuiet(controller, now_ms,
                           BALL_QUIET_EXIT_OUTWARD_PREDICTION);
            return BALL_QUIET_CONTROL_SAFETY_BRAKE;
        }

        if (error_abs_cm > BALL_QUIET_HARD_ERROR_CM) {
            if (controller->quiet_hard_error_frames < 255u) {
                controller->quiet_hard_error_frames++;
            }
        } else {
            controller->quiet_hard_error_frames = 0u;
        }
        if (controller->quiet_hard_error_frames >=
            BALL_QUIET_HARD_ERROR_FRAMES) {
            Ball_ExitQuiet(controller, now_ms,
                           BALL_QUIET_EXIT_HARD_ERROR);
            return BALL_QUIET_CONTROL_EXIT;
        }

        if ((speed_abs_cm_s > BALL_QUIET_SPEED_EXIT_CM_S) &&
            (moving_away != 0u)) {
            if (controller->quiet_speed_exit_frames < 255u) {
                controller->quiet_speed_exit_frames++;
            }
        } else {
            controller->quiet_speed_exit_frames = 0u;
        }
        if (controller->quiet_speed_exit_frames >=
            BALL_QUIET_SPEED_EXIT_FRAMES) {
            Ball_ExitQuiet(controller, now_ms,
                           BALL_QUIET_EXIT_SPEED);
            return BALL_QUIET_CONTROL_SAFETY_BRAKE;
        }

        if ((error_abs_cm > BALL_QUIET_HOLD_ERROR_CM) &&
            (Ball_Abs(controller->quiet_position_cm -
                      controller->quiet_anchor_cm) >=
             BALL_QUIET_ANCHOR_MOVE_CM)) {
            if (controller->quiet_position_confirm_active == 0u) {
                controller->quiet_position_confirm_active = 1u;
                controller->quiet_position_confirm_start_ms = now_ms;
            } else if ((uint32_t)(now_ms -
                                  controller->quiet_position_confirm_start_ms) >=
                       BALL_QUIET_POSITION_CONFIRM_MS) {
                BallQuietExitReason reason =
                    (error_abs_cm > BALL_QUIET_LARGE_ERROR_CM) ?
                        BALL_QUIET_EXIT_LARGE_ERROR :
                        BALL_QUIET_EXIT_POSITION;
                Ball_ExitQuiet(controller, now_ms, reason);
                return BALL_QUIET_CONTROL_EXIT;
            }
        } else {
            controller->quiet_position_confirm_active = 0u;
        }

        if ((allow_bias_pulse != 0u) &&
            (error_abs_cm > BALL_QUIET_BIAS_ERROR_CM) &&
            (error_abs_cm <= BALL_QUIET_LARGE_ERROR_CM) &&
            (speed_abs_cm_s <= BALL_QUIET_BIAS_SPEED_CM_S) &&
            (controller->quiet_spread_cm <=
             BALL_QUIET_BIAS_SPREAD_CM) &&
            (Ball_QuietDirectionStable(controller) != 0u)) {
            if (controller->quiet_bias_active == 0u) {
                controller->quiet_bias_active = 1u;
                controller->quiet_bias_start_ms = now_ms;
            } else if (((uint32_t)(now_ms -
                                   controller->quiet_bias_start_ms) >=
                        BALL_QUIET_BIAS_CONFIRM_MS) &&
                       (Ball_QuietPulseRearmed(controller, now_ms) != 0u)) {
                Ball_StartQuietPulse(controller, now_ms);
                return BALL_QUIET_CONTROL_HANDLED;
            }
        } else {
            controller->quiet_bias_active = 0u;
        }

        controller->settle_mode = BALL_SETTLE_QUIET;
        Ball_SetServoCenter(controller);
        return BALL_QUIET_CONTROL_HANDLED;
    }

    if ((controller->quiet_estimate_valid != 0u) &&
        (moving_away != 0u) &&
        (speed_abs_cm_s > BALL_QUIET_ENTRY_AWAY_GUARD_CM_S)) {
        if (controller->quiet_speed_exit_frames < 255u) {
            controller->quiet_speed_exit_frames++;
        }
    } else {
        controller->quiet_speed_exit_frames = 0u;
    }

    if (controller->quiet_speed_exit_frames >=
        BALL_QUIET_ENTRY_AWAY_GUARD_FRAMES) {
        controller->quiet_enter_frames = 0u;
        return BALL_QUIET_CONTROL_NOT_HANDLED;
    }

    if ((controller->quiet_estimate_valid != 0u) &&
        (error_abs_cm <= BALL_QUIET_ENTER_ERROR_CM) &&
        (speed_abs_cm_s <= BALL_QUIET_ENTER_SPEED_CM_S)) {
        if (controller->quiet_enter_frames < 255u) {
            controller->quiet_enter_frames++;
        }
        if (controller->quiet_enter_frames >= BALL_QUIET_ENTER_FRAMES) {
            Ball_EnterQuiet(controller, now_ms);
            return BALL_QUIET_CONTROL_HANDLED;
        }
        /* The five-frame entry qualification is itself observation time.
         * Do not let the legacy three-frame pulse path fire while QHLD is
         * being confirmed at the 0.60..0.70 cm boundary. */
        controller->settle_mode = BALL_SETTLE_OBSERVE;
        Ball_SetServoCenter(controller);
        return BALL_QUIET_CONTROL_HANDLED;
    } else {
        /* Entry is a conjunctive five-frame qualification.  A history of
         * low-speed frames cannot make one boundary/noise frame enter QHLD. */
        controller->quiet_enter_frames = 0u;
    }
    return BALL_QUIET_CONTROL_NOT_HANDLED;
}

static void Ball_ControlBSettle(BallController *controller,
                                uint32_t now_ms,
                                uint8_t timed_out)
{
    float measured_error_cm = BALL_TARGET_B_CM - controller->position_cm;
    float speed_abs_cm_s = Ball_Abs(controller->velocity_cm_s);
    float damping_error_cm;
    float u_raw_deg = controller->pid_bank.fused_output_deg;
    uint8_t predicted_crossing;
    uint8_t moving_for_control;
    uint8_t pulse_motion_detected;
    uint8_t moving_toward_target;
    uint8_t moving_away_from_target;
    uint8_t fast_motion_confirmed;
    uint8_t fast_filter_conflict;

    controller->u_raw_deg = u_raw_deg;
    controller->damping_predicted_position_cm = controller->position_cm +
        BALL_SETTLE_PREDICT_S * controller->velocity_cm_s;
    damping_error_cm = BALL_TARGET_B_CM -
                       controller->damping_predicted_position_cm;
    predicted_crossing = Ball_PredictedTargetCrossing(
        measured_error_cm, damping_error_cm, controller->velocity_cm_s);

    if (speed_abs_cm_s <= BALL_SETTLE_MOVE_EXIT_CM_S) {
        if (controller->settle_pulse_ready_frames < 255u) {
            controller->settle_pulse_ready_frames++;
        }
    } else {
        controller->settle_pulse_ready_frames = 0u;
    }

    Ball_UpdateSettleMotion(controller);
    fast_motion_confirmed = Ball_UpdateSettleFastSign(controller);
    moving_for_control = ((controller->settle_moving != 0u) ||
        (speed_abs_cm_s > BALL_DONE_SPEED_TOL_CM_S)) ? 1u : 0u;
    pulse_motion_detected = ((moving_for_control != 0u) ||
        ((controller->has_adjacent_velocity != 0u) &&
         (Ball_Abs(controller->adjacent_velocity_cm_s) >=
          BALL_SETTLE_MOVE_ENTER_CM_S))) ? 1u : 0u;
    moving_toward_target = (measured_error_cm *
                            controller->velocity_cm_s > 0.0f) ? 1u : 0u;
    moving_away_from_target = (measured_error_cm *
                               controller->velocity_cm_s < 0.0f) ? 1u : 0u;
    fast_filter_conflict = ((fast_motion_confirmed != 0u) &&
        (controller->velocity_cm_s *
         (float)controller->settle_fast_sign < 0.0f)) ? 1u : 0u;
    if ((fast_motion_confirmed != 0u) &&
        (Ball_Abs(measured_error_cm) <= BALL_SETTLE_DAMP_BAND_CM)) {
        moving_for_control = 1u;
    }

    if ((Ball_Abs(measured_error_cm) <= BALL_DONE_POSITION_TOL_CM) &&
        (speed_abs_cm_s <= BALL_DONE_SPEED_TOL_CM_S)) {
        controller->pulse_phase = BALL_PULSE_IDLE;
        controller->quiet_pulse_sequence_active = 0u;
        controller->quiet_bias_active = 0u;
        controller->settle_mode =
            (controller->quiet_active != 0u) ?
                BALL_SETTLE_QUIET : BALL_SETTLE_HOLD;
        Ball_SetServoCenter(controller);
        if (controller->settle_hold_active == 0u) {
            controller->settle_hold_active = 1u;
            controller->settle_hold_start_ms = now_ms;
        } else if ((uint32_t)(now_ms - controller->settle_hold_start_ms) >=
                   BALL_DONE_HOLD_MS) {
            Ball_CompleteAtB(controller, now_ms, timed_out);
        }
        return;
    }

    controller->settle_hold_active = 0u;

    /*
     * A confirmed rebound is faster evidence than the five-sample filtered
     * velocity.  While those two signals disagree, coast instead of keeping
     * the former +28 degree BRAKE_B command and accelerating the rebound.
     */
    if ((controller->diagnostics.b_release_reason ==
         BALL_TRANSITION_B_REBOUND) &&
        (controller->has_adjacent_velocity != 0u) &&
        (controller->adjacent_velocity_cm_s > 0.0f) &&
        (controller->velocity_cm_s < 0.0f)) {
        controller->pulse_phase = BALL_PULSE_IDLE;
        controller->settle_mode = BALL_SETTLE_DAMP;
        Ball_SetServoCenter(controller);
        return;
    }

    /* Immediately after the main brake release, spend 120 ms only removing
     * energy.  No position pulse may be issued during this window. */
    if (controller->b_settle_brake_guard_active != 0u) {
        if ((uint32_t)(now_ms - controller->stage_start_ms) <
            BALL_B_SETTLE_POST_BRAKE_GUARD_MS) {
            controller->pulse_phase = BALL_PULSE_IDLE;
            controller->settle_mode = BALL_SETTLE_DAMP;
            if (speed_abs_cm_s > BALL_SETTLE_MOVE_EXIT_CM_S) {
                Ball_ApplyVelocityBrake(controller, speed_abs_cm_s);
            } else {
                Ball_SetServoCenter(controller);
            }
            return;
        }
        controller->b_settle_brake_guard_active = 0u;
    }

    {
        BallQuietControlResult quiet_result =
            Ball_ControlQuiet(controller, now_ms,
                              predicted_crossing, 1u);
        if (quiet_result == BALL_QUIET_CONTROL_SAFETY_BRAKE) {
            Ball_ApplyVelocityBrake(controller, speed_abs_cm_s);
            return;
        }
        if (quiet_result == BALL_QUIET_CONTROL_EXIT) {
            controller->settle_mode = BALL_SETTLE_QUIET_RECAPTURE;
            Ball_SetServoCenter(controller);
            return;
        }
        if (quiet_result == BALL_QUIET_CONTROL_HANDLED) {
            return;
        }
    }

    if ((controller->quiet_estimate_valid == 0u) &&
        (Ball_Abs(measured_error_cm) <= BALL_QUIET_HOLD_ERROR_CM) &&
        (speed_abs_cm_s <= BALL_QUIET_ENTER_SPEED_CM_S) &&
        !((measured_error_cm * controller->velocity_cm_s < 0.0f) &&
          (speed_abs_cm_s > BALL_QUIET_ENTRY_AWAY_GUARD_CM_S))) {
        /* Collect the first five median samples silently.  Without this
         * warm-up guard, the legacy low-speed pulse could start on frame 3
         * before the five-sample observer is even valid. */
        controller->settle_mode = BALL_SETTLE_OBSERVE;
        Ball_SetServoCenter(controller);
        return;
    }

    if (controller->pulse_phase == BALL_PULSE_ACTIVE) {
        if ((pulse_motion_detected != 0u) || (predicted_crossing != 0u)) {
            controller->pulse_phase = BALL_PULSE_IDLE;
            controller->settle_mode = BALL_SETTLE_DAMP;
            /* Do not start another position pulse in this same measurement.
             * Coast for one observation cycle after motion/crossing is seen. */
            Ball_SetServoCenter(controller);
            return;
        } else if ((uint32_t)(now_ms - controller->pulse_phase_start_ms) <
                   BALL_PULSE_ACTIVE_MS) {
            float magnitude = BALL_PULSE_INITIAL_OFFSET_DEG +
                (float)controller->pulse_level * BALL_PULSE_STEP_OFFSET_DEG;
            magnitude = Ball_Clamp(magnitude,
                                   BALL_PULSE_INITIAL_OFFSET_DEG,
                                   BALL_PULSE_MAX_OFFSET_DEG);
            controller->settle_mode = BALL_SETTLE_PULSE;
            Ball_SetServoAutomaticOffset(controller,
                (float)controller->pulse_direction * magnitude);
            return;
        } else {
            controller->pulse_phase = BALL_PULSE_OBSERVE;
            controller->pulse_phase_start_ms = now_ms;
            controller->settle_mode = BALL_SETTLE_OBSERVE;
            Ball_SetServoCenter(controller);
            return;
        }
    }

    if (controller->pulse_phase == BALL_PULSE_OBSERVE) {
        if (pulse_motion_detected != 0u) {
            controller->pulse_phase = BALL_PULSE_IDLE;
            controller->settle_mode = BALL_SETTLE_DAMP;
        } else if ((uint32_t)(now_ms - controller->pulse_phase_start_ms) <
                   BALL_PULSE_OBSERVE_MS) {
            controller->settle_mode = BALL_SETTLE_OBSERVE;
            Ball_SetServoCenter(controller);
            return;
        } else {
            if ((float)controller->pulse_direction *
                (controller->position_cm - controller->pulse_start_cm) <
                BALL_PULSE_EFFECTIVE_MOVE_CM) {
                controller->pulse_no_move_count++;
                if (controller->pulse_no_move_count >=
                    BALL_PULSE_NO_MOVE_REPEATS) {
                    controller->pulse_no_move_count = 0u;
                    if ((BALL_PULSE_INITIAL_OFFSET_DEG +
                         (float)controller->pulse_level *
                         BALL_PULSE_STEP_OFFSET_DEG) <
                        BALL_PULSE_MAX_OFFSET_DEG) {
                        controller->pulse_level++;
                    }
                }
            } else {
                controller->pulse_no_move_count = 0u;
                controller->pulse_level = 0u;
            }
            controller->pulse_phase = BALL_PULSE_IDLE;
        }
    }

    if (moving_for_control != 0u) {
        float recovery_guard_velocity_cm_s =
            (controller->velocity_cm_s < 0.0f) ?
                controller->velocity_cm_s : 0.0f;
        float recovery_v_in_cm_s;
        float recovery_d_remaining_cm;
        float recovery_d_stop_cm;
        uint8_t recovery_envelope_brake;

        if ((fast_motion_confirmed != 0u) &&
            (controller->settle_fast_sign < 0) &&
            (controller->adjacent_velocity_cm_s <
             recovery_guard_velocity_cm_s)) {
            recovery_guard_velocity_cm_s =
                controller->adjacent_velocity_cm_s;
        }
        recovery_v_in_cm_s = -recovery_guard_velocity_cm_s;
        recovery_d_remaining_cm = controller->position_cm +
            BALL_B_GUARD_POSITION_LEAD_S *
                recovery_guard_velocity_cm_s - BALL_TARGET_B_CM;
        recovery_d_stop_cm = BALL_B_STOP_DELAY_S * recovery_v_in_cm_s +
            (recovery_v_in_cm_s * recovery_v_in_cm_s) /
                (2.0f * BALL_B_STOP_DECEL_CM_S2) +
            BALL_B_STOP_MARGIN_CM;
        recovery_envelope_brake =
            ((moving_toward_target != 0u) &&
             (controller->velocity_cm_s < 0.0f) &&
             (controller->position_cm > BALL_TARGET_B_CM) &&
             (speed_abs_cm_s > BALL_DYNAMIC_SPEED_CM_S) &&
             (recovery_d_remaining_cm <=
              (recovery_d_stop_cm +
               BALL_B_SETTLE_RECOVERY_MARGIN_CM))) ? 1u : 0u;
        controller->pulse_phase = BALL_PULSE_IDLE;
        controller->settle_mode = BALL_SETTLE_DAMP;

        /*
         * Inside +/-1 cm, never command in the direction of velocity.  Coast
         * while approaching, then brake when the 160 ms prediction crosses B.
         */
        if (recovery_envelope_brake != 0u) {
            /* If the first main brake stopped short, the recovery motion still
             * obeys the same stopping-distance envelope.  This prevents the
             * recovery drive from crossing the first +/-0.8 cm band at high
             * speed and does not inject energy toward B. */
            if ((fast_motion_confirmed != 0u) &&
                (controller->settle_fast_sign < 0)) {
                Ball_ApplyVelocityBrake(controller, speed_abs_cm_s);
            } else {
                /* Require two adjacent-sample frames in the B direction, and
                 * release as soon as that evidence disappears. */
                Ball_SetServoCenter(controller);
            }
        } else if ((Ball_Abs(measured_error_cm) <= BALL_SETTLE_DAMP_BAND_CM) &&
            (fast_filter_conflict != 0u)) {
            /* Two consecutive fast samples contradict the lagged fit.  Coast
             * until the estimator catches up, which cannot add kinetic energy. */
            Ball_SetServoCenter(controller);
        } else if ((moving_away_from_target != 0u) ||
            (predicted_crossing != 0u) ||
            ((controller->damping_predicted_position_cm < BALL_TARGET_B_CM) &&
             (controller->velocity_cm_s < 0.0f))) {
            Ball_ApplyVelocityBrake(controller, speed_abs_cm_s);
        } else if (Ball_Abs(measured_error_cm) <=
                   BALL_SETTLE_DAMP_BAND_CM) {
            Ball_SetServoCenter(controller);
        } else if (moving_toward_target != 0u) {
            float direction = (u_raw_deg > 0.0f) ? 1.0f :
                              ((u_raw_deg < 0.0f) ? -1.0f :
                               ((measured_error_cm > 0.0f) ? 1.0f : -1.0f));
            Ball_SetServoAutomaticOffset(controller,
                direction * Ball_DynamicMagnitude(u_raw_deg));
        } else {
            Ball_ApplyVelocityBrake(controller, speed_abs_cm_s);
        }
        return;
    }

    if (controller->settle_pulse_ready_frames <
        BALL_SETTLE_MOVE_EXIT_FRAMES) {
        controller->settle_mode = BALL_SETTLE_OBSERVE;
        Ball_SetServoCenter(controller);
        return;
    }

    Ball_StartSettlePulse(controller, now_ms,
        (u_raw_deg > 0.0f) ? 1 :
        ((u_raw_deg < 0.0f) ? -1 :
         ((measured_error_cm >= 0.0f) ? 1 : -1)));
}

static void Ball_EnterTimeout(BallController *controller)
{
    BallState phase = controller->state;
    controller->diagnostics.timeout_phase = phase;
    controller->state = BALL_STATE_TIMEOUT;
    controller->result_flags |= BALL_RESULT_TIMEOUT;
    controller->result_flags &= (uint16_t)~BALL_RESULT_PASS;
    controller->timeout_pending = 0u;
    controller->timeout_settled = 0u;
    Ball_ResetPulse(controller);
}

static void Ball_StartTaskFromReady(BallController *controller,
                                    uint32_t now_ms)
{
    BallEstimate estimate;
    int16_t zero_px;

    controller->control_now_ms = now_ms;
    zero_px = controller->center_median_dx_px;
    memset(&controller->diagnostics, 0, sizeof(controller->diagnostics));
    controller->diagnostics.timeout_phase = BALL_STATE_IDLE;
    controller->result_flags = BALL_RESULT_RUN;
    controller->task_start_ms = now_ms;
    controller->stage_start_ms = now_ms;
    controller->task_started = 1u;
    controller->start_latched = 0u;
    controller->timeout_pending = 0u;
    controller->timeout_settled = 0u;
    controller->dx_zero_px = zero_px;
    controller->position_cm = -(float)(controller->last_dx_px - zero_px) /
                              BALL_PX_PER_CM;
    Ball_ResetMotionHistory(controller);
    BallPidBank_Reset(&controller->pid_bank);
    BallEstimator_Add(&controller->estimator,
                      controller->position_cm,
                      now_ms,
                      &estimate);
    controller->predicted_position_cm = controller->position_cm;
    controller->damping_predicted_position_cm = controller->position_cm;
    controller->has_last_position = 1u;
    controller->last_position_cm = controller->position_cm;
    controller->last_position_ms = now_ms;
    controller->drive_reference_cm = controller->position_cm;
    controller->drive_motion_detected = 0u;
    controller->a_peak_cm = controller->position_cm;
    controller->a_peak_valid = 1u;
    controller->state = BALL_STATE_DRIVE_A;
    BallPidBank_Update(&controller->pid_bank,
                       BALL_TARGET_A_CM,
                       controller->position_cm,
                       0.0f,
                       now_ms,
                       0u);
    Ball_ResetCenterWindow(controller);
    Ball_ClearLog(controller);
    Ball_SetServoPhaseOffset(controller, BALL_DRIVE_A_OFFSET_DEG);
    Ball_Log(controller, now_ms, 0u);
}

void BallController_Init(BallController *controller, uint32_t now_ms)
{
    memset(controller, 0, sizeof(*controller));
    controller->control_now_ms = now_ms;
    controller->state = BALL_STATE_WAIT_CENTER;
    controller->wait_start_ms = now_ms;
    controller->diagnostics.timeout_phase = BALL_STATE_WAIT_CENTER;
    BallEstimator_Init(&controller->estimator);
    BallFastEstimator_Init(&controller->fast_estimator);
    BallPidBank_Init(&controller->pid_bank);
    BallQuietObserver_Init(&controller->quiet_observer);
    Ball_SetServoCenter(controller);
}

void BallController_RequestStart(BallController *controller, uint32_t now_ms)
{
    controller->control_now_ms = now_ms;
    if ((controller->state != BALL_STATE_WAIT_CENTER) &&
        (controller->state != BALL_STATE_IDLE) &&
        (controller->state != BALL_STATE_ABORTED) &&
        (controller->state != BALL_STATE_VISION_FAULT)) {
        return;
    }

    /* Start immediately; do not wait for a centered, stable ball. */
    if (controller->has_valid_measurement == 0u) {
        controller->last_dx_px = BALL_IMAGE_CENTER_DX_PX;
    }
    controller->center_median_dx_px = BALL_IMAGE_CENTER_DX_PX;
    Ball_StartTaskFromReady(controller, now_ms);
}

void BallController_RequestAbort(BallController *controller, uint32_t now_ms)
{
    controller->control_now_ms = now_ms;
    if (controller->state == BALL_STATE_IDLE) {
        Ball_SetServoCenter(controller);
        return;
    }

    controller->state = BALL_STATE_ABORTED;
    controller->result_flags = BALL_RESULT_ABORT;
    controller->task_started = 0u;
    controller->start_latched = 0u;
    controller->timeout_pending = 0u;
    controller->timeout_settled = 0u;
    controller->has_valid_measurement = 0u;
    memset(&controller->diagnostics, 0, sizeof(controller->diagnostics));
    controller->diagnostics.timeout_phase = BALL_STATE_ABORTED;
    Ball_ResetCenterWindow(controller);
    Ball_ResetMotionHistory(controller);
    BallPidBank_Reset(&controller->pid_bank);
    Ball_ClearLog(controller);
    Ball_SetServoCenter(controller);
    Ball_Log(controller, now_ms, 0u);
}

void BallController_RequestTimeout(BallController *controller, uint32_t now_ms)
{
    controller->control_now_ms = now_ms;
    if (BallController_TaskTimerActive(controller) != 0u) {
        uint32_t reference_ms = controller->has_valid_measurement ?
            controller->last_valid_ms : controller->wait_start_ms;
        uint32_t age_ms = (uint32_t)(now_ms - reference_ms);
        Ball_EnterTimeout(controller);
        Ball_Log(controller, now_ms,
                 (age_ms > 65535u) ? 65535u : (uint16_t)age_ms);
    }
}

void BallController_OnVisionFault(BallController *controller,
                                  uint32_t now_ms,
                                  uint16_t data_age_ms)
{
    uint8_t task_was_started = controller->task_started;
    controller->control_now_ms = now_ms;

    if (BallController_VisionWatchActive(controller) == 0u) {
        controller->has_valid_measurement = 0u;
        Ball_ResetCenterWindow(controller);
        Ball_ResetMotionHistory(controller);
        Ball_SetServoCenter(controller);
        return;
    }
    controller->state = BALL_STATE_VISION_FAULT;
    controller->result_flags |= BALL_RESULT_VISION;
    controller->result_flags &= (uint16_t)~BALL_RESULT_PASS;
    controller->task_started = 0u;
    if (task_was_started != 0u) {
        controller->start_latched = 0u;
    }
    controller->timeout_pending = 0u;
    controller->timeout_settled = 0u;
    Ball_ResetCenterWindow(controller);
    Ball_ResetMotionHistory(controller);
    if (task_was_started != 0u) {
        BallPidBank_Reset(&controller->pid_bank);
    }
    Ball_SetServoCenter(controller);
    Ball_Log(controller, now_ms, data_age_ms);
}

void BallController_OnMeasurement(BallController *controller,
                                  int16_t dx_px,
                                  uint32_t now_ms)
{
    float old_position_cm = controller->position_cm;
    float previous_adjacent_velocity = controller->adjacent_velocity_cm_s;
    uint8_t had_last_position = controller->has_last_position;
    uint8_t had_adjacent_velocity = controller->has_adjacent_velocity;
    uint8_t turn_crossing = 0u;
    float turn_position_cm = 0.0f;
    BallEstimate estimate;

    controller->control_now_ms = now_ms;
    controller->last_dx_px = dx_px;
    controller->last_valid_ms = now_ms;
    controller->has_valid_measurement = 1u;

    if ((controller->state == BALL_STATE_WAIT_CENTER) ||
        (controller->state == BALL_STATE_IDLE) ||
        (controller->state == BALL_STATE_ABORTED) ||
        (controller->state == BALL_STATE_VISION_FAULT)) {
        int16_t median_dx_px;
        Ball_CenterWindowPush(controller, dx_px, now_ms);
        Ball_SetServoCenter(controller);
        if (Ball_CenterWindowReady(controller, &median_dx_px) != 0u) {
            controller->center_ready = 1u;
            controller->center_median_dx_px = median_dx_px;
            controller->position_cm = -(float)(dx_px - median_dx_px) /
                                      BALL_PX_PER_CM;
            controller->predicted_position_cm = controller->position_cm;
            controller->damping_predicted_position_cm = controller->position_cm;
            controller->velocity_cm_s = 0.0f;
            if (controller->start_latched != 0u) {
                Ball_StartTaskFromReady(controller, now_ms);
            }
        } else {
            controller->center_ready = 0u;
        }
        Ball_Log(controller, now_ms, 0u);
        return;
    }

    controller->position_cm = -(float)(dx_px - controller->dx_zero_px) /
                              BALL_PX_PER_CM;

    if (had_last_position != 0u) {
        uint32_t dt_ms = (uint32_t)(now_ms - controller->last_position_ms);
        if ((dt_ms >= BALL_ESTIMATOR_MIN_DT_MS) &&
            (dt_ms <= BALL_ESTIMATOR_MAX_DT_MS)) {
            float current_adjacent_velocity =
                (controller->position_cm - old_position_cm) * 1000.0f /
                (float)dt_ms;
            if ((had_adjacent_velocity != 0u) &&
                (previous_adjacent_velocity > 0.0f) &&
                (current_adjacent_velocity <= 0.0f)) {
                float denominator = previous_adjacent_velocity -
                                    current_adjacent_velocity;
                float fraction = (denominator > 0.000001f) ?
                    (previous_adjacent_velocity / denominator) : 1.0f;
                fraction = Ball_Clamp(fraction, 0.0f, 1.0f);
                turn_position_cm = old_position_cm +
                    fraction * (controller->position_cm - old_position_cm);
                turn_crossing = 1u;
            }
            controller->adjacent_velocity_cm_s = current_adjacent_velocity;
            controller->has_adjacent_velocity = 1u;
        } else {
            /* Do not infer an A turning point across a rejected frame gap. */
            controller->adjacent_velocity_cm_s = 0.0f;
            controller->has_adjacent_velocity = 0u;
        }
    }
    controller->last_position_cm = controller->position_cm;
    controller->last_position_ms = now_ms;
    controller->has_last_position = 1u;

    BallEstimator_Add(&controller->estimator,
                      controller->position_cm,
                      now_ms,
                      &estimate);
    controller->velocity_cm_s = estimate.velocity_cm_s;
    controller->predicted_position_cm = estimate.predicted_position_cm;
    controller->damping_predicted_position_cm = controller->position_cm +
        BALL_SETTLE_PREDICT_S * controller->velocity_cm_s;

    if ((controller->state == BALL_STATE_DRIVE_B) ||
        (controller->state == BALL_STATE_B_APPROACH) ||
        (controller->state == BALL_STATE_BRAKE_B)) {
        BallFastEstimate fast_estimate;
        BallFastEstimator_Add(&controller->fast_estimator,
                              controller->position_cm,
                              now_ms,
                              &fast_estimate);
        if (fast_estimate.velocity_valid != 0u) {
            controller->velocity_fast_cm_s = fast_estimate.velocity_cm_s;
            controller->velocity_fast_valid = 1u;
        } else {
            controller->velocity_fast_cm_s = 0.0f;
            controller->velocity_fast_valid = 0u;
            if (fast_estimate.gap_reset != 0u) {
                controller->b_approach_confirm_frames = 0u;
                controller->b_approach_candidate_reason =
                    BALL_APPROACH_REASON_NONE;
            }
        }
    }

    if ((controller->state == BALL_STATE_DRIVE_B) ||
        (controller->state == BALL_STATE_B_APPROACH) ||
        (controller->state == BALL_STATE_BRAKE_B) ||
        (controller->state == BALL_STATE_B_SETTLE) ||
        (controller->state == BALL_STATE_DONE_HOLD) ||
        (controller->state == BALL_STATE_TIMEOUT)) {
        Ball_UpdateBGuardMetrics(controller);
    }

    if ((controller->state == BALL_STATE_B_SETTLE) ||
        (controller->state == BALL_STATE_DONE_HOLD) ||
        (controller->state == BALL_STATE_TIMEOUT)) {
        Ball_UpdateQuietObserver(controller, now_ms);
    }

    {
        float pid_target_cm = ((controller->state == BALL_STATE_DRIVE_A) ||
            (controller->state == BALL_STATE_BRAKE_A) ||
            (controller->state == BALL_STATE_A_CAPTURE)) ?
                BALL_TARGET_A_CM : BALL_TARGET_B_CM;
        uint8_t integral_allowed = 0u;
        if (((controller->state == BALL_STATE_B_SETTLE) ||
             ((controller->state == BALL_STATE_TIMEOUT) &&
              (controller->timeout_settled == 0u))) &&
            (controller->quiet_active == 0u) &&
            (controller->quiet_pulse_sequence_active == 0u) &&
            (controller->quiet_reacquire_wait_frames == 0u) &&
            (controller->quiet_warmup_hold == 0u)) {
            integral_allowed = 1u;
        }
        BallPidBank_Update(&controller->pid_bank,
                           pid_target_cm,
                           controller->position_cm,
                           controller->velocity_cm_s,
                           now_ms,
                           integral_allowed);
    }

    if ((controller->state == BALL_STATE_DRIVE_A) ||
        (controller->state == BALL_STATE_BRAKE_A) ||
        (controller->state == BALL_STATE_A_CAPTURE)) {
        Ball_UpdateAPeak(controller);
    }
    if ((controller->state == BALL_STATE_DRIVE_B) ||
        (controller->state == BALL_STATE_B_APPROACH) ||
        (controller->state == BALL_STATE_BRAKE_B) ||
        (controller->state == BALL_STATE_B_SETTLE) ||
        (controller->state == BALL_STATE_DONE_HOLD) ||
        (controller->state == BALL_STATE_TIMEOUT)) {
        Ball_UpdateBDiagnostics(controller);
    }

    {
        uint32_t elapsed_ms = Ball_TaskElapsed(controller, now_ms);
        Ball_UpdateBudgetFlags(controller, elapsed_ms);
        if ((controller->diagnostics.t_b_band_ms == 0u) &&
            (controller->task_started != 0u) &&
            ((controller->state == BALL_STATE_DRIVE_B) ||
             (controller->state == BALL_STATE_B_APPROACH) ||
             (controller->state == BALL_STATE_BRAKE_B) ||
             (controller->state == BALL_STATE_B_SETTLE) ||
             (controller->state == BALL_STATE_DONE_HOLD) ||
             (controller->state == BALL_STATE_TIMEOUT)) &&
            (Ball_Abs(controller->position_cm - BALL_TARGET_B_CM) <=
             BALL_B_FIRST_BAND_TOL_CM)) {
            controller->diagnostics.t_b_band_ms = elapsed_ms;
            controller->diagnostics.b_band_entry_velocity_cm_s =
                controller->velocity_cm_s;
        }
        if (((controller->timeout_pending != 0u) ||
             (elapsed_ms >= BALL_OFFICIAL_TIMEOUT_MS)) &&
            (controller->diagnostics.t_done_ms == 0u) &&
            (controller->state != BALL_STATE_TIMEOUT) &&
            (controller->state != BALL_STATE_DONE_HOLD)) {
            Ball_EnterTimeout(controller);
        }
    }

    switch (controller->state) {
    case BALL_STATE_DRIVE_A:
    {
        float positive_velocity = (controller->velocity_cm_s > 0.0f) ?
            controller->velocity_cm_s : 0.0f;
        float phase_prediction_cm = controller->position_cm +
            BALL_BRAKE_A_LEAD_S * positive_velocity;

        if (((controller->position_cm - controller->drive_reference_cm) >=
             BALL_DRIVE_MOVE_DETECT_CM) ||
            (controller->velocity_cm_s >= BALL_DRIVE_MOVE_DETECT_SPEED_CM_S)) {
            controller->drive_motion_detected = 1u;
        }

        if (phase_prediction_cm >= BALL_BRAKE_A_TRIGGER_CM) {
            controller->state = BALL_STATE_BRAKE_A;
            controller->stage_start_ms = now_ms;
            controller->diagnostics.a_brake_start_cm = controller->position_cm;
            Ball_SetServoPhaseOffset(controller, BALL_BRAKE_A_OFFSET_DEG);
        } else if ((controller->drive_motion_detected == 0u) &&
                   ((uint32_t)(now_ms - controller->stage_start_ms) >=
                    BALL_DRIVE_A_BOOST_DELAY_MS)) {
            Ball_SetServoPhaseOffset(controller, BALL_DRIVE_A_BOOST_OFFSET_DEG);
        } else {
            Ball_SetServoPhaseOffset(controller, BALL_DRIVE_A_OFFSET_DEG);
        }
        break;
    }

    case BALL_STATE_BRAKE_A:
        Ball_SetServoPhaseOffset(controller, BALL_BRAKE_A_OFFSET_DEG);
        if (turn_crossing != 0u) {
            Ball_HandleATurn(controller, turn_position_cm, now_ms,
                             BALL_TRANSITION_A_INTERPOLATED);
        } else if (Ball_AReverseFallbackReady(controller) != 0u) {
            Ball_HandleATurn(controller, controller->a_peak_cm, now_ms,
                             BALL_TRANSITION_A_PEAK_FALLBACK);
        } else if ((controller->velocity_cm_s <= BALL_A_CAPTURE_ENTER_SPEED_CM_S) ||
                   (controller->adjacent_velocity_cm_s <=
                    BALL_A_CAPTURE_ENTER_SPEED_CM_S)) {
            controller->state = BALL_STATE_A_CAPTURE;
            controller->stage_start_ms = now_ms;
        }
        break;

    case BALL_STATE_A_CAPTURE:
        if (controller->a_pulse_active != 0u) {
            if ((uint32_t)(now_ms - controller->a_pulse_start_ms) <
                BALL_A_EARLY_PULSE_MS) {
                Ball_SetServoAutomaticOffset(controller,
                    BALL_A_EARLY_PULSE_OFFSET_DEG);
            } else {
                controller->a_pulse_active = 0u;
                Ball_SetServoPhaseOffset(controller, BALL_BRAKE_A_OFFSET_DEG);
            }
            break;
        }
        if (turn_crossing != 0u) {
            Ball_HandleATurn(controller, turn_position_cm, now_ms,
                             BALL_TRANSITION_A_INTERPOLATED);
        } else if (Ball_AReverseFallbackReady(controller) != 0u) {
            Ball_HandleATurn(controller, controller->a_peak_cm, now_ms,
                             BALL_TRANSITION_A_PEAK_FALLBACK);
        } else if ((controller->a_overrun_recovery != 0u) &&
                   (controller->position_cm <= BALL_A_TURN_MAX_CM) &&
                   (controller->velocity_cm_s <= 0.0f)) {
            controller->last_transition_reason =
                BALL_TRANSITION_A_OVERRUN_RECOVERY;
            controller->diagnostics.a_transition_reason =
                (uint8_t)BALL_TRANSITION_A_OVERRUN_RECOVERY;
            Ball_EnterDriveB(controller, now_ms);
        } else if (controller->position_cm > BALL_A_TURN_MAX_CM) {
            controller->a_overrun_recovery = 1u;
            Ball_SetServoPhaseOffset(controller, BALL_BRAKE_A_OFFSET_DEG);
        } else if ((controller->position_cm < BALL_A_TURN_MIN_CM) &&
                   (Ball_Abs(controller->velocity_cm_s) <=
                    BALL_DONE_SPEED_TOL_CM_S)) {
            Ball_StartAEarlyPulse(controller, now_ms);
        } else if ((uint32_t)(now_ms - controller->stage_start_ms) >=
                   BALL_A_CAPTURE_MAX_MS) {
            uint32_t elapsed_ms = Ball_TaskElapsed(controller, now_ms);
            controller->diagnostics.a_turn_cm = controller->a_peak_cm;
            controller->diagnostics.a_transition_reason =
                (uint8_t)BALL_TRANSITION_A_CAPTURE_TIMEOUT;
            controller->diagnostics.a_capture_timed_out = 1u;
            controller->last_transition_reason =
                BALL_TRANSITION_A_CAPTURE_TIMEOUT;
            if ((controller->a_peak_cm < BALL_A_TURN_MIN_CM) ||
                (controller->a_peak_cm > BALL_A_TURN_MAX_CM)) {
                controller->result_flags |= BALL_RESULT_A_RANGE;
            }
            if (controller->diagnostics.t_a_ms == 0u) {
                controller->diagnostics.t_a_ms = elapsed_ms;
            }
            Ball_EnterDriveB(controller, now_ms);
        } else {
            Ball_SetServoPhaseOffset(controller, BALL_BRAKE_A_OFFSET_DEG);
        }
        break;

    case BALL_STATE_DRIVE_B:
    {
        BallApproachReason approach_reason =
            Ball_GetApproachReason(controller);

        if (((controller->drive_reference_cm - controller->position_cm) >=
             BALL_DRIVE_MOVE_DETECT_CM) ||
            (controller->velocity_cm_s <= -BALL_DRIVE_MOVE_DETECT_SPEED_CM_S)) {
            controller->drive_motion_detected = 1u;
        }

        if (approach_reason == BALL_APPROACH_REASON_NONE) {
            controller->b_approach_confirm_frames = 0u;
            controller->b_approach_candidate_reason =
                BALL_APPROACH_REASON_NONE;
        } else if (controller->b_approach_candidate_reason ==
                   (uint8_t)approach_reason) {
            if (controller->b_approach_confirm_frames < 255u) {
                controller->b_approach_confirm_frames++;
            }
        } else {
            controller->b_approach_candidate_reason = (uint8_t)approach_reason;
            controller->b_approach_confirm_frames = 1u;
        }

        if (controller->b_approach_confirm_frames >=
            BALL_B_APPROACH_CONFIRM_FRAMES) {
            Ball_EnterBApproach(controller, now_ms, approach_reason);
            /* Preserve an explicit BAPR record even if this same measurement
             * also satisfies the non-deferrable 113 degree brake predicate. */
            Ball_Log(controller, now_ms, 0u);
            Ball_ControlBApproach(controller, now_ms);
        } else if ((controller->drive_motion_detected == 0u) &&
                   ((uint32_t)(now_ms - controller->stage_start_ms) >=
                    BALL_DRIVE_B_BOOST_DELAY_MS)) {
            Ball_SetServoPhaseOffset(controller, BALL_DRIVE_B_BOOST_OFFSET_DEG);
        } else {
            Ball_SetServoPhaseOffset(controller, BALL_DRIVE_B_OFFSET_DEG);
        }
        break;
    }

    case BALL_STATE_B_APPROACH:
        Ball_ControlBApproach(controller, now_ms);
        break;

    case BALL_STATE_BRAKE_B:
    {
        uint8_t rebound_now;
        uint8_t fast_release_now;
        uint8_t slow_release_now;
        BallTransitionReason reason = BALL_TRANSITION_NONE;

        /* No PID trim may reduce the physical +28 degree brake. */
        Ball_SetServoAutomaticOffset(controller, BALL_BRAKE_B_OFFSET_DEG);
        rebound_now = (((controller->position_cm - controller->b_min_cm) >=
                        BALL_BRAKE_B_REBOUND_CM) &&
                       (controller->has_adjacent_velocity != 0u) &&
                       (controller->adjacent_velocity_cm_s > 0.0f)) ? 1u : 0u;
        fast_release_now = ((controller->velocity_fast_valid != 0u) &&
            (controller->velocity_fast_cm_s >=
             -BALL_B_FAST_RELEASE_SPEED_CM_S)) ? 1u : 0u;
        slow_release_now = (controller->velocity_cm_s >=
            -BALL_BRAKE_B_RELEASE_SPEED_CM_S) ? 1u : 0u;

        if (rebound_now != 0u) {
            if (controller->b_rebound_confirm_frames < 255u) {
                controller->b_rebound_confirm_frames++;
            }
        } else {
            controller->b_rebound_confirm_frames = 0u;
        }

        if ((rebound_now != 0u) || (fast_release_now != 0u) ||
            (slow_release_now != 0u)) {
            if (controller->b_brake_release_confirm_frames < 255u) {
                controller->b_brake_release_confirm_frames++;
            }
            if (rebound_now != 0u) {
                reason = BALL_TRANSITION_B_REBOUND;
            } else if (fast_release_now != 0u) {
                reason = BALL_TRANSITION_B_FAST_SPEED;
            } else {
                reason = BALL_TRANSITION_B_FILTERED_SPEED;
            }
        } else {
            controller->b_brake_release_confirm_frames = 0u;
        }

        if (controller->b_brake_release_confirm_frames >=
            BALL_B_BRAKE_RELEASE_CONFIRM_FRAMES) {
            controller->state = BALL_STATE_B_SETTLE;
            controller->stage_start_ms = now_ms;
            controller->diagnostics.t_b_settle_ms =
                Ball_TaskElapsed(controller, now_ms);
            controller->diagnostics.b_release_reason = (uint8_t)reason;
            controller->last_transition_reason = reason;
            if ((reason == BALL_TRANSITION_B_REBOUND) ||
                (controller->b_min_cm < BALL_TARGET_B_CM)) {
                Ball_LatchFirstBOvershoot(controller);
            }
            Ball_ResetPulse(controller);
            controller->b_settle_brake_guard_active = 1u;
            controller->settle_pulse_ready_frames = 0u;
            controller->velocity_fast_valid = 0u;
            controller->last_transition_reason = reason;
            if (Ball_Abs(controller->velocity_cm_s) >=
                BALL_SETTLE_MOVE_ENTER_CM_S) {
                controller->settle_moving = 1u;
            }
            Ball_ControlBSettle(controller, now_ms, 0u);
        }
        break;
    }

    case BALL_STATE_B_SETTLE:
        Ball_ControlBSettle(controller, now_ms, 0u);
        break;

    case BALL_STATE_DONE_HOLD:
    {
        float measured_error_cm =
            BALL_TARGET_B_CM - controller->position_cm;
        float predicted_error_cm =
            BALL_TARGET_B_CM -
            controller->damping_predicted_position_cm;
        uint8_t predicted_crossing = Ball_PredictedTargetCrossing(
            measured_error_cm, predicted_error_cm,
            controller->velocity_cm_s);
        BallQuietControlResult quiet_result =
            Ball_ControlQuiet(controller, now_ms,
                              predicted_crossing, 0u);
        if ((quiet_result == BALL_QUIET_CONTROL_EXIT) ||
            (quiet_result == BALL_QUIET_CONTROL_SAFETY_BRAKE)) {
            controller->state = BALL_STATE_B_SETTLE;
            controller->stage_start_ms = now_ms;
            if (quiet_result == BALL_QUIET_CONTROL_SAFETY_BRAKE) {
                Ball_ApplyVelocityBrake(
                    controller, Ball_Abs(controller->velocity_cm_s));
            } else {
                controller->settle_mode =
                    BALL_SETTLE_QUIET_RECAPTURE;
                Ball_SetServoCenter(controller);
            }
        } else {
            Ball_SetServoCenter(controller);
        }
        break;
    }

    case BALL_STATE_TIMEOUT:
        if (controller->timeout_settled != 0u) {
            float measured_error_cm =
                BALL_TARGET_B_CM - controller->position_cm;
            float predicted_error_cm =
                BALL_TARGET_B_CM -
                controller->damping_predicted_position_cm;
            uint8_t predicted_crossing = Ball_PredictedTargetCrossing(
                measured_error_cm, predicted_error_cm,
                controller->velocity_cm_s);
            BallQuietControlResult quiet_result =
                Ball_ControlQuiet(controller, now_ms,
                                  predicted_crossing, 0u);
            if ((quiet_result == BALL_QUIET_CONTROL_EXIT) ||
                (quiet_result == BALL_QUIET_CONTROL_SAFETY_BRAKE)) {
                controller->timeout_settled = 0u;
                controller->stage_start_ms = now_ms;
                if (quiet_result == BALL_QUIET_CONTROL_SAFETY_BRAKE) {
                    Ball_ApplyVelocityBrake(
                        controller,
                        Ball_Abs(controller->velocity_cm_s));
                } else {
                    controller->settle_mode =
                        BALL_SETTLE_QUIET_RECAPTURE;
                    Ball_SetServoCenter(controller);
                }
            } else {
                Ball_SetServoCenter(controller);
            }
        } else {
            Ball_ControlBSettle(controller, now_ms, 1u);
        }
        break;

    case BALL_STATE_IDLE:
    case BALL_STATE_WAIT_CENTER:
    case BALL_STATE_VISION_FAULT:
    case BALL_STATE_ABORTED:
    default:
        Ball_SetServoCenter(controller);
        break;
    }

    Ball_Log(controller, now_ms, 0u);
}

uint8_t BallController_IsActive(const BallController *controller)
{
    switch (controller->state) {
    case BALL_STATE_WAIT_CENTER:
    case BALL_STATE_DRIVE_A:
    case BALL_STATE_BRAKE_A:
    case BALL_STATE_A_CAPTURE:
    case BALL_STATE_DRIVE_B:
    case BALL_STATE_B_APPROACH:
    case BALL_STATE_BRAKE_B:
    case BALL_STATE_B_SETTLE:
    case BALL_STATE_DONE_HOLD:
    case BALL_STATE_TIMEOUT:
        return 1u;
    default:
        return 0u;
    }
}

uint8_t BallController_IsReady(const BallController *controller)
{
    return controller->center_ready;
}

uint8_t BallController_IsArmed(const BallController *controller)
{
    return controller->start_latched;
}

uint8_t BallController_VisionWatchActive(const BallController *controller)
{
    switch (controller->state) {
    case BALL_STATE_DRIVE_A:
    case BALL_STATE_BRAKE_A:
    case BALL_STATE_A_CAPTURE:
    case BALL_STATE_DRIVE_B:
    case BALL_STATE_B_APPROACH:
    case BALL_STATE_BRAKE_B:
    case BALL_STATE_B_SETTLE:
    case BALL_STATE_DONE_HOLD:
    case BALL_STATE_TIMEOUT:
        return 1u;
    default:
        return 0u;
    }
}

uint8_t BallController_TaskTimerActive(const BallController *controller)
{
    if ((controller->task_started == 0u) ||
        (controller->timeout_pending != 0u) ||
        (controller->diagnostics.t_done_ms != 0u) ||
        (controller->state == BALL_STATE_DONE_HOLD) ||
        (controller->state == BALL_STATE_TIMEOUT) ||
        (controller->state == BALL_STATE_VISION_FAULT) ||
        (controller->state == BALL_STATE_ABORTED)) {
        return 0u;
    }
    return 1u;
}

uint32_t BallController_TaskElapsedMs(const BallController *controller,
                                      uint32_t now_ms)
{
    return Ball_TaskElapsed(controller, now_ms);
}

uint32_t BallController_DataAgeMs(const BallController *controller,
                                  uint32_t now_ms)
{
    uint32_t reference_ms = controller->has_valid_measurement ?
        controller->last_valid_ms : controller->wait_start_ms;
    return (uint32_t)(now_ms - reference_ms);
}

const char *BallController_StateShort(BallState state)
{
    switch (state) {
    case BALL_STATE_IDLE:         return "IDLE";
    case BALL_STATE_WAIT_CENTER:  return "WAIT";
    case BALL_STATE_DRIVE_A:      return "DRVA";
    case BALL_STATE_BRAKE_A:      return "BRKA";
    case BALL_STATE_A_CAPTURE:    return "ACAP";
    case BALL_STATE_DRIVE_B:      return "DRVB";
    case BALL_STATE_B_APPROACH:   return "BAPR";
    case BALL_STATE_BRAKE_B:      return "BRKB";
    case BALL_STATE_B_SETTLE:     return "BSET";
    case BALL_STATE_DONE_HOLD:    return "DONE";
    case BALL_STATE_VISION_FAULT: return "VISN";
    case BALL_STATE_ABORTED:      return "ABRT";
    case BALL_STATE_TIMEOUT:      return "TOUT";
    default:                      return "UNKN";
    }
}

const char *BallController_SettleModeShort(BallSettleMode mode)
{
    switch (mode) {
    case BALL_SETTLE_DAMP:            return "BDMP";
    case BALL_SETTLE_PULSE:           return "BPUL";
    case BALL_SETTLE_OBSERVE:         return "BOBS";
    case BALL_SETTLE_HOLD:            return "BHLD";
    case BALL_SETTLE_QUIET:           return "QHLD";
    case BALL_SETTLE_QUIET_PULSE:     return "QPUL";
    case BALL_SETTLE_QUIET_OBSERVE:   return "QOBS";
    case BALL_SETTLE_QUIET_RECAPTURE: return "QREC";
    case BALL_SETTLE_IDLE:
    default:                          return "BIDL";
    }
}

const char *BallController_TransitionReasonShort(BallTransitionReason reason)
{
    switch (reason) {
    case BALL_TRANSITION_A_INTERPOLATED:       return "AINT";
    case BALL_TRANSITION_A_PEAK_FALLBACK:      return "APEK";
    case BALL_TRANSITION_A_CAPTURE_TIMEOUT:    return "ATMO";
    case BALL_TRANSITION_A_OVERRUN_RECOVERY:   return "AOVR";
    case BALL_TRANSITION_B_FILTERED_SPEED:     return "BVEL";
    case BALL_TRANSITION_B_REBOUND:            return "BRBD";
    case BALL_TRANSITION_B_FAST_SPEED:         return "BFST";
    case BALL_TRANSITION_NONE:
    default:                                   return "NONE";
    }
}

const char *BallController_ApproachReasonShort(BallApproachReason reason)
{
    switch (reason) {
    case BALL_APPROACH_REASON_STOP_MARGIN: return "DST";
    case BALL_APPROACH_REASON_HIGH_SPEED:  return "SPD";
    case BALL_APPROACH_REASON_PHASE:       return "PHS";
    case BALL_APPROACH_REASON_NONE:
    default:                               return "---";
    }
}

const char *BallController_BrakeReasonShort(BallBrakeReason reason)
{
    switch (reason) {
    case BALL_BRAKE_REASON_STOP_DISTANCE: return "DST";
    case BALL_BRAKE_REASON_PHASE:         return "PHS";
    case BALL_BRAKE_REASON_HARD_GUARD:    return "GRD";
    case BALL_BRAKE_REASON_NONE:
    default:                              return "---";
    }
}

const char *BallController_ResultShort(const BallController *controller)
{
    if ((controller->result_flags & BALL_RESULT_ABORT) != 0u) {
        return "ABORT";
    }
    if ((controller->result_flags & BALL_RESULT_VISION) != 0u) {
        return "VISION";
    }
    if ((controller->result_flags & BALL_RESULT_TIMEOUT) != 0u) {
        return "TIMEOUT";
    }
    if ((controller->result_flags & BALL_RESULT_PASS) != 0u) {
        return "PASS";
    }
    if ((controller->result_flags & BALL_RESULT_RUN) != 0u) {
        return "RUN";
    }
    return "IDLE";
}

uint16_t BallController_LogCount(const BallController *controller)
{
    return controller->log_count;
}

uint8_t BallController_GetLog(const BallController *controller,
                              uint16_t chronological_index,
                              BallLogRecord *record)
{
    uint16_t oldest;
    uint16_t slot;
    if (chronological_index >= controller->log_count) {
        return 0u;
    }
    oldest = (uint16_t)((controller->log_head + BALL_LOG_CAPACITY -
                        controller->log_count) % BALL_LOG_CAPACITY);
    slot = (uint16_t)((oldest + chronological_index) % BALL_LOG_CAPACITY);
    *record = controller->logs[slot];
    return 1u;
}

void BallController_SetLogFrozen(BallController *controller, uint8_t frozen)
{
    controller->log_frozen = (frozen != 0u) ? 1u : 0u;
}
