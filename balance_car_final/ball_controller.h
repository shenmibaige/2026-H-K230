#ifndef BALL_CONTROLLER_H
#define BALL_CONTROLLER_H

#include <stdint.h>

#include "ball_config.h"
#include "ball_estimator.h"
#include "ball_fast_estimator.h"
#include "ball_pid_bank.h"
#include "ball_quiet_observer.h"

typedef enum {
    BALL_STATE_IDLE = 0,
    BALL_STATE_WAIT_CENTER,
    BALL_STATE_DRIVE_A,
    BALL_STATE_BRAKE_A,
    BALL_STATE_A_CAPTURE,
    BALL_STATE_DRIVE_B,
    BALL_STATE_BRAKE_B,
    BALL_STATE_B_SETTLE,
    BALL_STATE_DONE_HOLD,
    BALL_STATE_VISION_FAULT,
    BALL_STATE_ABORTED,
    BALL_STATE_TIMEOUT,
    /* Appended to preserve the established v3.1 CSV state IDs 0..11. */
    BALL_STATE_B_APPROACH = 12
} BallState;

typedef enum {
    BALL_APPROACH_REASON_NONE = 0,
    BALL_APPROACH_REASON_STOP_MARGIN,
    BALL_APPROACH_REASON_HIGH_SPEED,
    BALL_APPROACH_REASON_PHASE
} BallApproachReason;

typedef enum {
    BALL_BRAKE_REASON_NONE = 0,
    BALL_BRAKE_REASON_STOP_DISTANCE,
    BALL_BRAKE_REASON_PHASE,
    BALL_BRAKE_REASON_HARD_GUARD
} BallBrakeReason;

typedef enum {
    BALL_APPROACH_COMMAND_COAST = 0,
    BALL_APPROACH_COMMAND_PREBRAKE,
    BALL_APPROACH_COMMAND_REDRIVE
} BallApproachCommand;

typedef enum {
    BALL_PULSE_IDLE = 0,
    BALL_PULSE_ACTIVE,
    BALL_PULSE_OBSERVE
} BallPulsePhase;

typedef enum {
    BALL_SETTLE_IDLE = 0,
    BALL_SETTLE_DAMP,
    BALL_SETTLE_PULSE,
    BALL_SETTLE_OBSERVE,
    BALL_SETTLE_HOLD,
    /* Appended so all v3.2 settle-mode values remain stable in K3 logs. */
    BALL_SETTLE_QUIET,
    BALL_SETTLE_QUIET_PULSE,
    BALL_SETTLE_QUIET_OBSERVE,
    BALL_SETTLE_QUIET_RECAPTURE
} BallSettleMode;

typedef enum {
    BALL_QUIET_EXIT_NONE = 0,
    BALL_QUIET_EXIT_POSITION,
    BALL_QUIET_EXIT_SPEED,
    BALL_QUIET_EXIT_HARD_ERROR,
    BALL_QUIET_EXIT_OUTWARD_PREDICTION,
    BALL_QUIET_EXIT_LARGE_ERROR
} BallQuietExitReason;

typedef enum {
    BALL_TRANSITION_NONE = 0,
    BALL_TRANSITION_A_INTERPOLATED,
    BALL_TRANSITION_A_PEAK_FALLBACK,
    BALL_TRANSITION_A_CAPTURE_TIMEOUT,
    BALL_TRANSITION_A_OVERRUN_RECOVERY,
    BALL_TRANSITION_B_FILTERED_SPEED,
    BALL_TRANSITION_B_REBOUND,
    BALL_TRANSITION_B_FAST_SPEED
} BallTransitionReason;

enum {
    BALL_RESULT_RUN             = 0x0001u,
    BALL_RESULT_PASS            = 0x0002u,
    BALL_RESULT_TIMEOUT         = 0x0004u,
    BALL_RESULT_VISION          = 0x0008u,
    BALL_RESULT_ABORT           = 0x0010u,
    BALL_RESULT_A_LATE          = 0x0020u,
    BALL_RESULT_B_BRAKE_LATE    = 0x0040u,
    BALL_RESULT_B_BAND_LATE     = 0x0080u,
    BALL_RESULT_DONE_LATE       = 0x0100u,
    BALL_RESULT_A_RANGE         = 0x0200u,
    BALL_RESULT_B_APPROACH_LATE = 0x0400u
};

typedef struct {
    uint32_t time_ms;
    int16_t dx_px;
    int16_t x100;
    int16_t v100;
    int16_t xpred100;
    uint16_t servo10;
    uint16_t data_age;
    uint16_t result_flags;
    uint8_t state;
    uint8_t detail;
    int8_t pid_output2[BALL_PID_COUNT];
    uint8_t pid_weight255[BALL_PID_COUNT];
    int16_t pid_fused10;
    uint8_t pid_update_mask;
    uint8_t pid_dominant;
    int16_t vfast100;
    int16_t vguard100;
    int16_t xguard100;
    int16_t drem100;
    int16_t dstop100;
    int16_t vlimit100;
    uint8_t approach_reason;
    uint8_t brake_reason;
    int16_t vB_enter100;
    int16_t quiet_error100;
    uint8_t quiet_flags;
    uint8_t servo_event_count;
} BallLogRecord;

typedef struct {
    uint32_t t_a_ms;
    uint32_t t_b_approach_ms;
    uint32_t t_b_brake_ms;
    uint32_t t_b_enter_ms;
    uint32_t t_b_band_ms;
    uint32_t t_b_settle_ms;
    uint32_t t_done_ms;
    float a_turn_cm;
    float a_brake_start_cm;
    float b_brake_start_cm;
    float b_min_cm;
    float b_first_overshoot_cm;
    float b_band_entry_velocity_cm_s;
    uint8_t b_cross_count;
    uint8_t a_transition_reason;
    uint8_t b_release_reason;
    uint8_t a_capture_timed_out;
    uint8_t b_approach_reason;
    uint8_t b_brake_reason;
    uint32_t t_quiet_enter_ms;
    uint16_t quiet_exit_count;
    uint16_t quiet_pulse_count;
    uint16_t servo_event_count;
    uint8_t quiet_exit_reason;
    int16_t max_quiet_error100;
    uint8_t max_servo_events_in_1s;
    BallState timeout_phase;
} BallDiagnostics;

typedef struct {
    BallState state;
    uint16_t result_flags;
    float servo_command_deg;

    int16_t last_dx_px;
    int16_t dx_zero_px;
    float position_cm;
    float velocity_cm_s;
    float velocity_fast_cm_s;
    float velocity_guard_cm_s;
    float predicted_position_cm;
    float damping_predicted_position_cm;
    float x_guard_cm;
    float d_remaining_cm;
    float d_stop_cm;
    float v_limit_cm_s;
    float adjacent_velocity_cm_s;
    float u_raw_deg;

    uint32_t wait_start_ms;
    uint32_t task_start_ms;
    uint32_t stage_start_ms;
    uint32_t last_valid_ms;
    uint32_t last_position_ms;
    float last_position_cm;
    uint8_t has_valid_measurement;
    uint8_t has_last_position;
    uint8_t has_adjacent_velocity;
    uint8_t task_started;
    uint8_t start_latched;
    uint8_t timeout_pending;
    uint8_t timeout_settled;

    int16_t center_dx_px[BALL_CENTER_SAMPLE_COUNT];
    uint32_t center_time_ms[BALL_CENTER_SAMPLE_COUNT];
    uint8_t center_count;
    uint8_t center_ready;
    int16_t center_median_dx_px;

    BallEstimator estimator;
    BallFastEstimator fast_estimator;
    BallPidBank pid_bank;
    BallQuietObserver quiet_observer;

    float drive_reference_cm;
    uint8_t drive_motion_detected;
    uint8_t a_pulse_active;
    uint8_t a_overrun_recovery;
    uint32_t a_pulse_start_ms;
    float a_peak_cm;
    uint8_t a_peak_valid;
    uint8_t a_reverse_confirm_frames;

    float b_min_cm;
    int8_t b_last_side;
    uint8_t b_side_valid;
    uint8_t b_rebound_confirm_frames;
    uint8_t b_brake_release_confirm_frames;
    uint8_t b_first_overshoot_latched;
    uint8_t velocity_fast_valid;
    uint8_t b_approach_confirm_frames;
    uint8_t b_approach_candidate_reason;
    uint8_t b_approach_command;
    uint8_t b_approach_command_valid;
    uint8_t b_settle_brake_guard_active;
    uint8_t settle_pulse_ready_frames;
    uint32_t b_approach_command_start_ms;

    BallPulsePhase pulse_phase;
    BallSettleMode settle_mode;
    uint32_t pulse_phase_start_ms;
    float pulse_start_cm;
    int8_t pulse_direction;
    uint8_t pulse_level;
    uint8_t pulse_no_move_count;

    uint8_t settle_hold_active;
    uint32_t settle_hold_start_ms;
    uint8_t settle_moving;
    uint8_t settle_still_frames;
    int8_t settle_fast_sign;
    uint8_t settle_fast_sign_frames;
    BallTransitionReason last_transition_reason;

    float quiet_position_cm;
    float quiet_error_cm;
    float quiet_spread_cm;
    float quiet_anchor_cm;
    uint32_t quiet_position_confirm_start_ms;
    uint32_t quiet_bias_start_ms;
    uint32_t quiet_last_pulse_end_ms;
    uint32_t quiet_event_window_start_ms;
    uint32_t control_now_ms;
    uint8_t quiet_estimate_valid;
    uint8_t quiet_positive_error_count;
    uint8_t quiet_negative_error_count;
    uint8_t quiet_active;
    uint8_t quiet_enter_frames;
    uint8_t quiet_position_confirm_active;
    uint8_t quiet_speed_exit_frames;
    uint8_t quiet_hard_error_frames;
    uint8_t quiet_bias_active;
    uint8_t quiet_reacquire_wait_frames;
    uint8_t quiet_warmup_hold;
    uint8_t quiet_pulse_sequence_active;
    uint8_t quiet_has_last_pulse_end;
    uint8_t quiet_event_window_active;
    uint8_t quiet_event_window_count;

    BallDiagnostics diagnostics;

    BallLogRecord logs[BALL_LOG_CAPACITY];
    uint16_t log_head;
    uint16_t log_count;
    uint8_t log_frozen;
    uint8_t has_log_time;
    BallState last_logged_state;
    uint32_t last_log_ms;
} BallController;

void BallController_Init(BallController *controller, uint32_t now_ms);
void BallController_RequestStart(BallController *controller, uint32_t now_ms);
void BallController_RequestAbort(BallController *controller, uint32_t now_ms);
void BallController_RequestTimeout(BallController *controller, uint32_t now_ms);
void BallController_OnVisionFault(BallController *controller,
                                  uint32_t now_ms,
                                  uint16_t data_age_ms);
void BallController_OnMeasurement(BallController *controller,
                                  int16_t dx_px,
                                  uint32_t now_ms);

uint8_t BallController_IsActive(const BallController *controller);
uint8_t BallController_IsReady(const BallController *controller);
uint8_t BallController_IsArmed(const BallController *controller);
uint8_t BallController_VisionWatchActive(const BallController *controller);
uint8_t BallController_TaskTimerActive(const BallController *controller);
uint32_t BallController_TaskElapsedMs(const BallController *controller,
                                      uint32_t now_ms);
uint32_t BallController_DataAgeMs(const BallController *controller,
                                  uint32_t now_ms);
const char *BallController_StateShort(BallState state);
const char *BallController_SettleModeShort(BallSettleMode mode);
const char *BallController_TransitionReasonShort(BallTransitionReason reason);
const char *BallController_ApproachReasonShort(BallApproachReason reason);
const char *BallController_BrakeReasonShort(BallBrakeReason reason);
const char *BallController_ResultShort(const BallController *controller);

uint16_t BallController_LogCount(const BallController *controller);
uint8_t BallController_GetLog(const BallController *controller,
                              uint16_t chronological_index,
                              BallLogRecord *record);
void BallController_SetLogFrozen(BallController *controller, uint8_t frozen);

#endif
