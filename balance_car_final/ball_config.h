#ifndef BALL_CONFIG_H
#define BALL_CONFIG_H

/*
 * H Q3 v3.3 quiet-hold parameters layered on the frozen v3.2 front end.
 * Units are part of each name: degrees, pixels, centimetres, seconds or ms.
 * Tune one parameter at a time and record the corresponding physical run.
 */

#define BALL_PX_PER_CM                         45.0f
#define BALL_IMAGE_CENTER_DX_PX                 0
#define BALL_LOST_DX_PX                      (-999)

#define BALL_SERVO_CENTER_DEG                  85.0f
#define BALL_SERVO_AUTO_MIN_DEG                50.0f
#define BALL_SERVO_AUTO_MAX_DEG               120.0f
#define BALL_SERVO_HARD_MIN_DEG                40.0f
#define BALL_SERVO_HARD_MAX_DEG               120.0f

#define BALL_CENTER_SAMPLE_COUNT                 31u
#define BALL_CENTER_MEDIAN_LIMIT_PX              30
#define BALL_CENTER_WINDOW_MIN_MS               500u
#define BALL_CENTER_MAX_SWING_CM                0.50f
#define BALL_CENTER_MAX_SPEED_CM_S              0.50f

#define BALL_ESTIMATOR_SAMPLE_COUNT                5u
#define BALL_ESTIMATOR_MIN_DT_MS                   8u
#define BALL_ESTIMATOR_MAX_DT_MS                  60u
#define BALL_ESTIMATOR_SPEED_LIMIT_CM_S          30.0f
#define BALL_ESTIMATOR_ALPHA                      0.40f
#define BALL_PREDICT_DEFAULT_S                    0.10f

#define BALL_FAST_ESTIMATOR_SAMPLE_COUNT              3u
#define BALL_FAST_ESTIMATOR_MIN_DT_MS                  8u
#define BALL_FAST_ESTIMATOR_MAX_DT_MS                 60u
#define BALL_FAST_ESTIMATOR_SPEED_LIMIT_CM_S         30.0f
#define BALL_FAST_ESTIMATOR_ALPHA                     0.65f

#define BALL_PID_COUNT                                6u
#define BALL_TARGET_A_CM                           5.0f
#define BALL_PID_PHASE_TRIM_MAX_DEG                5.0f
#define BALL_PID_FUSED_MAX_DEG                    35.0f
#define BALL_PID_INTEGRAL_ERROR_MAX_CM             1.0f
#define BALL_PID_INTEGRAL_SPEED_MAX_CM_S           0.35f
#define BALL_PID_INTEGRAL_OUTPUT_MAX_DEG            4.0f

#define BALL_VISION_TIMEOUT_MS                     80u

#define BALL_DRIVE_A_OFFSET_DEG                   30.0f
#define BALL_DRIVE_A_BOOST_OFFSET_DEG             35.0f
#define BALL_DRIVE_A_BOOST_DELAY_MS               160u
#define BALL_DRIVE_MOVE_DETECT_CM                 0.20f
#define BALL_DRIVE_MOVE_DETECT_SPEED_CM_S         0.50f

#define BALL_BRAKE_A_LEAD_S                        0.22f
#define BALL_BRAKE_A_TRIGGER_CM                    4.45f
#define BALL_BRAKE_A_OFFSET_DEG                  (-28.0f)
#define BALL_A_CAPTURE_ENTER_SPEED_CM_S            2.00f
#define BALL_A_TURN_MIN_CM                         4.20f
#define BALL_A_TURN_MAX_CM                         5.80f
#define BALL_A_EARLY_PULSE_OFFSET_DEG             20.0f
#define BALL_A_EARLY_PULSE_MS                      80u
#define BALL_A_REVERSE_DROP_CM                      0.08f
#define BALL_A_REVERSE_SPEED_CM_S                   0.50f
#define BALL_A_REVERSE_CONFIRM_FRAMES                 2u
#define BALL_A_CAPTURE_MAX_MS                       300u

#define BALL_DRIVE_B_OFFSET_DEG                  (-30.0f)
#define BALL_DRIVE_B_BOOST_OFFSET_DEG            (-35.0f)
#define BALL_DRIVE_B_BOOST_DELAY_MS               160u

/* BALL_BRAKE_B_LEAD_S is retained as the frozen v3.1 comparison value. */
#define BALL_BRAKE_B_LEAD_S                        0.26f
#define BALL_BRAKE_B_TRIGGER_CM                  (-4.35f)
#define BALL_BRAKE_B_OFFSET_DEG                   28.0f
#define BALL_BRAKE_B_RELEASE_SPEED_CM_S            0.50f
#define BALL_BRAKE_B_REBOUND_CM                     0.06f
#define BALL_BRAKE_B_REBOUND_CONFIRM_FRAMES          2u
#define BALL_B_CROSS_HYST_CM                        0.15f

#define BALL_B_GUARD_POSITION_LEAD_S                0.06f
#define BALL_B_APPROACH_PHASE_LEAD_S                0.28f
#define BALL_B_APPROACH_MARGIN_CM                    0.70f
#define BALL_B_APPROACH_CONFIRM_FRAMES                  2u
#define BALL_B_APPROACH_HIGH_SPEED_CM_S             11.0f
#define BALL_B_APPROACH_PREBRAKE_OFFSET_DEG          22.0f
#define BALL_B_APPROACH_REDRIVE_OFFSET_DEG          (-22.0f)
#define BALL_B_APPROACH_COMMAND_HOLD_MS                40u
#define BALL_B_APPROACH_ENVELOPE_BAND_CM_S            0.80f
#define BALL_B_APPROACH_REDRIVE_MARGIN_CM_S            1.00f
#define BALL_B_APPROACH_REDRIVE_MIN_DISTANCE_CM        1.40f
#define BALL_B_STOP_DELAY_S                            0.10f
#define BALL_B_STOP_DECEL_CM_S2                       30.0f
#define BALL_B_STOP_MARGIN_CM                          0.30f
#define BALL_B_ENVELOPE_DECEL_CM_S2                   24.0f
#define BALL_B_ENVELOPE_MARGIN_CM                      0.30f
#define BALL_B_ENVELOPE_MIN_CM_S                       2.0f
#define BALL_B_ENVELOPE_MAX_CM_S                      11.0f
#define BALL_B_HARD_BRAKE_X_GUARD_CM                 (-3.80f)
#define BALL_B_HARD_BRAKE_MIN_SPEED_CM_S               4.0f
#define BALL_B_FAST_RELEASE_SPEED_CM_S                  0.80f
#define BALL_B_BRAKE_RELEASE_CONFIRM_FRAMES                2u
#define BALL_B_SETTLE_POST_BRAKE_GUARD_MS                120u
#define BALL_B_SETTLE_RECOVERY_MARGIN_CM                  0.42f

/* v3.3 B-point quiet observer and hysteresis.  These parameters affect only
 * B_SETTLE, DONE_HOLD, and an already-settled TIMEOUT. */
#define BALL_QUIET_SAMPLE_COUNT                              5u
#define BALL_QUIET_MIN_DT_MS            BALL_ESTIMATOR_MIN_DT_MS
#define BALL_QUIET_MAX_DT_MS            BALL_ESTIMATOR_MAX_DT_MS
#define BALL_QUIET_ENTER_ERROR_CM                         0.70f
#define BALL_QUIET_ENTER_SPEED_CM_S                       0.35f
#define BALL_QUIET_ENTER_FRAMES                              5u
#define BALL_QUIET_ENTRY_AWAY_GUARD_CM_S                  0.25f
#define BALL_QUIET_ENTRY_AWAY_GUARD_FRAMES                   2u
#define BALL_QUIET_HOLD_ERROR_CM                          0.75f
#define BALL_QUIET_POSITION_CONFIRM_MS                     200u
#define BALL_QUIET_ANCHOR_MOVE_CM                         0.15f
#define BALL_QUIET_SPEED_EXIT_CM_S                        0.70f
#define BALL_QUIET_SPEED_EXIT_FRAMES                         3u
#define BALL_QUIET_HARD_ERROR_CM                          0.95f
#define BALL_QUIET_HARD_ERROR_FRAMES                         2u
#define BALL_QUIET_LARGE_ERROR_CM                         0.90f
#define BALL_QUIET_BIAS_ERROR_CM                          0.60f
#define BALL_QUIET_BIAS_SPEED_CM_S                        0.35f
#define BALL_QUIET_BIAS_SPREAD_CM                         0.16f
#define BALL_QUIET_BIAS_DIRECTION_COUNT                      4u
#define BALL_QUIET_BIAS_CONFIRM_MS                          400u
#define BALL_QUIET_PULSE_ACTIVE_MS                           40u
#define BALL_QUIET_PULSE_OBSERVE_MS                         160u
#define BALL_QUIET_PULSE_REARM_MS                           300u
#define BALL_QUIET_PULSE_END_SPEED_CM_S                    0.50f
#define BALL_QUIET_RECAPTURE_WAIT_FRAMES                      1u
#define BALL_QUIET_SERVO_EVENT_WINDOW_MS                   1000u

#define BALL_TARGET_B_CM                         (-5.0f)
#define BALL_SETTLE_KP_DEG_PER_CM                  4.0f
#define BALL_SETTLE_KV_DEG_PER_CM_S                1.2f
#define BALL_DYNAMIC_SPEED_CM_S                    0.50f
#define BALL_DYNAMIC_MIN_OFFSET_DEG               20.0f
#define BALL_DYNAMIC_MAX_OFFSET_DEG               30.0f

/*
 * B-point energy management.  Inside the damping band an automatic command
 * may only coast or oppose measured motion; it must not add kinetic energy.
 */
#define BALL_SETTLE_PREDICT_S                       0.16f
#define BALL_SETTLE_DAMP_BAND_CM                    1.00f
#define BALL_SETTLE_MOVE_ENTER_CM_S                 0.70f
#define BALL_SETTLE_MOVE_EXIT_CM_S                  0.35f
#define BALL_SETTLE_MOVE_EXIT_FRAMES                  3u
#define BALL_SETTLE_DAMP_BASE_DEG                  20.0f
#define BALL_SETTLE_DAMP_KV_DEG_PER_CM_S            1.50f
#define BALL_SETTLE_DAMP_MAX_DEG                   28.0f

#define BALL_PULSE_INITIAL_OFFSET_DEG             20.0f
#define BALL_PULSE_STEP_OFFSET_DEG                 2.0f
#define BALL_PULSE_MAX_OFFSET_DEG                 26.0f
#define BALL_PULSE_ACTIVE_MS                       60u
#define BALL_PULSE_OBSERVE_MS                      80u
#define BALL_PULSE_EFFECTIVE_MOVE_CM               0.08f
#define BALL_PULSE_NO_MOVE_REPEATS                  2u

#define BALL_DONE_POSITION_TOL_CM                  0.60f
#define BALL_DONE_SPEED_TOL_CM_S                   0.50f
#define BALL_DONE_HOLD_MS                          300u
#define BALL_HOLD_RECAPTURE_POSITION_CM            0.75f
#define BALL_HOLD_RECAPTURE_SPEED_CM_S             0.60f

#define BALL_BUDGET_A_TURN_MS                     1500u
#define BALL_BUDGET_B_APPROACH_MS                 2800u
#define BALL_BUDGET_B_BRAKE_MS                    3200u
#define BALL_BUDGET_B_BAND_MS                     3800u
#define BALL_BUDGET_DONE_MS                       4500u
#define BALL_OFFICIAL_TIMEOUT_MS                  5000u
#define BALL_B_FIRST_BAND_TOL_CM                   0.80f
#define BALL_B_FIRST_BAND_SPEED_TARGET_CM_S        1.50f
#define BALL_B_FIRST_OVERSHOOT_TARGET_CM           0.60f
#define BALL_B_CROSS_COUNT_TARGET                     2u
#define BALL_B_BAND_TO_DONE_TARGET_MS               700u

#define BALL_OLED_ACTIVE_PERIOD_MS                 250u
#define BALL_OLED_PAGE_STEP_PERIOD_MS               20u
#define BALL_LOG_CAPACITY                          256u
#define BALL_LOG_MIN_PERIOD_MS                      40u

#endif
