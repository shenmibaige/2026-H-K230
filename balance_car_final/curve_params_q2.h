#ifndef CURVE_PARAMS_H
#define CURVE_PARAMS_H

/*
 * 2026 Liaoning H Q2 geometry-model smooth tracing params.
 *
 * Motor1 = right rear, Motor2 = left rear. Motor1 mirror mounting
 * reverses electrical forward in motor.c (keeps gap-hold live-car
 * convention). Encoder version & single-turn count not confirmed yet,
 * so encoder is telemetry only; grayscale position PID drives
 * left/right PWM directly -- no dual-wheel speed PID.
 *
 * v1.3 stop-precision derivative (2026-07-30):
 * - The verified tracing PID and normal 15-17 s lap settings stay unchanged.
 * - A two-stage final-approach speed schedule reduces kinetic stopping
 *   distance before the A-line.
 * - Raw B/C/D confirmation provides a 1 ms-path finish stop while the
 *   original filtered confirmation remains as a fallback.
 *
 * v13 speed tuning: target lap 16 s (was ~20 s).
 *   BASE_STRAIGHT_PWM   78 ->  98
 *   BASE_MID_PWM        64 ->  80
 *   BASE_OUTER_PWM      52 ->  65
 *   PWM_LIMIT           95 -> 100
 *   RISE_PER_5MS         4 ->   5
 *   FALL_PER_5MS         6 ->   8
 *
 *   LINE_PID_KP        0.50 -> 0.48 (slightly softer at speed)
 *   LINE_PID_KD        0.20 -> 0.25 (more damping for higher speed)
 *   OUT_LIMIT          38.0 -> 52.0 (keep steering authority)
 *
 *   HARD_TIMEOUT_MS  30000 -> 25000
 */

#define CURVE_SENSOR_COUNT                  5U
#define CURVE_SENSOR_MASK                   0x1FU
#define CURVE_FINISH_BCD_MASK               0x0EU
#define CURVE_HISTORY_MASK                  0x1FU
#define CURVE_HISTORY_SAMPLES               5U
#define CURVE_Q8_SCALE                      256

/* A/B/C/D/E lateral positions relative to C, in mm. */
#define CURVE_SENSOR_A_MM                   (-45)
#define CURVE_SENSOR_B_MM                   (-15)
#define CURVE_SENSOR_C_MM                   0
#define CURVE_SENSOR_D_MM                   15
#define CURVE_SENSOR_E_MM                   45

/* Verified body & track geometry. */
#define CURVE_TRACK_RADIUS_MM               500
#define CURVE_WHEEL_TRACK_MM                180
#define CURVE_WHEEL_DIAMETER_MM             65
#define CURVE_SENSOR_LOOKAHEAD_MM           265
#define CURVE_CASTER_AXLE_DISTANCE_MM       215
#define CURVE_SENSOR_MODULE_WIDTH_MM        92
#define CURVE_THEORY_INNER_RADIUS_MM        410
#define CURVE_THEORY_OUTER_RADIUS_MM        590

#define CURVE_CONTROL_PERIOD_MS             5U
#define CURVE_OLED_REFRESH_MS               100U
#define CURVE_START_RAMP_MS                 300U
#define CURVE_FINISH_ARM_MIN_MS             2000U
#define CURVE_FINISH_APPROACH_STAGE1_MS     12000U
#define CURVE_FINISH_APPROACH_STAGE2_MS     14000U
#define CURVE_HARD_TIMEOUT_MS               25000U
#define CURVE_START_CLEAR_CYCLES            3U
#define CURVE_FINISH_CONFIRM_CYCLES         3U
#define CURVE_FINISH_RAW_CONFIRM_SAMPLES    2U

/*
 * Position PID:
 * - Input = error in mm (negative = line on left).
 * - Internal error = 0 - line_error.
 * - Positive output = turn left (slow left wheel, speed right wheel).
 *
 * Ki discrete period is fixed at 5 ms.
 */
#define CURVE_LINE_PID_KP                   0.48f
#define CURVE_LINE_PID_KI                   0.01f
#define CURVE_LINE_PID_KD                   0.25f
#define CURVE_LINE_PID_I_OUT_LIMIT          15.0f
#define CURVE_LINE_PID_OUT_LIMIT            52.0f

#define CURVE_ERROR_NEAR_Q8                 (15 * CURVE_Q8_SCALE / 2)
#define CURVE_ERROR_MID_Q8                  (45 * CURVE_Q8_SCALE / 2)
#define CURVE_BASE_STRAIGHT_PWM             98
#define CURVE_BASE_MID_PWM                  80
#define CURVE_BASE_OUTER_PWM                65

/* 12-14 s first approach stage. */
#define CURVE_APPROACH1_STRAIGHT_PWM        82
#define CURVE_APPROACH1_MID_PWM             70
#define CURVE_APPROACH1_OUTER_PWM           58

/* >=14 s final approach stage. */
#define CURVE_APPROACH2_STRAIGHT_PWM        60
#define CURVE_APPROACH2_MID_PWM             54
#define CURVE_APPROACH2_OUTER_PWM           46

#define CURVE_PWM_LIMIT                     100
#define CURVE_PWM_RISE_PER_5MS              5
#define CURVE_PWM_FALL_PER_5MS              8

#define CURVE_KEY_START                     1U
#define CURVE_KEY_EMERGENCY_STOP            4U

#endif
