#include "ball_pid_bank.h"

#include <string.h>

typedef struct {
    float error_max_cm;
    float blend_half_width_cm;
    float kp_deg_per_cm;
    float ki_deg_per_cm_s;
    float kd_deg_per_cm_s;
    float output_max_deg;
} BallPidConfig;

/*
 * The six physical error regions retain the useful idea found in
 * balance_car_v3.0.  The reference code used one PID with six switched gain
 * sets and pixel-per-frame units; v3.1 instead owns six independent states,
 * updates every lane with true M0 time, and blends their bounded outputs.
 */
static const BallPidConfig g_pid_config[BALL_PID_COUNT] = {
    {0.17f, 0.12f,  4.0f, 0.80f, 1.9f, 20.0f},
    {0.50f, 0.15f,  5.0f, 0.50f, 1.7f, 22.0f},
    {1.00f, 0.20f,  6.5f, 0.25f, 1.5f, 24.0f},
    {1.67f, 0.25f,  8.0f, 0.10f, 1.4f, 28.0f},
    {2.67f, 0.35f,  9.5f, 0.05f, 1.2f, 32.0f},
    {9999.0f, 0.0f, 11.0f, 0.02f, 1.0f, 35.0f}
};

static float BallPid_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float BallPid_Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static void BallPid_AssignWeights(BallPidBank *bank, float error_abs_cm)
{
    uint8_t i;

    for (i = 0u; i < BALL_PID_COUNT; ++i) {
        bank->lane[i].weight = 0.0f;
    }

    for (i = 0u; i < (BALL_PID_COUNT - 1u); ++i) {
        float boundary = g_pid_config[i].error_max_cm;
        float half_width = g_pid_config[i].blend_half_width_cm;
        float lower = boundary - half_width;
        float upper = boundary + half_width;

        if (error_abs_cm < lower) {
            bank->lane[i].weight = 1.0f;
            bank->dominant_index = i;
            return;
        }
        if (error_abs_cm <= upper) {
            float blend = (error_abs_cm - lower) / (upper - lower);
            blend = BallPid_Clamp(blend, 0.0f, 1.0f);
            bank->lane[i].weight = 1.0f - blend;
            bank->lane[i + 1u].weight = blend;
            bank->dominant_index = (blend < 0.5f) ? i : (uint8_t)(i + 1u);
            return;
        }
    }

    bank->lane[BALL_PID_COUNT - 1u].weight = 1.0f;
    bank->dominant_index = BALL_PID_COUNT - 1u;
}

void BallPidBank_Init(BallPidBank *bank)
{
    BallPidBank_Reset(bank);
}

void BallPidBank_Reset(BallPidBank *bank)
{
    memset(bank, 0, sizeof(*bank));
}

void BallPidBank_ResetDynamics(BallPidBank *bank)
{
    uint8_t i;

    for (i = 0u; i < BALL_PID_COUNT; ++i) {
        bank->lane[i].integral_cm_s = 0.0f;
        bank->lane[i].output_deg = 0.0f;
        bank->lane[i].weight = 0.0f;
    }
    bank->error_cm = 0.0f;
    bank->predicted_error_cm = 0.0f;
    bank->fused_output_deg = 0.0f;
    bank->last_update_ms = 0u;
    bank->has_update_time = 0u;
    bank->update_mask = 0u;
    bank->dominant_index = 0u;
}

void BallPidBank_ResetIntegrals(BallPidBank *bank)
{
    uint8_t i;

    for (i = 0u; i < BALL_PID_COUNT; ++i) {
        bank->lane[i].integral_cm_s = 0.0f;
    }
}

void BallPidBank_Update(BallPidBank *bank,
                        float target_cm,
                        float position_cm,
                        float velocity_cm_s,
                        uint32_t now_ms,
                        uint8_t integral_allowed)
{
    float dt_s = 0.0f;
    uint8_t dt_valid = 0u;
    uint8_t i;

    if (bank->has_update_time != 0u) {
        uint32_t dt_ms = (uint32_t)(now_ms - bank->last_update_ms);
        if ((dt_ms >= BALL_ESTIMATOR_MIN_DT_MS) &&
            (dt_ms <= BALL_ESTIMATOR_MAX_DT_MS)) {
            dt_s = (float)dt_ms * 0.001f;
            dt_valid = 1u;
        } else {
            for (i = 0u; i < BALL_PID_COUNT; ++i) {
                bank->lane[i].integral_cm_s = 0.0f;
            }
        }
    }
    bank->last_update_ms = now_ms;
    bank->has_update_time = 1u;
    bank->update_mask = 0u;
    bank->error_cm = target_cm - position_cm;
    bank->predicted_error_cm = target_cm -
        (position_cm + BALL_PREDICT_DEFAULT_S * velocity_cm_s);

    BallPid_AssignWeights(bank, BallPid_Abs(bank->predicted_error_cm));
    bank->fused_output_deg = 0.0f;

    for (i = 0u; i < BALL_PID_COUNT; ++i) {
        const BallPidConfig *config = &g_pid_config[i];
        float integral = bank->lane[i].integral_cm_s;
        float output;

        if ((integral_allowed != 0u) && (dt_valid != 0u) &&
            (BallPid_Abs(bank->error_cm) <= BALL_PID_INTEGRAL_ERROR_MAX_CM) &&
            (BallPid_Abs(velocity_cm_s) <= BALL_PID_INTEGRAL_SPEED_MAX_CM_S)) {
            float candidate_integral = integral + bank->error_cm * dt_s;
            float integral_limit = BALL_PID_INTEGRAL_OUTPUT_MAX_DEG /
                                   config->ki_deg_per_cm_s;
            float candidate_output;

            candidate_integral = BallPid_Clamp(candidate_integral,
                                                -integral_limit,
                                                integral_limit);
            candidate_output = config->kp_deg_per_cm *
                               bank->predicted_error_cm +
                               config->ki_deg_per_cm_s * candidate_integral -
                               config->kd_deg_per_cm_s * velocity_cm_s;

            /* Conditional integration: do not add wind-up in the saturated
             * direction, but permit integration that returns toward range. */
            if ((BallPid_Abs(candidate_output) <= config->output_max_deg) ||
                (candidate_output * bank->error_cm < 0.0f)) {
                integral = candidate_integral;
            }
        } else {
            integral = 0.0f;
        }

        output = config->kp_deg_per_cm * bank->predicted_error_cm +
                 config->ki_deg_per_cm_s * integral -
                 config->kd_deg_per_cm_s * velocity_cm_s;
        output = BallPid_Clamp(output,
                               -config->output_max_deg,
                               config->output_max_deg);

        bank->lane[i].integral_cm_s = integral;
        bank->lane[i].output_deg = output;
        bank->lane[i].update_count++;
        bank->update_mask |= (uint8_t)(1u << i);
        bank->fused_output_deg += bank->lane[i].weight * output;
    }

    bank->fused_output_deg = BallPid_Clamp(bank->fused_output_deg,
                                           -BALL_PID_FUSED_MAX_DEG,
                                           BALL_PID_FUSED_MAX_DEG);
}
