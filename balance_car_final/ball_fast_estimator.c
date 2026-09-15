#include "ball_fast_estimator.h"

#include <string.h>

static float BallFast_Clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

void BallFastEstimator_Init(BallFastEstimator *estimator)
{
    BallFastEstimator_Reset(estimator);
}

void BallFastEstimator_Reset(BallFastEstimator *estimator)
{
    memset(estimator, 0, sizeof(*estimator));
}

void BallFastEstimator_Seed(BallFastEstimator *estimator,
                            float position_cm,
                            uint32_t time_ms)
{
    BallFastEstimator_Reset(estimator);
    estimator->samples[0].time_ms = time_ms;
    estimator->samples[0].position_cm = position_cm;
    estimator->count = 1u;
    estimator->last_time_ms = time_ms;
    estimator->has_last_time = 1u;
}

static float BallFast_LeastSquaresVelocity(const BallFastEstimator *estimator)
{
    float mean_t_s = 0.0f;
    float mean_x_cm = 0.0f;
    float numerator = 0.0f;
    float denominator = 0.0f;
    uint32_t origin_ms = estimator->samples[0].time_ms;
    uint8_t i;

    for (i = 0u; i < estimator->count; ++i) {
        float t_s = (float)(uint32_t)(estimator->samples[i].time_ms -
            origin_ms) * 0.001f;
        mean_t_s += t_s;
        mean_x_cm += estimator->samples[i].position_cm;
    }
    mean_t_s /= (float)estimator->count;
    mean_x_cm /= (float)estimator->count;

    for (i = 0u; i < estimator->count; ++i) {
        float t_s = (float)(uint32_t)(estimator->samples[i].time_ms -
            origin_ms) * 0.001f;
        float dt_s = t_s - mean_t_s;
        float dx_cm = estimator->samples[i].position_cm - mean_x_cm;
        numerator += dt_s * dx_cm;
        denominator += dt_s * dt_s;
    }
    if (denominator <= 0.0000001f) {
        return 0.0f;
    }
    return numerator / denominator;
}

void BallFastEstimator_Add(BallFastEstimator *estimator,
                           float position_cm,
                           uint32_t time_ms,
                           BallFastEstimate *estimate)
{
    uint32_t interval_ms = 0u;
    uint8_t gap_reset = 0u;

    if (estimator->has_last_time != 0u) {
        interval_ms = (uint32_t)(time_ms - estimator->last_time_ms);
        if ((interval_ms < BALL_FAST_ESTIMATOR_MIN_DT_MS) ||
            (interval_ms > BALL_FAST_ESTIMATOR_MAX_DT_MS)) {
            BallFastEstimator_Reset(estimator);
            gap_reset = 1u;
        }
    }

    if (estimator->count >= BALL_FAST_ESTIMATOR_SAMPLE_COUNT) {
        uint8_t i;
        for (i = 1u; i < BALL_FAST_ESTIMATOR_SAMPLE_COUNT; ++i) {
            estimator->samples[i - 1u] = estimator->samples[i];
        }
        estimator->count = BALL_FAST_ESTIMATOR_SAMPLE_COUNT - 1u;
    }
    estimator->samples[estimator->count].time_ms = time_ms;
    estimator->samples[estimator->count].position_cm = position_cm;
    estimator->count++;
    estimator->last_time_ms = time_ms;
    estimator->has_last_time = 1u;

    estimate->velocity_cm_s = 0.0f;
    estimate->interval_ms = interval_ms;
    estimate->velocity_valid = 0u;
    estimate->gap_reset = gap_reset;

    if (estimator->count >= BALL_FAST_ESTIMATOR_SAMPLE_COUNT) {
        float raw_velocity_cm_s = BallFast_LeastSquaresVelocity(estimator);
        raw_velocity_cm_s = BallFast_Clamp(raw_velocity_cm_s,
            -BALL_FAST_ESTIMATOR_SPEED_LIMIT_CM_S,
            BALL_FAST_ESTIMATOR_SPEED_LIMIT_CM_S);
        if (estimator->has_filtered_velocity == 0u) {
            estimator->filtered_velocity_cm_s = raw_velocity_cm_s;
            estimator->has_filtered_velocity = 1u;
        } else {
            estimator->filtered_velocity_cm_s =
                BALL_FAST_ESTIMATOR_ALPHA * raw_velocity_cm_s +
                (1.0f - BALL_FAST_ESTIMATOR_ALPHA) *
                    estimator->filtered_velocity_cm_s;
        }
        estimator->filtered_velocity_cm_s = BallFast_Clamp(
            estimator->filtered_velocity_cm_s,
            -BALL_FAST_ESTIMATOR_SPEED_LIMIT_CM_S,
            BALL_FAST_ESTIMATOR_SPEED_LIMIT_CM_S);
        estimate->velocity_cm_s = estimator->filtered_velocity_cm_s;
        estimate->velocity_valid = 1u;
    }
}
