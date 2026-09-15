#ifndef BALL_FAST_ESTIMATOR_H
#define BALL_FAST_ESTIMATOR_H

#include <stdint.h>

#include "ball_config.h"

typedef struct {
    uint32_t time_ms;
    float position_cm;
} BallFastEstimatorSample;

typedef struct {
    BallFastEstimatorSample samples[BALL_FAST_ESTIMATOR_SAMPLE_COUNT];
    uint8_t count;
    uint8_t has_last_time;
    uint8_t has_filtered_velocity;
    uint32_t last_time_ms;
    float filtered_velocity_cm_s;
} BallFastEstimator;

typedef struct {
    float velocity_cm_s;
    uint32_t interval_ms;
    uint8_t velocity_valid;
    uint8_t gap_reset;
} BallFastEstimate;

void BallFastEstimator_Init(BallFastEstimator *estimator);
void BallFastEstimator_Reset(BallFastEstimator *estimator);
void BallFastEstimator_Seed(BallFastEstimator *estimator,
                            float position_cm,
                            uint32_t time_ms);
void BallFastEstimator_Add(BallFastEstimator *estimator,
                           float position_cm,
                           uint32_t time_ms,
                           BallFastEstimate *estimate);

#endif
