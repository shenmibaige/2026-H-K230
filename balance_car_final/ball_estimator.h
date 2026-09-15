#ifndef BALL_ESTIMATOR_H
#define BALL_ESTIMATOR_H

#include <stdint.h>

#include "ball_config.h"

typedef struct {
    uint32_t time_ms;
    float position_cm;
} BallEstimatorSample;

typedef struct {
    BallEstimatorSample samples[BALL_ESTIMATOR_SAMPLE_COUNT];
    uint8_t count;
    uint8_t has_last_time;
    uint8_t has_filtered_velocity;
    uint32_t last_time_ms;
    float filtered_velocity_cm_s;
} BallEstimator;

typedef struct {
    float position_cm;
    float velocity_cm_s;
    float predicted_position_cm;
    uint32_t interval_ms;
    uint8_t velocity_valid;
    uint8_t gap_reset;
} BallEstimate;

void BallEstimator_Init(BallEstimator *estimator);
void BallEstimator_Reset(BallEstimator *estimator);
void BallEstimator_Add(BallEstimator *estimator,
                       float position_cm,
                       uint32_t time_ms,
                       BallEstimate *estimate);

#endif
