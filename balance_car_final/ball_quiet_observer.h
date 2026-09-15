#ifndef BALL_QUIET_OBSERVER_H
#define BALL_QUIET_OBSERVER_H

#include <stdint.h>

#include "ball_config.h"

typedef struct {
    float position_cm[BALL_QUIET_SAMPLE_COUNT];
    uint32_t time_ms[BALL_QUIET_SAMPLE_COUNT];
    uint8_t count;
    uint8_t next;
    uint8_t has_last_time;
    uint32_t last_time_ms;
} BallQuietObserver;

typedef struct {
    float median_position_cm;
    float error_cm;
    float spread_cm;
    uint8_t positive_error_count;
    uint8_t negative_error_count;
    uint8_t valid;
    uint8_t gap_reset;
} BallQuietEstimate;

void BallQuietObserver_Init(BallQuietObserver *observer);
void BallQuietObserver_Reset(BallQuietObserver *observer);
void BallQuietObserver_Add(BallQuietObserver *observer,
                           float position_cm,
                           float target_cm,
                           uint32_t now_ms,
                           BallQuietEstimate *estimate);

#endif

