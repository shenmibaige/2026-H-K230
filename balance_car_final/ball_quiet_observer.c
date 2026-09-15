#include "ball_quiet_observer.h"

#include <string.h>

static void BallQuiet_Sort(float values[BALL_QUIET_SAMPLE_COUNT])
{
    uint8_t i;

    for (i = 1u; i < BALL_QUIET_SAMPLE_COUNT; ++i) {
        float key = values[i];
        uint8_t j = i;
        while ((j > 0u) && (values[j - 1u] > key)) {
            values[j] = values[j - 1u];
            --j;
        }
        values[j] = key;
    }
}

void BallQuietObserver_Init(BallQuietObserver *observer)
{
    BallQuietObserver_Reset(observer);
}

void BallQuietObserver_Reset(BallQuietObserver *observer)
{
    memset(observer, 0, sizeof(*observer));
}

void BallQuietObserver_Add(BallQuietObserver *observer,
                           float position_cm,
                           float target_cm,
                           uint32_t now_ms,
                           BallQuietEstimate *estimate)
{
    uint8_t i;
    float sorted[BALL_QUIET_SAMPLE_COUNT];

    memset(estimate, 0, sizeof(*estimate));

    if (observer->has_last_time != 0u) {
        uint32_t dt_ms = (uint32_t)(now_ms - observer->last_time_ms);
        if ((dt_ms < BALL_QUIET_MIN_DT_MS) ||
            (dt_ms > BALL_QUIET_MAX_DT_MS)) {
            BallQuietObserver_Reset(observer);
            estimate->gap_reset = 1u;
        }
    }

    observer->position_cm[observer->next] = position_cm;
    observer->time_ms[observer->next] = now_ms;
    observer->next = (uint8_t)((observer->next + 1u) %
                               BALL_QUIET_SAMPLE_COUNT);
    if (observer->count < BALL_QUIET_SAMPLE_COUNT) {
        observer->count++;
    }
    observer->last_time_ms = now_ms;
    observer->has_last_time = 1u;

    if (observer->count < BALL_QUIET_SAMPLE_COUNT) {
        return;
    }

    for (i = 0u; i < BALL_QUIET_SAMPLE_COUNT; ++i) {
        float error_cm;
        sorted[i] = observer->position_cm[i];
        error_cm = target_cm - observer->position_cm[i];
        if (error_cm > 0.0f) {
            estimate->positive_error_count++;
        } else if (error_cm < 0.0f) {
            estimate->negative_error_count++;
        }
    }
    BallQuiet_Sort(sorted);
    estimate->median_position_cm =
        sorted[BALL_QUIET_SAMPLE_COUNT / 2u];
    estimate->spread_cm =
        sorted[BALL_QUIET_SAMPLE_COUNT - 1u] - sorted[0u];
    estimate->error_cm = target_cm - estimate->median_position_cm;
    estimate->valid = 1u;
}

