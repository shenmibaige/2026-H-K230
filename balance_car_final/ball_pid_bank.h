#ifndef BALL_PID_BANK_H
#define BALL_PID_BANK_H

#include <stdint.h>

#include "ball_config.h"

typedef struct {
    float integral_cm_s;
    float output_deg;
    float weight;
    uint32_t update_count;
} BallPidLane;

typedef struct {
    BallPidLane lane[BALL_PID_COUNT];
    float error_cm;
    float predicted_error_cm;
    float fused_output_deg;
    uint32_t last_update_ms;
    uint8_t has_update_time;
    uint8_t update_mask;
    uint8_t dominant_index;
} BallPidBank;

void BallPidBank_Init(BallPidBank *bank);
void BallPidBank_Reset(BallPidBank *bank);
void BallPidBank_ResetDynamics(BallPidBank *bank);
void BallPidBank_ResetIntegrals(BallPidBank *bank);
void BallPidBank_Update(BallPidBank *bank,
                        float target_cm,
                        float position_cm,
                        float velocity_cm_s,
                        uint32_t now_ms,
                        uint8_t integral_allowed);

#endif
