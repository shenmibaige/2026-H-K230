#ifndef BALL_UI_H
#define BALL_UI_H

#include <stdint.h>

typedef struct {
    uint32_t last_refresh_ms;
    uint32_t refresh_count;
    uint8_t dirty;
    uint8_t initialized;
} BallUiThrottle;

void BallUiThrottle_Init(BallUiThrottle *ui, uint32_t now_ms);
void BallUiThrottle_Request(BallUiThrottle *ui);
uint8_t BallUiThrottle_TakeRefresh(BallUiThrottle *ui,
                                   uint32_t now_ms,
                                   uint8_t active);

#endif
