#include "ball_ui.h"

#include <string.h>

#include "ball_config.h"

void BallUiThrottle_Init(BallUiThrottle *ui, uint32_t now_ms)
{
    memset(ui, 0, sizeof(*ui));
    ui->last_refresh_ms = now_ms;
    ui->dirty = 1u;
    ui->initialized = 1u;
}

void BallUiThrottle_Request(BallUiThrottle *ui)
{
    ui->dirty = 1u;
}

uint8_t BallUiThrottle_TakeRefresh(BallUiThrottle *ui,
                                   uint32_t now_ms,
                                   uint8_t active)
{
    uint32_t minimum_period_ms = BALL_OLED_ACTIVE_PERIOD_MS;

    (void)active;
    if ((ui->initialized == 0u) || (ui->dirty == 0u)) {
        return 0u;
    }
    if ((uint32_t)(now_ms - ui->last_refresh_ms) < minimum_period_ms) {
        return 0u;
    }

    ui->last_refresh_ms = now_ms;
    ui->refresh_count++;
    ui->dirty = 0u;
    return 1u;
}
