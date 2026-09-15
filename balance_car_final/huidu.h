#ifndef HUIDU_H
#define HUIDU_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio);
void huidu_get_value(void);
uint8_t huidu_get_mask(void);

extern uint8_t huidu_value[];

#endif
