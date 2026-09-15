#include "delay.h"

void delay_us(uint32_t us)
{
    uint32_t cycles = (CPUCLK_FREQ ) * us;
    delay_cycles(cycles);
}  // 微秒级延时函数声明
void delay_ms(uint32_t ms)
{
    uint32_t cycles = (CPUCLK_FREQ /1000) * ms;
    delay_cycles(cycles);
}  // 毫秒级延时函数声明
