#include "huidu.h"

uint8_t huidu_value[] = {0, 0, 0, 0, 0};

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio); 
    if((high_bits & gpio) != 0) 
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}

void huidu_get_value(void)
{
    huidu_value[0] = get_gpio_state(HUIDU_PORT, HUIDU_L2_PIN);
    huidu_value[1] = get_gpio_state(HUIDU_PORT, HUIDU_L1_PIN);
    huidu_value[2] = get_gpio_state(HUIDU_PORT, HUIDU_M_PIN);
    huidu_value[3] = get_gpio_state(HUIDU_PORT, HUIDU_R1_PIN);
    huidu_value[4] = get_gpio_state(HUIDU_PORT, HUIDU_R2_PIN);
}

uint8_t huidu_get_mask(void)
{
    uint8_t index;
    uint8_t mask = 0U;

    for (index = 0U; index < 5U; index++)
    {
        if (huidu_value[index] != 0U)
        {
            mask = (uint8_t)(mask | (uint8_t)(1U << index));
        }
    }
    return mask;
}
