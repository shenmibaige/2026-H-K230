#include "ti_msp_dl_config.h"
#include "encoder.h"

static volatile int32_t Encoder_Count1;
static volatile int32_t Encoder_Count2;

static uint32_t Encoder_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Encoder_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

void Encoder_Init(void)
{
    Encoder_Reset();
    DL_GPIO_clearInterruptStatus(
        ENCODER_A1_PORT,
        ENCODER_A1_PIN | ENCODER_B1_PIN);
    DL_GPIO_clearInterruptStatus(
        ENCODER_A2_PORT,
        ENCODER_A2_PIN | ENCODER_B2_PIN);
    DL_GPIO_enableInterrupt(
        ENCODER_A1_PORT,
        ENCODER_A1_PIN | ENCODER_B1_PIN);
    DL_GPIO_enableInterrupt(
        ENCODER_A2_PORT,
        ENCODER_A2_PIN | ENCODER_B2_PIN);
    NVIC_ClearPendingIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODER_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);
}

void Encoder_Reset(void)
{
    uint32_t primask = Encoder_EnterCritical();
    Encoder_Count1 = 0L;
    Encoder_Count2 = 0L;
    Encoder_ExitCritical(primask);
}

int32_t Encoder_ReadDelta1(void)
{
    int32_t value;
    uint32_t primask = Encoder_EnterCritical();
    value = Encoder_Count1;
    Encoder_Count1 = 0L;
    Encoder_ExitCritical(primask);
    return value;
}

int32_t Encoder_ReadDelta2(void)
{
    int32_t value;
    uint32_t primask = Encoder_EnterCritical();
    value = Encoder_Count2;
    Encoder_Count2 = 0L;
    Encoder_ExitCritical(primask);
    return value;
}

static int16_t Encoder_SaturateLegacy(int32_t value)
{
    if (value > 32767L)
    {
        return 32767;
    }
    if (value < -32768L)
    {
        return -32768;
    }
    return (int16_t)value;
}

int16_t Encoder_Get1(void)
{
    return Encoder_SaturateLegacy(Encoder_ReadDelta1());
}

int16_t Encoder_Get2(void)
{
    return Encoder_SaturateLegacy(Encoder_ReadDelta2());
}

void GROUP1_IRQHandler(void)
{
    uint32_t status1 = DL_GPIO_getPendingInterrupt(GPIOB);
    uint32_t status2;

    switch (status1)
    {
        case ENCODER_A1_IIDX:
            if (DL_GPIO_readPins(
                    ENCODER_B1_PORT,
                    ENCODER_B1_PIN) == 0U)
            {
                Encoder_Count1--;
            }
            break;

        case ENCODER_B1_IIDX:
            if (DL_GPIO_readPins(
                    ENCODER_A1_PORT,
                    ENCODER_A1_PIN) == 0U)
            {
                Encoder_Count1++;
            }
            break;

        default:
            break;
    }

    status2 = DL_GPIO_getPendingInterrupt(GPIOA);
    switch (status2)
    {
        case ENCODER_A2_IIDX:
            if (DL_GPIO_readPins(
                    ENCODER_B2_PORT,
                    ENCODER_B2_PIN) == 0U)
            {
                Encoder_Count2--;
            }
            break;

        case ENCODER_B2_IIDX:
            if (DL_GPIO_readPins(
                    ENCODER_A2_PORT,
                    ENCODER_A2_PIN) == 0U)
            {
                Encoder_Count2++;
            }
            break;

        default:
            break;
    }
}
