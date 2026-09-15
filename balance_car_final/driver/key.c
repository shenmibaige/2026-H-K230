#include "key.h"
#include "ti_msp_dl_config.h"   // 包含 SysConfig 生成的引脚宏定义

/* 全局变量，用于存储按键键码 */
volatile uint8_t Key_Num;

/**
 * @brief  按键初始化
 * @note   引脚工作模式（下拉输入）已通过 SysConfig 配置完成，此处仅做占位。
 *         若需要额外使能 GPIO 模块时钟或中断，可在此补充。
 */
void Key_Init(void)
{
    /* 
     * 一般情况下，SysConfig 生成的初始化函数（如 SYSCFG_DL_init()）
     * 已在 main 开头调用，无需再次配置 GPIO。
     */
}

/**
 * @brief  获取当前按键状态（非阻塞）
 * @return 有按键按下时直接返回 1~4，无按键按下时返回 0
 */
static uint8_t Key_GetState(void)
{
    if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B6_PIN) != 0)
    {
        return 1;   // PB6 按键按下（高电平）
    }
    if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B7_PIN) != 0)
    {
        return 2;   // PB7 按键按下
    }
    if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B8_PIN) != 0)
    {
        return 3;   // PB8 按键按下
    }
    if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B9_PIN) != 0)
    {
        return 4;   // PB9 按键按下
    }
    return 0;       // 无按键按下
}

uint8_t Key_IsPressed(uint8_t key_num)
{
    switch (key_num)
    {
        case 1U:
            return (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B6_PIN) != 0U)
                       ? 1U
                       : 0U;
        case 2U:
            return (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B7_PIN) != 0U)
                       ? 1U
                       : 0U;
        case 3U:
            return (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B8_PIN) != 0U)
                       ? 1U
                       : 0U;
        case 4U:
            return (DL_GPIO_readPins(KEY_PORT, KEY_PIN_B9_PIN) != 0U)
                       ? 1U
                       : 0U;
        default:
            return 0U;
    }
}

/**
 * @brief  读取按键键码（读后清零）
 * @return 按键键码（1~4），无键码时返回 0
 */
uint8_t Key_GetNum(void)
{
    uint8_t Temp;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
        if ((primask & 1U) == 0U)
        {
            __enable_irq();
        }
        return Temp;
    }
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
    return 0;
}

/**
 * @brief  按键扫描定时中断函数
 * @note   必须每 1ms 调用一次（放在 1ms 定时器中断服务函数中），
 *         内部通过 20 分频实现 20ms 消抖与松手检测。
 */
void Key_Tick(void)
{
    static uint8_t Count;                // 计次分频
    static uint8_t CurrState, PrevState; // 本次状态和上次状态

    Count++;
    if (Count >= 20)                     // 每 20ms 执行一次
    {
        Count = 0;

        PrevState = CurrState;
        CurrState = Key_GetState();      // 获取当前按键状态

        /* 检测松手瞬间：当前无键按下，上次有键按下 */
        if (CurrState != 0 && PrevState == 0)
        {
            Key_Num = CurrState;         // 存入松手时的按键键码
        }
    }
}
