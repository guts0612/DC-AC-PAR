#include "key.h"

#define KEY_DEBOUNCE_MS 20U
/* PE0/PB9 独立消抖；事件按位保存，同时松开也不会互相覆盖。 */
static volatile uint8_t Key_Pending;
static volatile uint8_t Key_Ready;
static uint8_t Key_Stable[2], Key_Candidate[2], Key_Confirmed[2];
static uint32_t Key_ChangeTick[2];

void Key_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i, state;
    __disable_irq();
    state = Key_GetState();
    for (i = 0U; i < 2U; ++i)
    {
        Key_Stable[i] = (state >> i) & 1U;
        Key_Candidate[i] = Key_Stable[i];
        Key_Confirmed[i] = 0U;
        Key_ChangeTick[i] = HAL_GetTick();
    }
    Key_Pending = 0U;
    Key_Ready = 1U;
    __set_PRIMASK(primask);
}

/* 一次取走一个松手事件：1=PE0，2=PB9，0=无事件。 */
uint8_t Key(void)
{
    uint8_t number = 0U;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((Key_Pending & 1U) != 0U) { number = 1U; Key_Pending &= (uint8_t)~1U; }
    else if ((Key_Pending & 2U) != 0U) { number = 2U; Key_Pending &= (uint8_t)~2U; }
    __set_PRIMASK(primask);
    return number;
}

/* 两键均上拉、接 GND 按下。返回位图：bit0=PE0，bit1=PB9。 */
uint8_t Key_GetState(void)
{
    uint8_t state = 0U;
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) state |= 1U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) state |= 2U;
    return state;
}

/* SysTick 每 1 ms 扫描；按下和松开各稳定 20 ms 后，松手触发一次。
 * 启动时已经按住的键必须先松开再按；长按不重复。
 */
void Key_Loop(void)
{
    uint8_t i, state, pressed;
    uint32_t now;
    if (Key_Ready == 0U) return;
    state = Key_GetState();
    now = HAL_GetTick();
    for (i = 0U; i < 2U; ++i)
    {
        pressed = (state >> i) & 1U;
        if (pressed != Key_Candidate[i])
        {
            Key_Candidate[i] = pressed;
            Key_ChangeTick[i] = now;
        }
        if ((pressed != Key_Stable[i]) &&
            ((uint32_t)(now - Key_ChangeTick[i]) >= KEY_DEBOUNCE_MS))
        {
            Key_Stable[i] = pressed;
            if (pressed != 0U) Key_Confirmed[i] = 1U;
            else if (Key_Confirmed[i] != 0U)
            {
                Key_Confirmed[i] = 0U;
                Key_Pending |= (uint8_t)(1U << i);
            }
        }
    }
}
