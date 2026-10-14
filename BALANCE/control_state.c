#include "control_safety.h"

#include "system.h"
#include "adc.h"
#include "motor.h"

extern int Time_count;

/*
 * 该状态是对现有控制变量的只读派生，不直接驱动PWM。
 * 目的：把“启动未就绪”和“运行中故障”从Flag_Stop语义中分离出来，
 * 为MonitorTask和后续IWDG门控提供一致判据。状态更新在任务上下文执行。
 */
volatile uint8_t g_akm_control_state = CONTROL_STATE_STARTUP;

void Control_StateRefresh(void)
{
    uint8_t next_state;

    if (Time_count < CONTROL_DELAY)
    {
        next_state = CONTROL_STATE_STARTUP;
    }
    else if (Check != 0)
    {
        next_state = CONTROL_STATE_SELF_CHECK;
    }
    else if (g_control_timeout_latched != 0u)
    {
        /* 通信超时是可恢复的运行状态，不等同于MCU故障。 */
        next_state = CONTROL_STATE_COMMAND_TIMEOUT;
    }
    else if ((Flag_Stop != 0u) || (EN == 0) || (Voltage < 10.0f))
    {
        next_state = CONTROL_STATE_HARD_STOP;
    }
    else
    {
        next_state = CONTROL_STATE_READY;
    }

    g_akm_control_state = next_state;
}

uint8_t Control_StateIsOperational(void)
{
    return ((g_akm_control_state == CONTROL_STATE_READY) ||
            (g_akm_control_state == CONTROL_STATE_COMMAND_TIMEOUT)) ? 1u : 0u;
}

uint8_t Control_StateAllowsWatchdog(void)
{
    return ((g_akm_control_state != CONTROL_STATE_STARTUP) &&
            (g_akm_control_state != CONTROL_STATE_SELF_CHECK)) ? 1u : 0u;
}
