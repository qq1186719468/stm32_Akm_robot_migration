#include "akm_watchdog.h"

#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_rcc.h"

#include "system.h"
#include "control_safety.h"
#include "akm_monitor.h"

static volatile uint8_t s_iwdg_enabled = 0u;
static volatile uint8_t s_previous_iwdg_reset = 0u;
volatile uint8_t g_akm_watchdog_test_hold = 0u;

void AkmWatchdog_CaptureResetCause(void)
{
    s_previous_iwdg_reset =
        (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) ? 1u : 0u;
    /* 清除RCC复位标志，避免后续普通复位仍被误判为IWDG复位。 */
    RCC_ClearFlag();
}

uint8_t AkmWatchdog_WasIwdgReset(void)
{
    return s_previous_iwdg_reset;
}

static void akm_iwdg_start(void)
{
#if AKM_IWDG_ENABLE
    /* IWDG使用独立LSI时钟；一旦Enable，后续只能通过KR刷新。 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(AKM_IWDG_PRESCALER);
    IWDG_SetReload(AKM_IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
    s_iwdg_enabled = 1u;
#else
    s_iwdg_enabled = 0u;
#endif
}

static void akm_iwdg_reload(void)
{
#if AKM_IWDG_ENABLE
    IWDG_ReloadCounter();
#endif
}

void AkmWatchdogTask(void *argument)
{
    TickType_t last_wake;
    uint32_t previous_balance;
    uint32_t previous_imu;
    uint8_t ready_streak = 0u;
    uint32_t current_balance;
    uint32_t current_imu;

    (void)argument;
    last_wake = xTaskGetTickCount();
    previous_balance = g_akm_monitor_heartbeat[AKM_MONITOR_HB_BALANCE];
    previous_imu = g_akm_monitor_heartbeat[AKM_MONITOR_HB_IMU];

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(AKM_IWDG_PERIOD_MS));
        AKM_MONITOR_HEARTBEAT(AKM_MONITOR_HB_WATCHDOG);
        Control_StateRefresh();

        current_balance = g_akm_monitor_heartbeat[AKM_MONITOR_HB_BALANCE];
        current_imu = g_akm_monitor_heartbeat[AKM_MONITOR_HB_IMU];

        /* 启动校准阶段不启动IWDG；运行阶段要求两个核心任务均有进展。 */
        if ((Control_StateAllowsWatchdog() != 0u) &&
            (current_balance != previous_balance) &&
            (current_imu != previous_imu))
        {
            if (ready_streak < 255u)
            {
                ready_streak++;
            }
        }
        else
        {
            ready_streak = 0u;
        }

        if ((s_iwdg_enabled == 0u) && (ready_streak >= 2u))
        {
            /* 需要连续两个250 ms窗口正常后才允许启动不可关闭的IWDG。 */
            akm_iwdg_start();
        }
        if ((s_iwdg_enabled != 0u) && (ready_streak != 0u) &&
            (g_akm_watchdog_test_hold == 0u))
        {
            akm_iwdg_reload();
        }

        previous_balance = current_balance;
        previous_imu = current_imu;
    }
}

uint8_t AkmWatchdog_IsEnabled(void)
{
    return s_iwdg_enabled;
}
