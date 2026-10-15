#ifndef __AKM_WATCHDOG_H
#define __AKM_WATCHDOG_H

/*
 * 阿克曼迁移版IWDG门控任务。
 * 默认关闭硬件复位（AKM_IWDG_ENABLE=0）。完成专项硬件验证后可显式启用。
 * 启用时IWDG不会在上电瞬间启动，
 * 只有启动/自检结束且Balance与MPU6050心跳连续正常两个250 ms窗口后才启动；
 * 启动后若核心任务不再推进，停止刷新并由硬件复位。
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AKM_IWDG_ENABLE
#define AKM_IWDG_ENABLE            (0u)
#endif

#define AKM_IWDG_TASK_PRIO         (3u)
#define AKM_IWDG_TASK_STK_SIZE     (256u)
#define AKM_IWDG_PERIOD_MS         (250u)

/* 32 kHz LSI、/64分频、重装1000，典型超时约2秒，实际值随LSI误差变化。 */
#define AKM_IWDG_PRESCALER         IWDG_Prescaler_64
#define AKM_IWDG_RELOAD            (1000u)

/* 记录并清除上一次复位原因；读取结果可供MonitorTask或日志使用。 */
void AkmWatchdog_CaptureResetCause(void);
uint8_t AkmWatchdog_WasIwdgReset(void);

/* Keil Watch测试开关：置1后停止刷新（仅AKM_IWDG_ENABLE=1时有效），用于验证复位。 */
extern volatile uint8_t g_akm_watchdog_test_hold;

/* 启动任务入口；IWDG仅在健康门控通过后写入IWDG寄存器。 */
void AkmWatchdogTask(void *argument);

/* 仅返回本任务是否已经真正启动IWDG。 */
uint8_t AkmWatchdog_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __AKM_WATCHDOG_H */
