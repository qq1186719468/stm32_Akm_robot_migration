#ifndef __CONTROL_SAFETY_H
#define __CONTROL_SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 当前阶段的停止原因：后续可扩展低压、使能断开和锁存故障。 */
#define CONTROL_STOP_NONE              (0u)
#define CONTROL_STOP_COMMAND_TIMEOUT   (1u)

/* 派生控制状态与Flag_Stop分离；它用于监控和看门狗门控，不替换原安全输出。 */
#define CONTROL_STATE_STARTUP           (0u)
#define CONTROL_STATE_READY             (1u)
#define CONTROL_STATE_COMMAND_TIMEOUT   (2u)
#define CONTROL_STATE_HARD_STOP         (3u)
#define CONTROL_STATE_SELF_CHECK        (4u)

extern volatile uint8_t g_control_stop_reason;
extern volatile uint8_t g_control_timeout_latched;
extern volatile uint8_t g_akm_control_state;

/* 清零四路增量PI历史量、目标、PWM，并将阿克曼舵机写到1500us中位。 */
void Control_ResetOutputState(void);

/* 根据启动计数、自检、使能、电压和超时锁存刷新派生状态。 */
void Control_StateRefresh(void);
uint8_t Control_StateIsOperational(void);
/* 只要不在启动/自检阶段，核心任务仍可被看门狗监护；硬停本身不应触发复位。 */
uint8_t Control_StateAllowsWatchdog(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_SAFETY_H */
