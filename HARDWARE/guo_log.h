#ifndef __KAOYA_LOG_H
#define __KAOYA_LOG_H

/*
 * C30D阿克曼迁移版的轻量异步事件日志。
 * 业务任务只投递固定长度事件记录，LogTask在低优先级上下文中格式化并输出，
 * 因此不会在100 Hz控制任务或DMA/USART中断中执行printf、浮点格式化或阻塞发送。
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AKM_LOG_ENABLE
#define AKM_LOG_ENABLE             (1u)
#endif

/* 当前工程USART2被原APP使用；连接APP时可将此宏改为0，仍保留队列统计。 */
#ifndef AKM_LOG_OUTPUT_ENABLE
#define AKM_LOG_OUTPUT_ENABLE      (1u)
#endif

#define AKM_LOG_TASK_PRIO          (1u)
#define AKM_LOG_TASK_STK_SIZE      (384u)
#define AKM_LOG_QUEUE_DEPTH        (32u)
#define AKM_LOG_TEXT_SIZE          (96u)

typedef enum
{
    AKM_LOG_EVENT_BOOT = 1u,
    AKM_LOG_EVENT_COMMAND_OK,
    AKM_LOG_EVENT_COMMAND_TIMEOUT,
    AKM_LOG_EVENT_PROTOCOL_ERROR,
    AKM_LOG_EVENT_DMA_ERROR,
    AKM_LOG_EVENT_LOW_BATTERY
} akm_log_event_t;

/*
 * 初始化日志队列。必须在创建LogTask前调用一次；重复调用不会覆盖已有队列。
 */
void KaoyaLog_Init(void);

/* 低优先级日志任务；只在任务上下文格式化和发送文本。 */
void KaoyaLogTask(void *argument);

/*
 * 投递整数事件。参数含义由event定义，统一使用有符号32位，避免在生产者
 * 中传递指向局部字符串或局部缓冲区的悬空指针。
 * 返回1表示进入队列，返回0表示未初始化或队列已满。
 */
uint8_t KaoyaLog_Post(akm_log_event_t event,
                      int32_t arg0,
                      int32_t arg1,
                      int32_t arg2);

/* 诊断接口：累计丢弃的事件数和已输出的事件数。 */
uint32_t KaoyaLog_GetDropCount(void);
uint32_t KaoyaLog_GetSentCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __KAOYA_LOG_H */
