#include "kaoya_log.h"

#include "usartx.h"
#include "akm_monitor.h"

#include <stdio.h>
#include <string.h>

#if AKM_LOG_ENABLE

/* 队列项只保存值，不保存调用者的指针；队列由FreeRTOS堆分配。 */
typedef struct
{
    uint8_t event;
    uint8_t reserved[3];
    uint32_t tick;
    int32_t arg0;
    int32_t arg1;
    int32_t arg2;
} akm_log_record_t;

static QueueHandle_t s_log_queue = NULL;
static volatile uint32_t s_drop_count = 0u;
static volatile uint32_t s_sent_count = 0u;

static void log_send_text(const char *text)
{
    uint16_t index;

    if ((text == NULL) || (AKM_LOG_OUTPUT_ENABLE == 0u))
    {
        return;
    }

    /* USART2当前为9600-8-N-1；发送只发生在LogTask，不阻塞控制任务。 */
    for (index = 0u; text[index] != '\0'; index++)
    {
        usart2_send((u8)text[index]);
    }
}

static uint16_t format_record(const akm_log_record_t *record, char *text)
{
    int length;

    if ((record == NULL) || (text == NULL))
    {
        return 0u;
    }

    /* 只使用整数格式，避免日志任务引入浮点printf的大量栈和代码开销。 */
    switch ((akm_log_event_t)record->event)
    {
    case AKM_LOG_EVENT_BOOT:
        length = sprintf(text, "[AKM][%lu] boot\r\n",
                         (unsigned long)record->tick);
        break;
    case AKM_LOG_EVENT_COMMAND_OK:
        /* arg0/arg1/arg2单位分别为mm/s、mm/s、mrad/s。 */
        length = sprintf(text, "[AKM][%lu] cmd vx=%ld vy=%ld wz=%ld\r\n",
                         (unsigned long)record->tick,
                         (long)record->arg0,
                         (long)record->arg1,
                         (long)record->arg2);
        break;
    case AKM_LOG_EVENT_COMMAND_TIMEOUT:
        /* arg0为触发超时前的command_lost_count。 */
        length = sprintf(text, "[AKM][%lu] timeout lost=%ld\r\n",
                         (unsigned long)record->tick,
                         (long)record->arg0);
        break;
    case AKM_LOG_EVENT_PROTOCOL_ERROR:
        /* arg0为累计协议错误计数。 */
        length = sprintf(text, "[AKM][%lu] proto_error count=%ld\r\n",
                         (unsigned long)record->tick,
                         (long)record->arg0);
        break;
    case AKM_LOG_EVENT_DMA_ERROR:
        /* arg0为累计DMA错误计数。 */
        length = sprintf(text, "[AKM][%lu] dma_error count=%ld\r\n",
                         (unsigned long)record->tick,
                         (long)record->arg0);
        break;
    case AKM_LOG_EVENT_LOW_BATTERY:
        /* arg0单位为mV。 */
        length = sprintf(text, "[AKM][%lu] low_battery mv=%ld\r\n",
                         (unsigned long)record->tick,
                         (long)record->arg0);
        break;
    default:
        length = sprintf(text, "[AKM][%lu] event=%u\r\n",
                         (unsigned long)record->tick,
                         (unsigned int)record->event);
        break;
    }

    if (length < 0)
    {
        text[0] = '\0';
        return 0u;
    }
    if (length >= (int)AKM_LOG_TEXT_SIZE)
    {
        length = (int)AKM_LOG_TEXT_SIZE - 1;
        text[length] = '\0';
    }
    return (uint16_t)length;
}

void KaoyaLog_Init(void)
{
    if (s_log_queue == NULL)
    {
        s_log_queue = xQueueCreate(AKM_LOG_QUEUE_DEPTH,
                                   (UBaseType_t)sizeof(akm_log_record_t));
    }
}

uint8_t KaoyaLog_Post(akm_log_event_t event,
                      int32_t arg0,
                      int32_t arg1,
                      int32_t arg2)
{
    akm_log_record_t record;

    if (s_log_queue == NULL)
    {
        s_drop_count++;
        return 0u;
    }

    record.event = (uint8_t)event;
    record.reserved[0] = 0u;
    record.reserved[1] = 0u;
    record.reserved[2] = 0u;
    record.tick = (uint32_t)xTaskGetTickCount();
    record.arg0 = arg0;
    record.arg1 = arg1;
    record.arg2 = arg2;

    /* 队列满时立即丢弃，不让控制任务等待日志输出。 */
    if (xQueueSend(s_log_queue, &record, (TickType_t)0u) != pdPASS)
    {
        s_drop_count++;
        return 0u;
    }
    return 1u;
}

void KaoyaLogTask(void *argument)
{
    akm_log_record_t record;
    char text[AKM_LOG_TEXT_SIZE];

    (void)argument;
    KaoyaLog_Init();
    if (s_log_queue == NULL)
    {
        /* 堆不足时不能对空队列调用xQueueReceive；保持任务存活并暴露故障。 */
        for (;;)
        {
            AKM_MONITOR_HEARTBEAT(AKM_MONITOR_HB_LOG);
            vTaskDelay(pdMS_TO_TICKS(1000u));
        }
    }
    log_send_text("[AKM] async log ready\r\n");
    (void)KaoyaLog_Post(AKM_LOG_EVENT_BOOT, 0, 0, 0);

    for (;;)
    {
        AKM_MONITOR_HEARTBEAT(AKM_MONITOR_HB_LOG);
        if (xQueueReceive(s_log_queue, &record, portMAX_DELAY) == pdPASS)
        {
            (void)format_record(&record, text);
            log_send_text(text);
            s_sent_count++;
        }
    }
}

uint32_t KaoyaLog_GetDropCount(void)
{
    return s_drop_count;
}

uint32_t KaoyaLog_GetSentCount(void)
{
    return s_sent_count;
}

#else

void KaoyaLog_Init(void) {}
void KaoyaLogTask(void *argument) { (void)argument; vTaskDelete(NULL); }
uint8_t KaoyaLog_Post(akm_log_event_t event, int32_t arg0, int32_t arg1, int32_t arg2)
{
    (void)event; (void)arg0; (void)arg1; (void)arg2;
    return 0u;
}
uint32_t KaoyaLog_GetDropCount(void) { return 0u; }
uint32_t KaoyaLog_GetSentCount(void) { return 0u; }

#endif
