#include "akm_monitor.h"

#include "system.h"
#include "adc.h"
#include "motor.h"
#include "uart3_dma.h"
#include "MPU6050.h"
#include "guo_uplink.h"
#include "control_safety.h"
#include "guo_log.h"
#include "akm_watchdog.h"
#include "tft_diag.h"

#include <string.h>

extern int Time_count;

/* 监控任务保持只读；心跳数组是唯一由各任务主动递增的诊断变量。 */
volatile uint32_t g_akm_monitor_heartbeat[AKM_MONITOR_HB_COUNT] = {0u};

static akm_monitor_task_registration_t s_registered_tasks[AKM_MONITOR_MAX_TASKS];
static uint32_t s_registered_task_count = 0u;
static uint32_t s_previous_heartbeat[AKM_MONITOR_HB_COUNT] = {0u};
/*
 * 导出最近一次完整快照，便于Keil Watch直接展开字段。
 * 该对象仍只由MonitorTask发布，消费者应优先调用AkmMonitor_GetSnapshot()。
 */
volatile akm_status_snapshot_t g_akm_monitor_snapshot;
static uint8_t s_previous_stop_reason = CONTROL_STOP_NONE;
static uint32_t s_previous_guo_error_count = 0u;
static uint32_t s_previous_dma_error_count = 0u;
static uint8_t s_low_battery_latched = 0u;

static uint16_t clamp_battery_mv(float voltage)
{
    float mv;

    /* 浮点电压转换为协议常用mV，并在转换为uint16前做饱和保护。 */
    mv = voltage * 1000.0f;
    if (mv <= 0.0f)
    {
        return 0u;
    }
    if (mv >= 65535.0f)
    {
        return 65535u;
    }
    return (uint16_t)(mv + 0.5f);
}

static uint16_t get_stack_hwm(TaskHandle_t handle, uint16_t total_words)
{
    UBaseType_t hwm;

    if ((handle == NULL) || (total_words == 0u))
    {
        return 0u;
    }

    /* high-water mark表示历史最少剩余栈，不是已经使用的栈大小。 */
#if defined(INCLUDE_uxTaskGetStackHighWaterMark2) && \
    (INCLUDE_uxTaskGetStackHighWaterMark2 == 1)
    hwm = uxTaskGetStackHighWaterMark2(handle);
#elif defined(INCLUDE_uxTaskGetStackHighWaterMark) && \
      (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    hwm = uxTaskGetStackHighWaterMark(handle);
#else
    hwm = 0u;
#endif

    if (hwm > (UBaseType_t)total_words)
    {
        hwm = (UBaseType_t)total_words;
    }
    return (uint16_t)hwm;
}

static uint8_t read_enable_pin(void)
{
    return (EN != 0) ? 1u : 0u;
}

static uint8_t make_source_flags(void)
{
    uint8_t flags = 0u;

    /* 压缩成位图便于OLED或协议上报；全0代表默认USART3/DMA控制路径。 */
    if (PS2_ON_Flag != 0u)       flags |= 0x01u;
    if (APP_ON_Flag != 0u)       flags |= 0x02u;
    if (Remote_ON_Flag != 0u)    flags |= 0x04u;
    if (CAN_ON_Flag != 0u)       flags |= 0x08u;
    if (Usart1_ON_Flag != 0u)    flags |= 0x10u;
    if (Usart5_ON_Flag != 0u)    flags |= 0x20u;
    /* No bit means the default USART3/DMA command path. */
    return flags;
}

void AkmMonitor_RegisterTask(const akm_monitor_task_registration_t *task_info)
{
    if ((task_info == NULL) || (task_info->handle == NULL) ||
        (task_info->name == NULL) ||
        (task_info->heartbeat_id >= AKM_MONITOR_HB_COUNT) ||
        (s_registered_task_count >= AKM_MONITOR_MAX_TASKS))
    {
        return;
    }

    /* 复制注册信息而非保存调用者结构体地址，避免局部变量生命周期问题。 */
    taskENTER_CRITICAL();
    s_registered_tasks[s_registered_task_count] = *task_info;
    s_registered_task_count++;
    taskEXIT_CRITICAL();
}

uint32_t AkmMonitor_GetTaskInfo(akm_monitor_task_info_t *out, uint32_t max)
{
    uint32_t count;
    uint32_t index;
    akm_monitor_task_registration_t local[AKM_MONITOR_MAX_TASKS];

    if ((out == NULL) || (max == 0u))
    {
        return 0u;
    }

    /* 临界区内只复制小型注册表，耗时API在临界区外调用。 */
    taskENTER_CRITICAL();
    count = s_registered_task_count;
    if (count > max)
    {
        count = max;
    }
    if (count > AKM_MONITOR_MAX_TASKS)
    {
        count = AKM_MONITOR_MAX_TASKS;
    }
    for (index = 0u; index < count; index++)
    {
        local[index] = s_registered_tasks[index];
    }
    taskEXIT_CRITICAL();

    for (index = 0u; index < count; index++)
    {
        uint32_t heartbeat;
        uint32_t previous;

        out[index].name = local[index].name;
        out[index].handle = local[index].handle;
        out[index].stack_words = local[index].stack_words;
        out[index].stack_high_water_words =
            get_stack_hwm(local[index].handle, local[index].stack_words);
        out[index].state = (uint8_t)eTaskGetState(local[index].handle);
        out[index].heartbeat_id = local[index].heartbeat_id;

        /* delta为最近两个监控周期之间的变化量，0表示未到达心跳埋点。 */
        heartbeat = g_akm_monitor_heartbeat[local[index].heartbeat_id];
        previous = s_previous_heartbeat[local[index].heartbeat_id];
        out[index].heartbeat = heartbeat;
        out[index].heartbeat_delta = heartbeat - previous;
    }
    return count;
}

void AkmMonitor_GetSnapshot(akm_status_snapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }

    /* 整体复制受临界区保护，消费者不会看到sequence已更新而字段未更新。 */
    taskENTER_CRITICAL();
    *out = g_akm_monitor_snapshot;
    taskEXIT_CRITICAL();
}

void AkmMonitor_CollectSnapshot(void)
{
    akm_status_snapshot_t next;
    akm_monitor_task_info_t task_info[AKM_MONITOR_MAX_TASKS];
    uint32_t task_count;
    uint32_t index;
    uint16_t minimum_hwm = 0xFFFFu;

    /* 先清局部对象，未来新增字段即使漏赋值也不会泄漏上次快照。 */
    memset(&next, 0, sizeof(next));
    Control_StateRefresh();

    /*
     * 在短临界区内冻结控制与传感器多字段，保证同一快照内部尽量一致。
     * 这里只复制现有结果，不重新读ADC/I2C，避免监控任务干扰硬件时序。
     */
    taskENTER_CRITICAL();
    next.tick = (uint32_t)xTaskGetTickCount();
    next.time_count = (uint32_t)Time_count;
    next.battery_mv = clamp_battery_mv(Voltage);
    next.flag_stop = Flag_Stop;
    next.enable_pin = read_enable_pin();
    next.car_mode = Car_Mode;
    next.command_lost_count = command_lost_count;
    next.control_stop_reason = g_control_stop_reason;
    next.control_timeout_latched = g_control_timeout_latched;
    next.control_state = g_akm_control_state;
    next.source_flags = make_source_flags();
    next.iwdg_enabled = AkmWatchdog_IsEnabled();
    next.iwdg_reset_cause = AkmWatchdog_WasIwdgReset();

    next.motor_a_target_mps = MOTOR_A.Target;
    next.motor_b_target_mps = MOTOR_B.Target;
    next.motor_a_actual_mps = MOTOR_A.Encoder;
    next.motor_b_actual_mps = MOTOR_B.Encoder;
    next.servo_target_pwm = Servo;
    next.servo_ccr2_pwm = (uint16_t)Servo_PWM;

    next.gyro_corrected_counts[0] = gyro[0];
    next.gyro_corrected_counts[1] = gyro[1];
    next.gyro_corrected_counts[2] = gyro[2];
    next.accel_corrected_counts[0] = accel[0];
    next.accel_corrected_counts[1] = accel[1];
    next.accel_corrected_counts[2] = accel[2];
    taskEXIT_CRITICAL();

    /* 通信计数getter为简单32位读取，放在临界区外缩短关调度时间。 */
    next.uart_overflow_count = UART3_DMA_GetOverflowCount();
    next.legacy_frame_count = UART3_DMA_GetFrameCount();
    next.legacy_check_error_count = UART3_DMA_GetCheckErrorCount();
    next.guo_frame_count = UART3_DMA_GetGuoFrameCount();
    next.guo_error_count = UART3_DMA_GetGuoErrorCount();
    next.guo_unsupported_count = UART3_DMA_GetGuoUnsupportedCount();
    next.dma_error_count = UART3_DMA_GetDmaErrorCount();
    next.usart_error_count = UART3_DMA_GetUsartErrorCount();
    next.tx_frame_count = UART3_DMA_GetTxFrameCount();
    next.tx_complete_count = UART3_DMA_GetTxCompleteCount();
    next.tx_drop_count = UART3_DMA_GetTxDropCount();
    next.tx_error_count = UART3_DMA_GetTxErrorCount();
    next.log_drop_count = GuoLog_GetDropCount();
    next.log_sent_count = GuoLog_GetSentCount();

    /* 只在计数/停止原因发生边沿时记录，避免每秒重复刷屏。 */
    if ((next.control_stop_reason != s_previous_stop_reason) &&
        (next.control_stop_reason == CONTROL_STOP_COMMAND_TIMEOUT))
    {
        (void)GuoLog_Post(AKM_LOG_EVENT_COMMAND_TIMEOUT,
                            (int32_t)next.command_lost_count, 0, 0);
    }
    s_previous_stop_reason = next.control_stop_reason;

    if (next.guo_error_count != s_previous_guo_error_count)
    {
        (void)GuoLog_Post(AKM_LOG_EVENT_PROTOCOL_ERROR,
                            (int32_t)next.guo_error_count, 0, 0);
        s_previous_guo_error_count = next.guo_error_count;
    }
    if (next.dma_error_count != s_previous_dma_error_count)
    {
        (void)GuoLog_Post(AKM_LOG_EVENT_DMA_ERROR,
                            (int32_t)next.dma_error_count, 0, 0);
        s_previous_dma_error_count = next.dma_error_count;
    }

    /* 低压事件只在下降沿记录；10.5 V回差用于防止阈值附近反复刷日志。 */
    if ((next.battery_mv < 10000u) && (s_low_battery_latched == 0u))
    {
        (void)GuoLog_Post(AKM_LOG_EVENT_LOW_BATTERY,
                            (int32_t)next.battery_mv, 0, 0);
        s_low_battery_latched = 1u;
    }
    else if (next.battery_mv >= 10500u)
    {
        s_low_battery_latched = 0u;
    }

#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
    next.free_heap_bytes = xPortGetFreeHeapSize();
    next.minimum_free_heap_bytes = xPortGetMinimumEverFreeHeapSize();
#else
    next.free_heap_bytes = 0u;
    next.minimum_free_heap_bytes = 0u;
#endif

    /* 汇总任务资源，并记录所有有效HWM中的最小值作为快速风险指标。 */
    task_count = AkmMonitor_GetTaskInfo(task_info, AKM_MONITOR_MAX_TASKS);
    next.registered_task_count = (uint16_t)task_count;
    for (index = 0u; index < task_count; index++)
    {
        if ((task_info[index].stack_high_water_words != 0u) &&
            (task_info[index].stack_high_water_words < minimum_hwm))
        {
            minimum_hwm = task_info[index].stack_high_water_words;
        }
        s_previous_heartbeat[task_info[index].heartbeat_id] =
            task_info[index].heartbeat;
    }
    if (minimum_hwm != 0xFFFFu)
    {
        next.minimum_stack_high_water_words = minimum_hwm;
    }

    /* sequence最后更新并整体发布，表示新快照已经全部准备完成。 */
    taskENTER_CRITICAL();
    next.sequence = g_akm_monitor_snapshot.sequence + 1u;
    g_akm_monitor_snapshot = next;
    taskEXIT_CRITICAL();
}

void AkmMonitorTask(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    /* vTaskDelayUntil使用绝对周期基准，减少普通vTaskDelay的累计漂移。 */
    for (;;)
    {
        AKM_MONITOR_HEARTBEAT(AKM_MONITOR_HB_MONITOR);
        AkmMonitor_CollectSnapshot();
        /* The monitor task owns the snapshot and refreshes TFT at 1 Hz. */
        TftDiag_Update();
        /* 低频状态上报与快照同属诊断链；失败只丢本次帧，不影响控制。 */
        (void)GuoUplink_TrySendLatest();
        vTaskDelayUntil(&last_wake, (TickType_t)M2T(1000u));
    }
}
