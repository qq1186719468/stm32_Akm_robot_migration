#ifndef __AKM_MONITOR_H
#define __AKM_MONITOR_H

/*
 * Read-only diagnostic layer for the C30D 1.0 Ackermann migration copy.
 *
 * The existing WHEELTEC control, encoder and MPU6050 tasks remain the
 * sources of truth.  This module only takes an atomic snapshot of their
 * already-published variables and exposes FreeRTOS health information.  It
 * never writes a motor PWM register and never changes a motion target.
 *
 * 中文边界：本模块是“旁路诊断”，不是安全状态机，也不是硬件看门狗。
 * 它只复制已有控制/传感器变量并统计任务资源，不调用Drive_Motor()、
 * Set_Pwm()或修改Move_X/Y/Z。heartbeat变化只能证明任务到达过埋点，
 * 不能单独证明整个控制系统绝对正常。
 */

#include <stdint.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" { 
#endif

#ifndef AKM_MONITOR_ENABLE
#define AKM_MONITOR_ENABLE        (1u)
#endif

#define AKM_MONITOR_TASK_PRIO     (2u)
#define AKM_MONITOR_TASK_STK_SIZE (384u)
#define AKM_MONITOR_MAX_TASKS     (10u)

/* 心跳ID与FreeRTOS优先级无关，数组下标只用于区分任务。 */
typedef enum
{
    AKM_MONITOR_HB_BALANCE = 0,
    AKM_MONITOR_HB_IMU,
    AKM_MONITOR_HB_SHOW,
    AKM_MONITOR_HB_LED,
    AKM_MONITOR_HB_PS2,
    AKM_MONITOR_HB_DATA,
    AKM_MONITOR_HB_UART3_DMA,
    AKM_MONITOR_HB_MONITOR,
    AKM_MONITOR_HB_LOG,
    AKM_MONITOR_HB_WATCHDOG,
    AKM_MONITOR_HB_COUNT
} akm_monitor_hb_id_t;

extern volatile uint32_t g_akm_monitor_heartbeat[AKM_MONITOR_HB_COUNT];

/* 各任务在正常循环点调用；计数自然回绕，比较差值仍可判断是否变化。 */
#define AKM_MONITOR_HEARTBEAT(id) \
    (g_akm_monitor_heartbeat[(uint32_t)(id)]++)

/* xTaskCreate成功并取得句柄后注册一次，stack_words单位为32位word。 */
typedef struct
{
    TaskHandle_t handle;
    const char  *name;
    uint16_t     stack_words;
    uint8_t      heartbeat_id;
} akm_monitor_task_registration_t;

/* 提供给OLED、上行通信或调试器的任务诊断副本。 */
typedef struct
{
    const char  *name;
    TaskHandle_t handle;
    uint16_t     stack_words;
    uint16_t     stack_high_water_words;
    uint8_t      state;
    uint8_t      heartbeat_id;
    uint32_t     heartbeat;
    uint32_t     heartbeat_delta;
} akm_monitor_task_info_t;

/*
 * IMU字段是现有MPU6050驱动完成偏置处理后的原始整数计数，不是姿态角、
 * SI单位或融合结果。后续状态上报适配层必须结合量程和轴映射再换算。
 *
 * IMU values are MPU6050 corrected integer counts (the existing driver has
 * already applied its bias correction and Z-axis gravity offset).  They are
 * not presented as fused attitude or SI units; a later telemetry adapter can
 * apply the board-specific scale and axis mapping.
 */
typedef struct
{
    uint32_t sequence;   /* 每完成一次快照递增 */
    uint32_t tick;       /* FreeRTOS tick */
    uint32_t time_count; /* 原控制任务启动计数 */

    uint16_t battery_mv;
    uint8_t  flag_stop;
    uint8_t  enable_pin;
    uint8_t  car_mode;
    uint8_t  command_lost_count;
    uint8_t  control_stop_reason;
    uint8_t  control_timeout_latched;
    uint8_t  control_state;
    uint8_t  source_flags; /* bit0..5对应PS2/APP/遥控/CAN/USART1/USART5 */
    uint8_t  iwdg_enabled;
    uint8_t  iwdg_reset_cause;

    float motor_a_target_mps;
    float motor_b_target_mps;
    float motor_a_actual_mps;
    float motor_b_actual_mps;
    int    servo_target_pwm; /* 控制算法给出的逻辑目标 */
    uint16_t servo_ccr2_pwm; /* TIM12 CH2当前寄存器读回值 */

    int16_t gyro_corrected_counts[3];
    int16_t accel_corrected_counts[3];

    uint32_t uart_overflow_count;
    uint32_t legacy_frame_count;
    uint32_t legacy_check_error_count;
    uint32_t kaoya_frame_count;
    uint32_t kaoya_error_count;
    uint32_t kaoya_unsupported_count;
    uint32_t dma_error_count;
    uint32_t usart_error_count;
    uint32_t tx_frame_count;
    /* TX DMA完成中断计数；与“启动次数”分开，便于现场判断完整发送。 */
    uint32_t tx_complete_count;
    uint32_t tx_drop_count;
    uint32_t tx_error_count;
    uint32_t log_drop_count;
    uint32_t log_sent_count;

    size_t   free_heap_bytes;
    size_t   minimum_free_heap_bytes;
    uint16_t registered_task_count;
    uint16_t minimum_stack_high_water_words; /* 已注册任务中最小剩余栈word */
} akm_status_snapshot_t;

/*
 * 最近一次由MonitorTask发布的快照；导出为全局对象，方便Keil Watch展开。
 * 多字段读取仍建议使用AkmMonitor_GetSnapshot()，以获得临界区保护。
 */
extern volatile akm_status_snapshot_t g_akm_monitor_snapshot;

/* 注册/读取任务元数据；注册表复制过程受临界区保护。
 * Register/inspect task metadata.  Registration is safe before the scheduler
 * starts and while the monitor task is running. */
void AkmMonitor_RegisterTask(const akm_monitor_task_registration_t *task_info);
uint32_t AkmMonitor_GetTaskInfo(akm_monitor_task_info_t *out, uint32_t max);

/* 原子复制最近一次完整快照，调用者不会读到半更新结构体。 */
void AkmMonitor_GetSnapshot(akm_status_snapshot_t *out);

/* 在任务上下文立即采集；后续UplinkTask可先采集再编码。 */
void AkmMonitor_CollectSnapshot(void);

/* FreeRTOS任务入口：严格每1秒采集一次，不参与100 Hz控制闭环。 */
void AkmMonitorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __AKM_MONITOR_H */
