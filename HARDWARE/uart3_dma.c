#include "uart3_dma.h"

#include "misc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#include "system.h"
#include "guo_protocol.h"
#include "guo_log.h"

#include <string.h>

extern int Time_count;

#if AKM_UART3_DMA_ENABLE

/*
 * STM32F407中USART3_RX可映射到DMA1 Stream1/Channel4。
 * 256字节DMA缓冲区接收硬件数据，512字节软件ring用于解耦ISR和解析任务。
 * 115200-8N-1下256字节约22 ms写满一次，长时间屏蔽中断仍可能丢失多圈历史数据。
 */
#define UART3_DMA_STREAM        DMA1_Stream1
#define UART3_DMA_CHANNEL       DMA_Channel_4
#define UART3_DMA_STREAM_IRQn   DMA1_Stream1_IRQn
#define UART3_TX_DMA_STREAM     DMA1_Stream3
#define UART3_TX_DMA_CHANNEL    DMA_Channel_4
#define UART3_TX_DMA_STREAM_IRQn DMA1_Stream3_IRQn
#define UART3_STREAM_TIMEOUT_TICKS  pdMS_TO_TICKS(100u)
#define UART3_TX_DMA_BUFFER_SIZE (64u)

static volatile uint8_t s_dma_buffer[UART3_DMA_RX_BUFFER_SIZE]; /* DMA硬件写入 */
static volatile uint8_t s_ring_buffer[UART3_DMA_RING_SIZE];     /* ISR写、任务读 */

/*
 * SPSC所有权约束：DMA/USART ISR只推进head，通信任务只推进tail。
 * 该约束让正常push/pop不需要互斥锁；错误恢复时双方共享字段才进入临界区。
 */
static volatile uint16_t s_dma_last_pos;
static volatile uint16_t s_ring_head;
static volatile uint16_t s_ring_tail;
/* 所有错误计数均为累计值，溢出后自然回绕；用于诊断而不是控制判据。 */
static volatile uint32_t s_overflow_count;
static volatile uint32_t s_frame_count;
static volatile uint32_t s_crc_error_count;
static volatile uint32_t s_guo_frame_count;
static volatile uint32_t s_guo_error_count;
static volatile uint32_t s_guo_unsupported_count;
static volatile uint32_t s_dma_error_count;
static volatile uint32_t s_usart_error_count;
static volatile uint8_t s_rx_loss_pending;
static volatile uint8_t s_dma_recover_pending;
static TickType_t s_last_command_log_tick;
static volatile TaskHandle_t s_comm_task_handle;
static guo_proto_parser_t s_guo_parser;

/*
 * TX缓冲区必须是静态存储期：DMA启动后，发送任务的局部数组可能已经
 * 失效。TX只有一个生产者（MonitorTask），DMA ISR是唯一释放者。
 */
static uint8_t s_tx_buffer[UART3_TX_DMA_BUFFER_SIZE];
static volatile uint16_t s_tx_length;
static volatile uint8_t s_tx_busy;
static volatile uint32_t s_tx_frame_count;    /* DMA成功启动次数，不等于发送完成 */
static volatile uint32_t s_tx_complete_count; /* 收到TCIF3且NDTR为0的完成次数 */
static volatile uint32_t s_tx_drop_count;
static volatile uint32_t s_tx_error_count;

typedef enum
{
    UART3_STREAM_IDLE = 0, /* 尚未被任何帧头占用 */
    UART3_STREAM_LEGACY,  /* 已看到0x7B，后续字节只交给旧11字节解析器 */
    UART3_STREAM_GUO    /* 已看到0xAA，后续字节只交给AA55解析器 */
} uart3_stream_mode_t;

static uart3_stream_mode_t s_stream_mode;
static uint8_t s_legacy_count;
static uint8_t s_check_mode_last;
static TickType_t s_stream_started_tick;

static void reset_stream_parsers(void);

static void uart3_error_notify_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* 出现串口错误后半帧不再可信，任务会丢弃ring残留并重新等待帧头。 */
    s_usart_error_count++;
    s_rx_loss_pending = 1u;
    if (s_comm_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_comm_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void dma_disable_wait(void)
{
    uint32_t wait_count = 100000u;

    /* F4手册要求确认EN位真正清零后再改NDTR或重新配置Stream。 */
    DMA_Cmd(UART3_DMA_STREAM, DISABLE);
    while ((DMA_GetCmdStatus(UART3_DMA_STREAM) != DISABLE) &&
           (wait_count != 0u))
    {
        wait_count--;
    }
}

static uint16_t ring_next(uint16_t index)
{
    /* UART3_DMA_RING_SIZE为2的整数次幂，因此位与比取模更适合ISR。 */
    return (uint16_t)((index + 1u) & (UART3_DMA_RING_SIZE - 1u));
}

static void notify_task_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_comm_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_comm_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/*
 * 把DMA自上次处理点以来新增的字节搬入软件ring。
 * Copy bytes written by DMA since the previous service point.  If a transfer
 * complete flag is pending while NDTR has already returned to the same value,
 * force one full-buffer lap; otherwise an exact 256-byte lap would look like
 * "no movement" and be silently lost. */
static void drain_dma_to_ring_from_isr(uint8_t force_full_lap)
{
    uint16_t current_pos;
    uint16_t bytes_to_copy;

    current_pos = (uint16_t)(UART3_DMA_RX_BUFFER_SIZE -
                             DMA_GetCurrDataCounter(UART3_DMA_STREAM));
    if (current_pos >= UART3_DMA_RX_BUFFER_SIZE)
    {
        current_pos = 0u;
    }

    if (force_full_lap != 0u)
    {
        /* A TC flag proves at least one wrap since the previous service
         * point.  Include that complete lap plus any bytes after the wrap. */
        if (current_pos >= s_dma_last_pos)
        {
            bytes_to_copy = (uint16_t)(UART3_DMA_RX_BUFFER_SIZE +
                                       current_pos - s_dma_last_pos);
        }
        else
        {
            bytes_to_copy = (uint16_t)(UART3_DMA_RX_BUFFER_SIZE -
                                       s_dma_last_pos + current_pos);
        }
    }
    else if (current_pos >= s_dma_last_pos)
    {
        bytes_to_copy = (uint16_t)(current_pos - s_dma_last_pos);
    }
    else
    {
        bytes_to_copy = (uint16_t)(UART3_DMA_RX_BUFFER_SIZE -
                                   s_dma_last_pos + current_pos);
    }

    while (bytes_to_copy != 0u)
    {
        uint16_t next = ring_next(s_ring_head);

        if (next == s_ring_tail)
        {
            /* ring已满：保留旧数据并计数；随后任务丢弃整批，避免拼出伪合法帧。 */
            s_overflow_count++;
            s_rx_loss_pending = 1u;
        }
        else
        {
            s_ring_buffer[s_ring_head] = s_dma_buffer[s_dma_last_pos];
            s_ring_head = next;
        }

        s_dma_last_pos++;
        if (s_dma_last_pos >= UART3_DMA_RX_BUFFER_SIZE)
        {
            s_dma_last_pos = 0u;
        }
        bytes_to_copy--;
    }

    notify_task_from_isr();
}

static int ring_pop(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL || s_ring_tail == s_ring_head)
    {
        return 0;
    }

    /* 该函数只由通信任务调用，所以正常读路径无需进入临界区。 */
    tail = s_ring_tail;
    *byte = s_ring_buffer[tail];
    s_ring_tail = ring_next(tail);
    return 1;
}

/*
 * 如果任务消费ring期间ISR报告丢字节，立刻停止当前解析并清除半帧。
 * If an ISR reports a loss while the task is draining, stop parsing the
 * current batch immediately.  The next valid frame must begin after the
 * loss; otherwise a damaged prefix could accidentally pass a checksum. */
static int discard_if_rx_loss_pending(void)
{
    uint8_t pending;

    taskENTER_CRITICAL();
    pending = s_rx_loss_pending;
    if (pending != 0u)
    {
        s_rx_loss_pending = 0u;
        s_ring_tail = s_ring_head;
    }
    taskEXIT_CRITICAL();

    if (pending != 0u)
    {
        reset_stream_parsers();
        return 1;
    }
    return 0;
}

/*
 * 校验并应用原WHEELTEC 11字节命令，保持原上位机兼容。
 * 解码和atan()可能耗时，因此本函数只能在UART3_DMA_CommTask中调用。
 */
static void apply_legacy_command(const uint8_t *frame)
{
    int i;
    uint8_t xor_value = 0u;
    float vz;
    float new_move_x;
    float new_move_y;
    float new_move_z;

    for (i = 0; i < 9; i++)
    {
        xor_value ^= frame[i];
    }

    if (frame[0] != FRAME_HEADER || frame[10] != FRAME_TAIL ||
        xor_value != frame[9])
    {
        s_crc_error_count++;
        return;
    }

    if (Time_count < CONTROL_DELAY || Check != 0)
    {
        return;
    }

    /* 临界区外完成解码和阿克曼换算，避免长时间阻塞调度。 */
    new_move_x = XYZ_Target_Speed_transition(frame[3], frame[4]);
    new_move_y = XYZ_Target_Speed_transition(frame[5], frame[6]);
    vz = XYZ_Target_Speed_transition(frame[7], frame[8]);
    if (Car_Mode == Akm_Car)
    {
        new_move_z = Vz_to_Akm_Angle(new_move_x, vz);
    }
    else
    {
        new_move_z = vz;
    }

    /* 一次性发布Move_X/Y/Z，避免Balance_task读到一半新、一半旧的目标。 */
    taskENTER_CRITICAL();
    PS2_ON_Flag = 0u;
    Remote_ON_Flag = 0u;
    APP_ON_Flag = 0u;
    CAN_ON_Flag = 0u;
    Usart1_ON_Flag = 0u;
    Usart5_ON_Flag = 0u;
    command_lost_count = 0u;
    Move_X = new_move_x;
    Move_Y = new_move_y;
    Move_Z = new_move_z;
    taskEXIT_CRITICAL();

    s_frame_count++;
}

/* 显式读取小端int16，避免结构体填充和CPU字节序造成协议不一致。 */
static int16_t guo_read_i16(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8);
    return (int16_t)value;
}

/*
 * 应用已经通过帧头、长度和CRC校验的Guo速度命令。
 * payload单位为mm/s、mm/s、mrad/s；进入原控制链前换算为m/s和rad/s。
 * 阿克曼模式下Move_Z保存的是由Vz_to_Akm_Angle()得到的转向量，不是车体wz。
 * Apply a validated Guo velocity command in the same unit convention used
 * by the original WHEELTEC control task (m/s, m/s, rad/s/steering angle). */
static void apply_guo_command(const guo_proto_frame_t *frame)
{
    int16_t vx_mmps;
    int16_t vy_mmps;
    int16_t wz_mradps;
    float new_move_x;
    float new_move_y;
    float wz_radps;
    float new_move_z;

    if ((frame == NULL) ||
        (frame->message_id != GUO_PROTO_CMD_VEL) ||
        (frame->payload_length != GUO_PROTO_CMD_VEL_LEN) ||
        ((frame->flags & (uint8_t)~GUO_PROTO_FLAG_ACK_REQ) != 0u))
    {
        return;
    }

    if (Time_count < CONTROL_DELAY || Check != 0)
    {
        return;
    }

    vx_mmps = guo_read_i16(&frame->payload[0]);
    vy_mmps = guo_read_i16(&frame->payload[2]);
    wz_mradps = guo_read_i16(&frame->payload[4]);

    new_move_x = (float)vx_mmps * 0.001f;
    new_move_y = (float)vy_mmps * 0.001f;
    wz_radps = (float)wz_mradps * 0.001f;
    if (Car_Mode == Akm_Car)
    {
        new_move_z = Vz_to_Akm_Angle(new_move_x, wz_radps);
    }
    else
    {
        new_move_z = wz_radps;
    }

    /* 回调位于通信任务上下文，可安全执行浮点换算；仍用临界区保证整组发布。 */
    taskENTER_CRITICAL();
    PS2_ON_Flag = 0u;
    Remote_ON_Flag = 0u;
    APP_ON_Flag = 0u;
    CAN_ON_Flag = 0u;
    Usart1_ON_Flag = 0u;
    Usart5_ON_Flag = 0u;
    command_lost_count = 0u;
    Move_X = new_move_x;
    Move_Y = new_move_y;
    Move_Z = new_move_z;
    taskEXIT_CRITICAL();

    /* 最多每秒记录一次有效命令，防止20 Hz控制流淹没低速日志串口。 */
    if ((xTaskGetTickCount() - s_last_command_log_tick) >=
        pdMS_TO_TICKS(1000u))
    {
        s_last_command_log_tick = xTaskGetTickCount();
        /* 只投递原始协议值，格式化工作交给LogTask。 */
        (void)GuoLog_Post(AKM_LOG_EVENT_COMMAND_OK,
                            (int32_t)vx_mmps,
                            (int32_t)vy_mmps,
                            (int32_t)wz_mradps);
    }
}

static void guo_on_frame(const guo_proto_frame_t *frame, void *context)
{
    (void)context;
    s_guo_frame_count++;
    /* 只实现速度指令；未知消息、长度或flags仅计数，不允许改变运动目标。 */
    if ((frame == NULL) ||
        (frame->message_id != GUO_PROTO_CMD_VEL) ||
        (frame->payload_length != GUO_PROTO_CMD_VEL_LEN) ||
        ((frame->flags & (uint8_t)~GUO_PROTO_FLAG_ACK_REQ) != 0u))
    {
        s_guo_unsupported_count++;
        return;
    }
    apply_guo_command(frame);
}

static void feed_check_parser(uint8_t byte)
{
    /* 保留原工程自检模式的ASCII回显通道，不走运动协议解析。 */
    if (message_count < (uint8_t)(sizeof(uart3_receive_message) - 1u))
    {
        uart3_receive_message[message_count] = byte;
        message_count++;
    }

    if (byte == '#')
    {
        uart3_receive_message[message_count] = '\0';
        uart3_send_flag = 1u;
    }
}

static void reset_stream_parsers(void)
{
    /* 同时释放协议所有权并清空两个解析器的半帧状态。 */
    s_stream_mode = UART3_STREAM_IDLE;
    s_legacy_count = 0u;
    s_check_mode_last = 0u;
    s_stream_started_tick = 0u;
    GuoProto_Reset(&s_guo_parser);
}

static int feed_legacy_parser(uint8_t byte);

static void start_stream_mode(uart3_stream_mode_t mode)
{
    s_stream_mode = mode;
    s_stream_started_tick = xTaskGetTickCount();
}

/*
 * 错误字节本身也可能是下一帧帧头，因此复位后需要重新喂入一次。
 * Re-feed a byte which can itself be the first byte of the next frame.  This
 * matters when a CRC/EOF error is immediately followed by a valid AA55
 * packet; dropping that AA would otherwise lose the next command. */
static void restart_from_header_byte(uint8_t byte)
{
    if (byte == FRAME_HEADER)
    {
        start_stream_mode(UART3_STREAM_LEGACY);
        (void)feed_legacy_parser(byte);
    }
    else if (byte == GUO_PROTO_SOF0)
    {
        start_stream_mode(UART3_STREAM_GUO);
        (void)GuoProto_FeedByte(&s_guo_parser, byte);
    }
}

static int feed_legacy_parser(uint8_t byte)
{
    static uint8_t frame[RECEIVE_DATA_SIZE];

    if (Time_count < CONTROL_DELAY)
    {
        s_legacy_count = 0u;
        return 0;
    }

    if (Check != 0)
    {
        s_legacy_count = 0u;
        feed_check_parser(byte);
        return 0;
    }

    if (s_legacy_count == 0u)
    {
        if (byte != FRAME_HEADER)
        {
            return 0;
        }
    }

    /* 帧内再次出现0x7B按普通数据处理，避免合法高字节0x7B导致错误重启。 */
    frame[s_legacy_count++] = byte;

    if (s_legacy_count == RECEIVE_DATA_SIZE)
    {
        apply_legacy_command(frame);
        s_legacy_count = 0u;
        return 1;
    }
    return 0;
}

/*
 * 帧头仲裁：一旦识别0x7B或AA55候选头，本帧只交给对应解析器。
 * 这样AA55 payload中的0x7B不会被误解释成旧运动命令。
 * Select one protocol after seeing its distinct start byte.  This prevents a
 * 0x7B in an AA55 payload (or vice versa) from being interpreted by the other
 * decoder and changing the motion target accidentally. */
static void feed_stream_byte(uint8_t byte)
{
    guo_proto_result_t result;

    if (s_stream_mode != UART3_STREAM_IDLE &&
        ((xTaskGetTickCount() - s_stream_started_tick) >
         (TickType_t)UART3_STREAM_TIMEOUT_TICKS))
    {
        /* A truncated/noisy frame must not permanently select one protocol. */
        reset_stream_parsers();
    }

    /* Match the original firmware: no command bytes are accepted during the
     * ten-second IMU/vehicle startup calibration window. */
    if (Time_count < CONTROL_DELAY)
    {
        reset_stream_parsers();
        return;
    }

    if (Check != 0)
    {
        s_check_mode_last = 1u;
        start_stream_mode(UART3_STREAM_LEGACY);
        (void)feed_legacy_parser(byte);
        return;
    }

    if (s_check_mode_last != 0u)
    {
        reset_stream_parsers();
        s_check_mode_last = 0u;
    }

    if (s_stream_mode == UART3_STREAM_LEGACY)
    {
        if (feed_legacy_parser(byte) != 0)
        {
            reset_stream_parsers();
        }
        return;
    }

    if (s_stream_mode == UART3_STREAM_GUO)
    {
        if ((s_guo_parser.state == GUO_PROTO_WAIT_SOF1) &&
            (byte == FRAME_HEADER))
        {
            /* A stray AA followed by a legacy header: abandon the candidate
             * AA55 frame and let the unambiguous 7B frame start. */
            reset_stream_parsers();
            start_stream_mode(UART3_STREAM_LEGACY);
            (void)feed_legacy_parser(byte);
            return;
        }
        result = GuoProto_FeedByte(&s_guo_parser, byte);
        if (result != GUO_PROTO_NEED_MORE)
        {
            if ((result == GUO_PROTO_ERR_VERSION) ||
                (result == GUO_PROTO_ERR_LENGTH) ||
                (result == GUO_PROTO_ERR_CRC) ||
                (result == GUO_PROTO_ERR_EOF))
            {
                s_guo_error_count++;
            }
            reset_stream_parsers();
            restart_from_header_byte(byte);
        }
        else if (s_guo_parser.state == GUO_PROTO_WAIT_SOF0)
        {
            /* The parser rejected an overlapping AA candidate and already
             * resynchronized; release protocol ownership immediately. */
            reset_stream_parsers();
            restart_from_header_byte(byte);
        }
        return;
    }

    /* Idle: AA starts Guo; 7B starts the fixed-length WHEELTEC frame. */
    if (byte == FRAME_HEADER)
    {
        start_stream_mode(UART3_STREAM_LEGACY);
        (void)feed_legacy_parser(byte);
    }
    else
    {
        result = GuoProto_FeedByte(&s_guo_parser, byte);
        if (result == GUO_PROTO_NEED_MORE)
        {
            if (s_guo_parser.state != GUO_PROTO_WAIT_SOF0)
            {
                start_stream_mode(UART3_STREAM_GUO);
            }
        }
        else if ((result == GUO_PROTO_ERR_VERSION) ||
                 (result == GUO_PROTO_ERR_LENGTH) ||
                 (result == GUO_PROTO_ERR_CRC) ||
                 (result == GUO_PROTO_ERR_EOF))
        {
            s_guo_error_count++;
            restart_from_header_byte(byte);
        }
    }
}

void UART3_DMA_Init(void)
{
    DMA_InitTypeDef dma_init;
    NVIC_InitTypeDef nvic_init;
    volatile uint32_t usart_status;
    volatile uint32_t usart_data;

    /* 先清软件状态和累计计数，再开启任何中断，避免启动阶段读取脏状态。 */
    s_dma_last_pos = 0u;
    s_ring_head = 0u;
    s_ring_tail = 0u;
    s_overflow_count = 0u;
    s_frame_count = 0u;
    s_crc_error_count = 0u;
    s_guo_frame_count = 0u;
    s_guo_error_count = 0u;
    s_guo_unsupported_count = 0u;
    s_dma_error_count = 0u;
    s_usart_error_count = 0u;
    s_rx_loss_pending = 0u;
    s_dma_recover_pending = 0u;
    s_comm_task_handle = NULL;
    s_tx_length = 0u;
    s_tx_busy = 0u;
    s_tx_frame_count = 0u;
    s_tx_complete_count = 0u;
    s_tx_drop_count = 0u;
    s_tx_error_count = 0u;
    s_stream_mode = UART3_STREAM_IDLE;
    s_legacy_count = 0u;
    s_check_mode_last = 0u;
    s_stream_started_tick = 0u;
    GuoProto_Init(&s_guo_parser, guo_on_frame, NULL);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    dma_disable_wait();
    DMA_DeInit(UART3_DMA_STREAM);

    /* 外设地址固定为USART3->DR，内存地址递增，字节宽、循环模式。 */
    dma_init.DMA_Channel = UART3_DMA_CHANNEL;
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    dma_init.DMA_Memory0BaseAddr = (uint32_t)s_dma_buffer;
    dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma_init.DMA_BufferSize = UART3_DMA_RX_BUFFER_SIZE;
    dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma_init.DMA_Mode = DMA_Mode_Circular;
    dma_init.DMA_Priority = DMA_Priority_High;
    dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(UART3_DMA_STREAM, &dma_init);

    DMA_ClearFlag(UART3_DMA_STREAM, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 |
                  DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
    DMA_ITConfig(UART3_DMA_STREAM,
                 DMA_IT_HT | DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE,
                 ENABLE);

    /* 优先级6满足FreeRTOS FromISR API要求：数值不得小于本工程阈值5。 */
    nvic_init.NVIC_IRQChannel = UART3_DMA_STREAM_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 6u;
    nvic_init.NVIC_IRQChannelSubPriority = 0u;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    /* RXNE is no longer consumed by the USART ISR while DMA is active. */
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

    /* Discard a stale byte/IDLE condition left by the old RXNE path. */
    usart_status = USART3->SR;
    usart_data = USART3->DR;
    (void)usart_status;
    (void)usart_data;

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_ITConfig(USART3, USART_IT_ERR, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    DMA_Cmd(UART3_DMA_STREAM, ENABLE);

#if AKM_UART3_TX_DMA_ENABLE
    /*
     * USART3_TX在F407映射到DMA1 Stream3/Channel4。RX使用Stream1，两个
     * Stream独立配置，不能把TX错误恢复误操作到RX流。
     */
    DMA_DeInit(UART3_TX_DMA_STREAM);
    dma_init.DMA_Channel = UART3_TX_DMA_CHANNEL;
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    dma_init.DMA_Memory0BaseAddr = (uint32_t)s_tx_buffer;
    dma_init.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    /* 初始NDTR使用合法非零值，真正发送前由TxTrySend改成实际长度。 */
    dma_init.DMA_BufferSize = UART3_TX_DMA_BUFFER_SIZE;
    dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma_init.DMA_Mode = DMA_Mode_Normal;
    dma_init.DMA_Priority = DMA_Priority_Medium;
    dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(UART3_TX_DMA_STREAM, &dma_init);
    /* 显式关闭可能由旧固件遗留的FIFO错误中断使能位。 */
    UART3_TX_DMA_STREAM->FCR &= (uint32_t)~DMA_SxFCR_FEIE;
    DMA_ClearFlag(UART3_TX_DMA_STREAM, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                  DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);
    /*
     * TX使用Direct Mode时只打开TC/TE/DME。
     * C30D实测Direct Mode会置FEIF3；若同时打开FEIE，会形成中断风暴，
     * 使一个状态帧产生多次“FIFO错误”并干扰USART连续发送。FEIF3在每次
     * 启动和收尾时作为普通状态位清除，不把它配置为中断源。
     */
    DMA_ITConfig(UART3_TX_DMA_STREAM,
                 DMA_IT_TC | DMA_IT_TE | DMA_IT_DME, ENABLE);

    nvic_init.NVIC_IRQChannel = UART3_TX_DMA_STREAM_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 6u;
    nvic_init.NVIC_IRQChannelSubPriority = 1u;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    /* 发送请求保持关闭，直到UART3_DMA_TxTrySend装载一整帧。 */
    USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
#endif
}

void UART3_DMA_CommTask(void *argument)
{
    uint8_t byte;
    uint8_t recover_pending;
    uint8_t loss_pending;

    /* 保存任务句柄后，ISR才能通过任务通知唤醒本任务。 */
    (void)argument;
    s_comm_task_handle = xTaskGetCurrentTaskHandle();

    for (;;)
    {
		AKM_MONITOR_HEARTBEAT(AKM_MONITOR_HB_UART3_DMA);
        /* Snapshot-and-clear the ISR flags atomically.  An ISR arriving
         * between a plain read and clear must be handled on the next loop,
         * not accidentally erased. */
        taskENTER_CRITICAL();
        recover_pending = s_dma_recover_pending;
        loss_pending = s_rx_loss_pending;
        s_dma_recover_pending = 0u;
        s_rx_loss_pending = 0u;
        taskEXIT_CRITICAL();

        if (recover_pending != 0u)
        {
            loss_pending = 1u;
            /* A transfer/FIFO error can stop a stream.  Re-arm it from task
             * context, after dropping the partial byte stream. */
            taskENTER_CRITICAL();
            dma_disable_wait();
            DMA_ClearFlag(UART3_DMA_STREAM, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 |
                          DMA_FLAG_TEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
            DMA_SetCurrDataCounter(UART3_DMA_STREAM,
                                   UART3_DMA_RX_BUFFER_SIZE);
            s_dma_last_pos = 0u;
            DMA_Cmd(UART3_DMA_STREAM, ENABLE);
            s_ring_tail = s_ring_head;
            taskEXIT_CRITICAL();
        }

        if (loss_pending != 0u)
        {
            /* A dropped byte can leave either parser mid-frame.  Discard the
             * partial frame and any bytes queued around it; wait for a fresh
             * header rather than allowing a damaged frame to pass by chance. */
            taskENTER_CRITICAL();
            s_ring_tail = s_ring_head;
            taskEXIT_CRITICAL();
            reset_stream_parsers();
        }

        /* Drain once before blocking too; bytes may arrive before task start. */
        while (ring_pop(&byte) != 0)
        {
            if (discard_if_rx_loss_pending() != 0)
            {
                break;
            }
            feed_stream_byte(byte);
        }

        /* 任务通知只表示“可能有新数据”，真正字节始终保存在ring中。 */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void UART3_DMA_IdleFromISR(void)
{
    uint8_t force_full_lap = 0u;

    /* IDLE到来时若TC也挂起，说明DMA至少完整绕过一圈，必须补算整圈。 */
    if (DMA_GetITStatus(UART3_DMA_STREAM, DMA_IT_TCIF1) != RESET)
    {
        force_full_lap = 1u;
        DMA_ClearITPendingBit(UART3_DMA_STREAM, DMA_IT_TCIF1);
    }
    drain_dma_to_ring_from_isr(force_full_lap);
}

void UART3_DMA_UsartErrorFromISR(void)
{
    uart3_error_notify_from_isr();
}

void UART3_DMA_StreamIRQHandler(void)
{
    uint8_t error_pending = 0u;
    uint8_t ht_pending;
    uint8_t tc_pending;
    uint8_t force_full_lap;

    /* TE/DME/FE出现后只在ISR置位；重启DMA等较重操作留给通信任务。 */
    if (DMA_GetITStatus(UART3_DMA_STREAM, DMA_IT_TEIF1) != RESET)
    {
        DMA_ClearITPendingBit(UART3_DMA_STREAM, DMA_IT_TEIF1);
        s_dma_error_count++;
        s_dma_recover_pending = 1u;
        s_rx_loss_pending = 1u;
        error_pending = 1u;
    }

    if (DMA_GetITStatus(UART3_DMA_STREAM, DMA_IT_DMEIF1) != RESET)
    {
        DMA_ClearITPendingBit(UART3_DMA_STREAM, DMA_IT_DMEIF1);
        s_dma_error_count++;
        s_dma_recover_pending = 1u;
        s_rx_loss_pending = 1u;
        error_pending = 1u;
    }

    if (DMA_GetITStatus(UART3_DMA_STREAM, DMA_IT_FEIF1) != RESET)
    {
        DMA_ClearITPendingBit(UART3_DMA_STREAM, DMA_IT_FEIF1);
        s_dma_error_count++;
        s_dma_recover_pending = 1u;
        s_rx_loss_pending = 1u;
        error_pending = 1u;
    }

    ht_pending = (DMA_GetITStatus(UART3_DMA_STREAM, DMA_IT_HTIF1) != RESET) ? 1u : 0u;
    tc_pending = (DMA_GetITStatus(UART3_DMA_STREAM, DMA_IT_TCIF1) != RESET) ? 1u : 0u;

    /* Service TC first.  It proves at least one full DMA-buffer lap; handling
     * HT afterwards only copies bytes that arrived beyond that TC snapshot. */
    if (tc_pending != 0u)
    {
        force_full_lap = 1u;
        DMA_ClearITPendingBit(UART3_DMA_STREAM, DMA_IT_TCIF1);
        drain_dma_to_ring_from_isr(force_full_lap);
    }

    if (ht_pending != 0u)
    {
        DMA_ClearITPendingBit(UART3_DMA_STREAM, DMA_IT_HTIF1);
        drain_dma_to_ring_from_isr(0u);
    }

    if (error_pending != 0u)
    {
        notify_task_from_isr();
    }
}

/* 诊断getter只读取32位累计值，STM32F407可原子读取对齐的32位变量。 */
uint32_t UART3_DMA_GetOverflowCount(void)
{
    return s_overflow_count;
}

uint32_t UART3_DMA_GetFrameCount(void)
{
    return s_frame_count;
}

uint32_t UART3_DMA_GetCheckErrorCount(void)
{
    return s_crc_error_count;
}

uint32_t UART3_DMA_GetGuoFrameCount(void)
{
    return s_guo_frame_count;
}

uint32_t UART3_DMA_GetGuoErrorCount(void)
{
    return s_guo_error_count;
}

uint32_t UART3_DMA_GetGuoUnsupportedCount(void)
{
    return s_guo_unsupported_count;
}

uint32_t UART3_DMA_GetDmaErrorCount(void)
{
    return s_dma_error_count;
}

uint32_t UART3_DMA_GetUsartErrorCount(void)
{
    return s_usart_error_count;
}

#if AKM_UART3_TX_DMA_ENABLE

uint8_t UART3_DMA_TxTrySend(const uint8_t *data, uint16_t length)
{
    uint8_t accepted = 0u;
    uint32_t wait_count;

    if ((data == NULL) || (length == 0u) ||
        (length > UART3_TX_DMA_BUFFER_SIZE))
    {
        return 0u;
    }

    /*
     * 复制和启动过程放在临界区内，防止DMA TC ISR在busy检查与启动之间
     * 插入。发送很短（当前34字节），临界区不会参与控制闭环计算。
     */
    taskENTER_CRITICAL();
    if (s_tx_busy == 0u)
    {
        memcpy(s_tx_buffer, data, length);
        s_tx_length = length;
        DMA_Cmd(UART3_TX_DMA_STREAM, DISABLE);
        wait_count = 100000u;
        while ((DMA_GetCmdStatus(UART3_TX_DMA_STREAM) != DISABLE) &&
               (wait_count != 0u))
        {
            /* Stream长度很短，等待硬件确认停机，避免改写NDTR竞态。 */
            wait_count--;
        }
        if (wait_count == 0u)
        {
            /* DMA异常卡在EN状态时不能无限期阻塞MonitorTask。 */
            s_tx_length = 0u;
            s_tx_error_count++;
            taskEXIT_CRITICAL();
            return 0u;
        }
        DMA_ClearFlag(UART3_TX_DMA_STREAM, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                      DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);
        UART3_TX_DMA_STREAM->FCR &= (uint32_t)~DMA_SxFCR_FEIE;
        DMA_SetCurrDataCounter(UART3_TX_DMA_STREAM, length);
        s_tx_busy = 1u;
        s_tx_frame_count++;
        /* 先使能Stream，再允许USART发出DMA请求，避免首个TXE请求竞态。 */
        DMA_Cmd(UART3_TX_DMA_STREAM, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
        accepted = 1u;
    }
    else
    {
        /* 非阻塞策略：不排队、不等待，下一次周期再尝试。 */
        s_tx_drop_count++;
    }
    taskEXIT_CRITICAL();

    return accepted;
}

uint8_t UART3_DMA_TxIsBusy(void)
{
    return s_tx_busy;
}

uint32_t UART3_DMA_GetTxFrameCount(void)
{
    return s_tx_frame_count;
}

uint32_t UART3_DMA_GetTxCompleteCount(void)
{

    return s_tx_complete_count;
}

uint32_t UART3_DMA_GetTxDropCount(void)
{
    return s_tx_drop_count;
}

uint32_t UART3_DMA_GetTxErrorCount(void)
{
    return s_tx_error_count;
}

void UART3_DMA_TxStreamIRQHandler(void)
{
    uint8_t transfer_complete;
    uint8_t transfer_error;

    /*
     * 先读取原始状态，再统一清除Stream3全部标志。TCIF3且NDTR为0是本
     * 模块的成功判据；Direct Mode伴随出现的FEIF3不再覆盖正常完成。
     */
    transfer_complete =
        ((DMA_GetFlagStatus(UART3_TX_DMA_STREAM, DMA_FLAG_TCIF3) != RESET) &&
         (DMA_GetCurrDataCounter(UART3_TX_DMA_STREAM) == 0u)) ? 1u : 0u;
    transfer_error =
        ((DMA_GetFlagStatus(UART3_TX_DMA_STREAM, DMA_FLAG_TEIF3) != RESET) ||
         (DMA_GetFlagStatus(UART3_TX_DMA_STREAM, DMA_FLAG_DMEIF3) != RESET))
            ? 1u : 0u;

    DMA_ClearFlag(UART3_TX_DMA_STREAM, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                  DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);

    if (transfer_complete != 0u)
    {
        s_tx_complete_count++;
    }
    else if (transfer_error != 0u)
    {
        s_tx_error_count++;
    }
    else
    {
        /* 无TC/TE/DME时不释放busy，防止未知中断把活动帧误判为结束。 */
        return;
    }

    DMA_Cmd(UART3_TX_DMA_STREAM, DISABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
    s_tx_length = 0u;
    s_tx_busy = 0u;
}

#else /* AKM_UART3_TX_DMA_ENABLE */

uint8_t UART3_DMA_TxTrySend(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return 0u;
}
uint8_t UART3_DMA_TxIsBusy(void) { return 0u; }
uint32_t UART3_DMA_GetTxFrameCount(void) { return 0u; }
uint32_t UART3_DMA_GetTxCompleteCount(void) { return 0u; }
uint32_t UART3_DMA_GetTxDropCount(void) { return 0u; }
uint32_t UART3_DMA_GetTxErrorCount(void) { return 0u; }
void UART3_DMA_TxStreamIRQHandler(void) {}

#endif /* AKM_UART3_TX_DMA_ENABLE */

#else /* AKM_UART3_DMA_ENABLE */

/* 关闭迁移功能时保留同名空实现，避免上层大量条件编译。 */
void UART3_DMA_Init(void) {}
void UART3_DMA_CommTask(void *argument) { (void)argument; vTaskDelete(NULL); }
void UART3_DMA_IdleFromISR(void) {}
void UART3_DMA_UsartErrorFromISR(void) {}
void UART3_DMA_StreamIRQHandler(void) {}
uint32_t UART3_DMA_GetOverflowCount(void) { return 0u; }
uint32_t UART3_DMA_GetFrameCount(void) { return 0u; }
uint32_t UART3_DMA_GetCheckErrorCount(void) { return 0u; }
uint32_t UART3_DMA_GetGuoFrameCount(void) { return 0u; }
uint32_t UART3_DMA_GetGuoErrorCount(void) { return 0u; }
uint32_t UART3_DMA_GetGuoUnsupportedCount(void) { return 0u; }
uint32_t UART3_DMA_GetDmaErrorCount(void) { return 0u; }
uint32_t UART3_DMA_GetUsartErrorCount(void) { return 0u; }
uint8_t UART3_DMA_TxTrySend(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return 0u;
}
uint8_t UART3_DMA_TxIsBusy(void) { return 0u; }
uint32_t UART3_DMA_GetTxFrameCount(void) { return 0u; }
uint32_t UART3_DMA_GetTxCompleteCount(void) { return 0u; }
uint32_t UART3_DMA_GetTxDropCount(void) { return 0u; }
uint32_t UART3_DMA_GetTxErrorCount(void) { return 0u; }
void UART3_DMA_TxStreamIRQHandler(void) {}

#endif /* AKM_UART3_DMA_ENABLE */
