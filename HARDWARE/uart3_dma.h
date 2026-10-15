#ifndef __UART3_DMA_H
#define __UART3_DMA_H

/*
 * USART3 DMA receiver/transmitter for the STM32F407VET6 migration copy.
 *
 * The original WHEELTEC firmware receives the 11-byte 0x7B...0x7D
 * command in the USART3 RXNE ISR.  This module moves only the byte
 * acquisition into DMA + IDLE and leaves the legacy frame format intact.
 * A task notification wakes the parser task; no motor-control code runs
 * from an interrupt.
 *
 * 中文数据链：
 * USART3->DR -> DMA循环缓冲区 -> IDLE/HT/TC中断 -> 软件环形缓冲区
 * -> FreeRTOS任务通知 -> UART3_DMA_CommTask协议解析 -> 发布运动目标。
 * 中断只负责搬运字节和通知任务，CRC、浮点换算和阿克曼运动学均在任务中执行。
 * 状态上报则使用独立的DMA1 Stream3/Channel4，RX Stream1与TX Stream3互不
 * 改写；TX缓冲区为静态数组，busy期间不排队，避免控制链被日志阻塞。
 */

#include "stm32f4xx.h"
#include <stdint.h>
#include "guo_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef AKM_UART3_DMA_ENABLE
/* 置0可回退到原USART3 RXNE逐字节中断接收路径，便于现场排查。 */
#define AKM_UART3_DMA_ENABLE 1
#endif

#ifndef AKM_UART3_TX_DMA_ENABLE
/* USART3发送也使用DMA；0可临时恢复旧的阻塞发送代码。 */
#define AKM_UART3_TX_DMA_ENABLE 1
#endif

/* 环形缓冲区长度必须保持为2的整数次幂，ring_next()使用位与实现回绕。 */
#define UART3_DMA_RX_BUFFER_SIZE  (256u)
#define UART3_DMA_RING_SIZE       (512u)
#define UART3_DMA_TASK_PRIO       (4u)
#define UART3_DMA_TASK_STK_SIZE   (512u)

/* 初始化DMA1 Stream1/Channel4、相关中断和协议解析器。 */
void UART3_DMA_Init(void);

/* 通信任务入口：从软件ring取数并在任务上下文解析协议。 */
void UART3_DMA_CommTask(void *argument);

/* USART3_IRQHandler清除IDLE条件后调用；本函数运行在中断上下文。 */
void UART3_DMA_IdleFromISR(void);

/* USART3_IRQHandler清除ORE/FE/NE错误后调用；只计数并通知任务恢复。 */
void UART3_DMA_UsartErrorFromISR(void);

/* DMA1_Stream1_IRQHandler转调入口，处理HT/TC及DMA错误。 */
void UART3_DMA_StreamIRQHandler(void);

/* 以下接口只返回累计诊断计数，可供OLED、MonitorTask或调试器读取。 */
uint32_t UART3_DMA_GetOverflowCount(void);
uint32_t UART3_DMA_GetFrameCount(void);
uint32_t UART3_DMA_GetCheckErrorCount(void);

/* Guo AA55/CRC16解析及底层DMA/USART错误累计值。 */
uint32_t UART3_DMA_GetGuoFrameCount(void);
uint32_t UART3_DMA_GetGuoErrorCount(void);
uint32_t UART3_DMA_GetGuoUnsupportedCount(void);
uint32_t UART3_DMA_GetDmaErrorCount(void);
uint32_t UART3_DMA_GetUsartErrorCount(void);

/*
 * USART3 TX DMA接口：函数只在任务上下文调用，把数据复制到静态缓冲区后
 * 立即返回。返回1表示DMA已启动，返回0表示参数非法或上一次发送仍未完成。
 * ISR只负责释放busy并累计完成/错误计数，绝不访问调用者的栈缓冲区。
 */
uint8_t UART3_DMA_TxTrySend(const uint8_t *data, uint16_t length);
uint8_t UART3_DMA_TxIsBusy(void);
uint32_t UART3_DMA_GetTxFrameCount(void);
uint32_t UART3_DMA_GetTxCompleteCount(void);
uint32_t UART3_DMA_GetTxDropCount(void);
uint32_t UART3_DMA_GetTxErrorCount(void);
void UART3_DMA_TxStreamIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART3_DMA_H */
