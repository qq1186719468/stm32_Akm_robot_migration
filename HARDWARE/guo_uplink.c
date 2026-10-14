#include "kaoya_uplink.h"

#include "akm_monitor.h"
#include "kaoya_status.h"
#include "kaoya_status_adapter.h"
#include "uart3_dma.h"

/*
 * MonitorTask每秒调用一次本函数。快照、单位适配和协议编码都在任务
 * 上下文执行，TX DMA只接管已经复制到静态缓冲区的数据，不阻塞控制环。
 */
uint8_t KaoyaUplink_TrySendLatest(void)
{
    static uint8_t sequence;
    akm_status_snapshot_t snapshot;
    kaoya_status_payload_t payload;
    uint8_t frame[KAOYA_STATUS_FRAME_LENGTH];
    uint16_t frame_length;

    AkmMonitor_GetSnapshot(&snapshot);
    if (KaoyaStatus_FromMonitorSnapshot(&snapshot, &payload) == 0u)
    {
        return 0u;
    }

    frame_length = KaoyaStatus_BuildFrame(&payload, sequence, frame,
                                          KAOYA_STATUS_FRAME_LENGTH);
    if (frame_length == 0u)
    {
        return 0u;
    }
    sequence++;
    return UART3_DMA_TxTrySend(frame, frame_length);
}
