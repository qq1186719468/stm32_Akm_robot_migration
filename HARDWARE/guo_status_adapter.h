#ifndef __GUO_STATUS_ADAPTER_H
#define __GUO_STATUS_ADAPTER_H

/*
 * C30D阿克曼车型的状态适配层：把MonitorTask的“已采集快照”转换为
 * Guo协议约定的有单位字段。这里不重新读取ADC、编码器或I2C。
 */
#include <stdint.h>
#include "akm_monitor.h"
#include "guo_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 成功返回1；空指针返回0。输出字段全部已做int16饱和。 */
uint8_t GuoStatus_FromMonitorSnapshot(
    const akm_status_snapshot_t *snapshot,
    guo_status_payload_t *payload);

#ifdef __cplusplus
}
#endif

#endif /* __GUO_STATUS_ADAPTER_H */
