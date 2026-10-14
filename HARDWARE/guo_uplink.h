#ifndef __KAOYA_UPLINK_H
#define __KAOYA_UPLINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 读取最近快照、完成单位适配、编码34字节AA55帧并尝试启动TX DMA。 */
uint8_t KaoyaUplink_TrySendLatest(void);

#ifdef __cplusplus
}
#endif

#endif /* __KAOYA_UPLINK_H */
