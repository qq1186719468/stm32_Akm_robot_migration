#ifndef __KAOYA_STATUS_H
#define __KAOYA_STATUS_H

#include <stdint.h>

#include "kaoya_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 状态上报帧：AA55 + 6字节头字段 + 22字节payload + CRC16 + 66BB。
 * 当前模块只负责编码，不负责选择数据来源，也不直接启动UART DMA发送。
 */
#define KAOYA_STATUS_MESSAGE_ID       (0x81u)
#define KAOYA_STATUS_PAYLOAD_LENGTH   (22u)
#define KAOYA_STATUS_FRAME_LENGTH     (34u)

typedef struct
{
    uint16_t battery_mv;       /* 电池电压，mV */
    int16_t  temperature_centi;/* 温度，0.01摄氏度 */
    int16_t  vx_mmps;          /* 车体前向速度，mm/s */
    int16_t  vy_mmps;          /* 车体侧向速度，mm/s；阿克曼车通常填0 */
    int16_t  vz_mradps;        /* 车体偏航角速度，mrad/s，不能直接填Move_Z */
    int16_t  gx_centi_dps;     /* X轴角速度，0.01 deg/s */
    int16_t  gy_centi_dps;     /* Y轴角速度，0.01 deg/s */
    int16_t  gz_centi_dps;     /* Z轴角速度，0.01 deg/s */
    int16_t  ax_centi_g;       /* X轴加速度，0.01 g */
    int16_t  ay_centi_g;       /* Y轴加速度，0.01 g */
    int16_t  az_centi_g;       /* Z轴加速度，0.01 g */
} kaoya_status_payload_t;

/* 成功返回34，参数为空或输出空间不足返回0；所有16位字段显式按小端编码。 */
uint16_t KaoyaStatus_BuildFrame(const kaoya_status_payload_t *status,
                                uint8_t sequence,
                                uint8_t *output,
                                uint16_t output_capacity);

#ifdef __cplusplus
}
#endif

#endif /* __KAOYA_STATUS_H */
