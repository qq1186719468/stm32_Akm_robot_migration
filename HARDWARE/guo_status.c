#include "kaoya_status.h"

/* 显式写入小端uint16，避免直接memcpy结构体带来的填充和字节序问题。 */
static void write_u16(uint8_t *buffer, uint16_t *offset, uint16_t value)
{
    buffer[(*offset)++] = (uint8_t)(value & 0xFFu);
    buffer[(*offset)++] = (uint8_t)(value >> 8);
}

static void write_i16(uint8_t *buffer, uint16_t *offset, int16_t value)
{
    /* C中转为uint16保留二进制补码位模式，接收端再按int16解释。 */
    write_u16(buffer, offset, (uint16_t)value);
}

/*
 * 把已经完成单位换算的状态数据编码成固定34字节帧。
 * 注意：这里不读取全局传感器变量，保证“采集快照”和“协议编码”分层。
 */
uint16_t KaoyaStatus_BuildFrame(const kaoya_status_payload_t *status,
                                uint8_t sequence,
                                uint8_t *output,
                                uint16_t output_capacity)
{
    uint16_t offset = 0u;
    uint16_t crc;

    if ((status == 0) || (output == 0) ||
        (output_capacity < KAOYA_STATUS_FRAME_LENGTH))
    {
        return 0u;
    }

    /* 帧头和通用控制字段。 */
    output[offset++] = KAOYA_PROTO_SOF0;
    output[offset++] = KAOYA_PROTO_SOF1;
    output[offset++] = KAOYA_PROTO_VERSION;
    output[offset++] = KAOYA_STATUS_MESSAGE_ID;
    output[offset++] = 0u; /* flags */
    output[offset++] = sequence;
    write_u16(output, &offset, KAOYA_STATUS_PAYLOAD_LENGTH);

    /* payload中每个字段均为16位小端，顺序必须与上位机协议一致。 */
    write_u16(output, &offset, status->battery_mv);
    write_i16(output, &offset, status->temperature_centi);
    write_i16(output, &offset, status->vx_mmps);
    write_i16(output, &offset, status->vy_mmps);
    write_i16(output, &offset, status->vz_mradps);
    write_i16(output, &offset, status->gx_centi_dps);
    write_i16(output, &offset, status->gy_centi_dps);
    write_i16(output, &offset, status->gz_centi_dps);
    write_i16(output, &offset, status->ax_centi_g);
    write_i16(output, &offset, status->ay_centi_g);
    write_i16(output, &offset, status->az_centi_g);

    /* CRC覆盖VER至payload末尾，不包含AA55、CRC自身和66BB。 */
    crc = KaoyaProto_Crc16CcittFalse(&output[2],
                                     (uint16_t)(6u + KAOYA_STATUS_PAYLOAD_LENGTH));
    write_u16(output, &offset, crc);
    output[offset++] = KAOYA_PROTO_EOF0;
    output[offset++] = KAOYA_PROTO_EOF1;

    return offset;
}
