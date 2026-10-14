#ifndef __KAOYA_PROTOCOL_H
#define __KAOYA_PROTOCOL_H

/*
 * Small, hardware-independent decoder for the KaoYa binary protocol.
 *
 * Frame format (little endian multi-byte fields):
 *   AA 55 | VER MSG FLAGS SEQ LEN_L LEN_H | PAYLOAD | CRC_L CRC_H | 66 BB
 * CRC16/CCITT-FALSE covers VER through the last payload byte.  The decoder
 * deliberately has no FreeRTOS or motor-control dependency so it can be
 * unit-tested on a host and reused by another UART later.
 *
 * 中文说明：本模块只关心“字节流 -> 完整协议帧”，不依赖串口、DMA、
 * FreeRTOS和电机控制。调用者逐字节喂入数据，只有版本、长度、帧尾和CRC
 * 全部正确时才触发回调，因此错误帧不会直接进入运动控制层。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KAOYA_PROTO_SOF0             (0xAAu)
#define KAOYA_PROTO_SOF1             (0x55u)
#define KAOYA_PROTO_EOF0             (0x66u)
#define KAOYA_PROTO_EOF1             (0xBBu)
#define KAOYA_PROTO_VERSION          (0x01u)
#define KAOYA_PROTO_CMD_VEL          (0x01u)
#define KAOYA_PROTO_CMD_VEL_LEN      (6u)
#define KAOYA_PROTO_FLAG_ACK_REQ     (0x01u)
/* 限制payload可防止异常长度越界写入frame.payload。 */
#define KAOYA_PROTO_MAX_PAYLOAD      (64u)

typedef struct
{
    uint8_t  version;        /* 协议版本，当前固定为1 */
    uint8_t  message_id;     /* 消息类型，例如0x01表示速度指令 */
    uint8_t  flags;          /* 位标志，当前仅支持ACK_REQ */
    uint8_t  sequence;       /* 8位帧序号，由发送方维护 */
    uint16_t payload_length; /* 小端长度字段解析后的主机值 */
    uint8_t  payload[KAOYA_PROTO_MAX_PAYLOAD];
} kaoya_proto_frame_t;

typedef void (*kaoya_proto_frame_callback_t)(const kaoya_proto_frame_t *frame,
                                             void *context);

typedef enum
{
    /* 每个枚举值对应当前期望接收的下一个字段。 */
    KAOYA_PROTO_WAIT_SOF0 = 0,
    KAOYA_PROTO_WAIT_SOF1,
    KAOYA_PROTO_READ_VERSION,
    KAOYA_PROTO_READ_MESSAGE,
    KAOYA_PROTO_READ_FLAGS,
    KAOYA_PROTO_READ_SEQUENCE,
    KAOYA_PROTO_READ_LENGTH_LOW,
    KAOYA_PROTO_READ_LENGTH_HIGH,
    KAOYA_PROTO_READ_PAYLOAD,
    KAOYA_PROTO_READ_CRC_LOW,
    KAOYA_PROTO_READ_CRC_HIGH,
    KAOYA_PROTO_READ_EOF0,
    KAOYA_PROTO_READ_EOF1
} kaoya_proto_state_t;

typedef struct
{
    kaoya_proto_state_t state;       /* 当前状态机位置 */
    kaoya_proto_frame_t frame;       /* 正在组装的帧 */
    uint16_t payload_offset;         /* 已收payload字节数 */
    uint16_t crc_calculated;         /* 边接收边累计的CRC */
    uint16_t crc_received;           /* 帧中携带的CRC */
    kaoya_proto_frame_callback_t callback; /* 完整合法帧回调 */
    void *context;                   /* 原样传给回调的用户上下文 */
} kaoya_proto_parser_t;

/* Return values from KaoyaProto_FeedByte. */
typedef enum
{
    KAOYA_PROTO_NEED_MORE = 0,
    KAOYA_PROTO_FRAME_OK = 1,
    KAOYA_PROTO_ERR_VERSION = -1,
    KAOYA_PROTO_ERR_LENGTH = -2,
    KAOYA_PROTO_ERR_CRC = -3,
    KAOYA_PROTO_ERR_EOF = -4
} kaoya_proto_result_t;

void KaoyaProto_Init(kaoya_proto_parser_t *parser,
                     kaoya_proto_frame_callback_t callback,
                     void *context);
/* 丢弃半帧并重新等待AA 55帧头。 */
void KaoyaProto_Reset(kaoya_proto_parser_t *parser);

/* 每调用一次输入一个字节，返回“继续等待、成功一帧或具体错误”。 */
kaoya_proto_result_t KaoyaProto_FeedByte(kaoya_proto_parser_t *parser,
                                          uint8_t byte);

/* Feed a byte array and return the number of complete valid frames. */
uint16_t KaoyaProto_Feed(kaoya_proto_parser_t *parser,
                          const uint8_t *data,
                          uint16_t length);

/* CRC16/CCITT-FALSE：poly=0x1021，init=0xFFFF，无反射，无最终异或。 */
uint16_t KaoyaProto_Crc16CcittFalse(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __KAOYA_PROTOCOL_H */
