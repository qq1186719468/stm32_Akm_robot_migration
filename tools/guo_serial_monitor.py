"""Read and validate AA55 status frames from XCOM/USB-TTL.

Examples (PowerShell):
  python tools/kaoya_serial_monitor.py --port COM8 --seconds 30
  python tools/kaoya_serial_monitor.py --hex "AA 55 ... 66 BB"

The monitor is a host-side acceptance aid.  It does not claim that a valid
status frame proves that the RX command path, motor output, or real vehicle
motion is correct; those require the Keil variables and safety tests.
"""

from __future__ import print_function

import argparse
import struct
import sys
import time

from kaoya_frame_tool import crc16_ccitt_false, parse_hex_bytes, parse_status_frame


class StatusStreamDecoder(object):
    """处理USB串口可能产生的拆包、粘包和噪声字节。"""

    def __init__(self):
        self.buffer = bytearray()
        self.valid_count = 0
        self.invalid_count = 0
        self.sequence_gap_count = 0
        self.last_sequence = None

    def feed(self, data):
        frames = []
        self.buffer.extend(bytearray(data))
        while True:
            header = self.buffer.find(b"\xAA\x55")
            if header < 0:
                # 保留末尾AA，防止下一次数据以55开头时丢失跨块帧头。
                if self.buffer[-1:] == b"\xAA":
                    self.buffer = bytearray(b"\xAA")
                else:
                    self.buffer.clear()
                break
            if header:
                del self.buffer[:header]
            if len(self.buffer) < 8:
                break
            payload_length = self.buffer[6] | (self.buffer[7] << 8)
            if self.buffer[2] != 1 or self.buffer[3] != 0x81 or payload_length != 22:
                self.invalid_count += 1
                del self.buffer[0]
                continue
            total = 2 + 6 + payload_length + 2 + 2
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            decoded = parse_status_frame(frame)
            if decoded is None:
                self.invalid_count += 1
                continue
            self.valid_count += 1
            sequence = decoded["sequence"]
            if self.last_sequence is not None:
                distance = (sequence - self.last_sequence) & 0xFF
                if distance != 1:
                    self.sequence_gap_count += 1
            self.last_sequence = sequence
            frames.append(decoded)
        return frames


def print_status(decoded):
    """以便于面试/验收记录的单位打印一帧状态。"""
    print("seq={sequence:3d} battery={battery_mv}mV "
          "vx={vx_mmps}mm/s vy={vy_mmps}mm/s "
          "gx={gx_centi_dps}/100dps gy={gy_centi_dps}/100dps "
          "gz={gz_centi_dps}/100dps "
          "ax={ax_centi_g}/100g ay={ay_centi_g}/100g az={az_centi_g}/100g".format(**decoded))


def make_test_status_frame(sequence):
    """生成一帧测试状态，供主机侧拆包/粘包自测使用。"""
    payload = struct.pack("<Hhhhhhhhhhh", 12345, -10, 300, 0, 0,
                          1, -2, 3, 0, 0, 100)
    body = struct.pack("<BBBBH", 1, 0x81, 0, sequence & 0xFF,
                       len(payload)) + payload
    return (b"\xAA\x55" + body +
            struct.pack("<H", crc16_ccitt_false(body)) + b"\x66\xBB")


def run_self_test():
    """验证拆包、粘包、坏CRC重同步和序号跳变检测。"""
    first = make_test_status_frame(1)
    second = make_test_status_frame(2)
    decoder = StatusStreamDecoder()
    output = []
    for chunk in (first[:3], first[3:17], first[17:] + second):
        output.extend(decoder.feed(chunk))
    assert [item["sequence"] for item in output] == [1, 2]

    broken = bytearray(make_test_status_frame(3))
    broken[-3] ^= 1
    output = decoder.feed(bytes(broken) + make_test_status_frame(5))
    assert [item["sequence"] for item in output] == [5]
    assert decoder.invalid_count == 1 and decoder.sequence_gap_count == 1
    print("stream-self-test: PASS (split/sticky/bad-crc/resync/sequence-gap)")


def main():
    parser = argparse.ArgumentParser(description="AA55 status-frame acceptance monitor")
    parser.add_argument("--port", help="USB-TTL port, e.g. COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0,
                        help="stop after this time; 0 means Ctrl-C")
    parser.add_argument("--hex", dest="hex_text",
                        help="decode copied XCOM hex text instead of opening a port")
    parser.add_argument("--self-test", action="store_true",
                        help="run offline stream decoder tests")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0

    if not args.port and not args.hex_text:
        parser.error("specify --port or --hex")
    if args.port and args.hex_text:
        parser.error("--port and --hex are mutually exclusive")

    decoder = StatusStreamDecoder()
    if args.hex_text:
        try:
            data = parse_hex_bytes(args.hex_text)
        except ValueError as exc:
            parser.error(str(exc))
        for frame in decoder.feed(data):
            print_status(frame)
        print("summary: valid=%d invalid=%d sequence_gaps=%d buffered=%d" %
              (decoder.valid_count, decoder.invalid_count,
               decoder.sequence_gap_count, len(decoder.buffer)))
        return 0 if decoder.valid_count else 1

    try:
        import serial
    except ImportError:
        parser.error("--port requires pyserial (pip install pyserial)")

    try:
        serial_port = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as exc:
        parser.error("cannot open serial port: %s" % exc)

    started = time.time()
    print("listening %s @ %d; press Ctrl-C to stop" % (args.port, args.baud))
    try:
        while args.seconds <= 0 or time.time() - started < args.seconds:
            chunk = serial_port.read(128)
            if chunk:
                for frame in decoder.feed(chunk):
                    print_status(frame)
    except KeyboardInterrupt:
        pass
    finally:
        serial_port.close()
    print("summary: valid=%d invalid=%d sequence_gaps=%d buffered=%d" %
          (decoder.valid_count, decoder.invalid_count,
           decoder.sequence_gap_count, len(decoder.buffer)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
