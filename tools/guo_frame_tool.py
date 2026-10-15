"""Build and optionally send a Guo AA55 velocity frame.

Examples (PowerShell):
  python tools/guo_frame_tool.py --vx 300 --wz 0
  python tools/guo_frame_tool.py --port COM7 --vx 300 --wz 150 --repeat 10

The tool is intentionally dependency-free for frame generation.  Serial
transmission uses pyserial only when --port is supplied.
"""

from __future__ import print_function

import argparse
import struct
import sys
import time


GUO_STATUS_FRAME_LENGTH = 34
GUO_STATUS_PAYLOAD_LENGTH = 22


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in bytearray(data):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_velocity_frame(vx_mmps, vy_mmps, wz_mradps, sequence=0):
    payload = struct.pack("<hhh", int(vx_mmps), int(vy_mmps), int(wz_mradps))
    body = struct.pack("<BBBBH", 1, 1, 0, int(sequence) & 0xFF, len(payload)) + payload
    crc = crc16_ccitt_false(body)
    return b"\xAA\x55" + body + struct.pack("<H", crc) + b"\x66\xBB"


def parse_status_frame(frame):
    """校验并解码迁移固件的34字节状态帧；非法帧返回None。"""
    if len(frame) != GUO_STATUS_FRAME_LENGTH:
        return None
    if frame[0:2] != b"\xAA\x55" or frame[-2:] != b"\x66\xBB":
        return None
    if frame[2] != 1 or frame[3] != 0x81:
        return None
    payload_len = frame[6] | (frame[7] << 8)
    if payload_len != GUO_STATUS_PAYLOAD_LENGTH:
        return None
    body_end = 8 + payload_len
    expected_crc = frame[body_end] | (frame[body_end + 1] << 8)
    if crc16_ccitt_false(frame[2:body_end]) != expected_crc:
        return None

    values = struct.unpack("<Hhhhhhhhhhh", bytes(frame[8:body_end]))
    return {
        "sequence": frame[5],
        "battery_mv": values[0],
        "temperature_centi": values[1],
        "vx_mmps": values[2],
        "vy_mmps": values[3],
        "vz_mradps": values[4],
        "gx_centi_dps": values[5],
        "gy_centi_dps": values[6],
        "gz_centi_dps": values[7],
        "ax_centi_g": values[8],
        "ay_centi_g": values[9],
        "az_centi_g": values[10],
    }


def parse_hex_bytes(text):
    """把XCOM复制的空格/换行分隔十六进制文本转成字节串。"""
    tokens = text.replace(",", " ").split()
    try:
        return bytes(bytearray(int(token, 16) for token in tokens))
    except ValueError:
        raise ValueError("hex text contains a non-hex byte")


def run_self_test():
    """离线检查协议长度、端序、CRC和状态解码，避免依赖串口硬件。"""
    frame = make_velocity_frame(-300, 12, 150, sequence=0xA5)
    assert len(frame) == 18
    assert frame[:2] == b"\xAA\x55" and frame[-2:] == b"\x66\xBB"
    assert crc16_ccitt_false(frame[2:14]) == (frame[14] | (frame[15] << 8))
    assert struct.unpack("<hhh", frame[8:14]) == (-300, 12, 150)

    payload = struct.pack("<Hhhhhhhhhhh", 12345, -10, 300, 0, 0,
                          1, -2, 3, 0, 0, 100)
    body = struct.pack("<BBBBH", 1, 0x81, 0, 7, len(payload)) + payload
    status = b"\xAA\x55" + body
    status += struct.pack("<H", crc16_ccitt_false(body)) + b"\x66\xBB"
    decoded = parse_status_frame(status)
    assert decoded is not None and decoded["sequence"] == 7
    assert decoded["battery_mv"] == 12345 and decoded["vx_mmps"] == 300
    broken = bytearray(status)
    broken[30] ^= 1
    assert parse_status_frame(bytes(broken)) is None
    print("self-test: PASS (control=18 bytes, status=34 bytes, CRC/LE/signed fields)")


def main():
    parser = argparse.ArgumentParser(description="Guo AA55 velocity-frame tester")
    parser.add_argument("--vx", type=int, default=0, help="forward speed, mm/s")
    parser.add_argument("--vy", type=int, default=0, help="lateral speed, mm/s")
    parser.add_argument("--wz", type=int, default=0, help="yaw rate, mrad/s")
    parser.add_argument("--seq", type=int, default=0, help="initial sequence number")
    parser.add_argument("--port", help="optional serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--bad-crc", action="store_true", help="flip CRC for negative testing")
    parser.add_argument("--decode-status", metavar="HEX",
                        help="decode one XCOM-copied 34-byte AA55 status frame")
    parser.add_argument("--self-test", action="store_true",
                        help="run dependency-free offline protocol tests")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0

    if args.decode_status:
        try:
            decoded = parse_status_frame(parse_hex_bytes(args.decode_status))
        except ValueError as exc:
            parser.error(str(exc))
        if decoded is None:
            parser.error("not a valid 34-byte AA55 status frame")
        for key in sorted(decoded):
            print("%s=%s" % (key, decoded[key]))
        return 0

    if not -32768 <= args.vx <= 32767 or not -32768 <= args.vy <= 32767 or not -32768 <= args.wz <= 32767:
        parser.error("vx/vy/wz must fit signed int16")
    if args.repeat < 1:
        parser.error("repeat must be >= 1")

    serial_port = None
    if args.port:
        try:
            import serial
            serial_port = serial.Serial(args.port, args.baud, timeout=1)
        except ImportError:
            parser.error("--port requires pyserial (pip install pyserial)")
        except Exception as exc:
            parser.error("cannot open serial port: %s" % exc)

    try:
        for index in range(args.repeat):
            frame = make_velocity_frame(args.vx, args.vy, args.wz, args.seq + index)
            if args.bad_crc:
                frame = frame[:-4] + bytes(bytearray([frame[-4] ^ 1, frame[-3], frame[-2], frame[-1]]))
            print(" ".join("%02X" % byte for byte in bytearray(frame)))
            if serial_port:
                serial_port.write(frame)
                serial_port.flush()
            if index + 1 < args.repeat:
                time.sleep(max(0.0, args.interval))
    finally:
        if serial_port:
            serial_port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
