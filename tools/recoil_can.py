#!/usr/bin/env python3
"""
recoil_can.py — host-side CAN control for the Recoil B-G431B-ESC1 firmware.

Protocol (from the firmware, see motor_controller.c / motor_controller_conf.h):
  - Classic CAN, 11-bit IDs, 1 Mbit/s.
  - CAN ID = (function << 7) | device_id.   device_id default = 1.
  - NMT (set mode):  func=0x0,  data = [mode_u8, addressed_device_u8]
  - SDO read:        func=0xC,  data[0]=0x40, data[1:3]=param_offset_u16(LE)
                     -> reply on func=0xB with 4 data bytes = value (LE)
  - SDO write:       func=0xC,  data[0]=0x20, data[1:3]=param_offset_u16(LE),
                     data[4:8]=value_u32(LE)   (no reply — fire and forget)
  - FLASH:           func=0xD,  data=[1]=store, [2]=load
  - PDO2 (pos/vel):  func=0x6,  data = [pos_target_f32, vel_target_f32]
                     -> reply func=0x5 = [pos_measured_f32, vel_measured_f32]

"Param offset" is the byte offset of the field inside the MotorController struct;
the firmware reads/writes 32 bits at that offset directly.

ADAPTER: defaults to an slcan device (CANable/CANtact-style) which is the most
Mac-friendly. Override with --interface / --channel / --bitrate. Examples:
  slcan:   --interface slcan   --channel /dev/tty.usbmodem1234   (default)
  pcan:    --interface pcan     --channel PCAN_USBBUS1
  socketcan (Linux): --interface socketcan --channel can0

Requires:  pip install python-can
"""

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed.  Run:  pip install python-can")


DEFAULT_DEVICE_ID = 1
DEFAULT_BITRATE = 1_000_000

# ---- Function codes (FrameFunction enum) ----
FUNC_NMT          = 0x0
FUNC_TRANSMIT_PDO_2 = 0x5
FUNC_RECEIVE_PDO_2  = 0x6
FUNC_TRANSMIT_SDO = 0xB
FUNC_RECEIVE_SDO  = 0xC
FUNC_FLASH        = 0xD
FUNC_HEARTBEAT    = 0xE

# ---- Modes (Mode enum) ----
MODES = {
    "disabled":          0x00,
    "idle":              0x01,
    "damping":           0x02,
    "calibration":       0x05,   # electrical flux calibration
    "vernier_calibration": 0x06, # static vernier calibration (no motor drive)
    "current":           0x10,
    "torque":            0x11,
    "velocity":          0x12,
    "position":          0x13,
}
MODE_NAMES = {v: k for k, v in MODES.items()}

# ---- Error bits (ErrorCode enum) ----
ERROR_BITS = {
    0x0001: "GENERAL",
    0x0002: "ESTOP",
    0x0004: "INITIALIZATION_ERROR",
    0x0008: "CALIBRATION_ERROR",
    0x0010: "POWERSTAGE_ERROR",
    0x0020: "INVALID_MODE",
    0x0040: "WATCHDOG_TIMEOUT",
    0x0080: "OVER_VOLTAGE",
    0x0100: "OVER_CURRENT",
    0x0200: "OVER_TEMPERATURE",
    0x0400: "CAN_RX_FAULT",
    0x0800: "CAN_TX_FAULT",
    0x1000: "I2C_FAULT",
    0x2000: "ENCODER_FAULT",
    0x4000: "VERNIER_INCONSISTENT",
    0x8000: "VERNIER_CALIBRATION_FAILED",
}

# ---- Parameter registry: name -> (byte_offset, type) ----
# type is one of: "f32", "u32", "i32".  uint8 fields are read via their aligned word.
PARAMS = {
    "device_id":            (0x000, "u32"),
    "firmware_version":     (0x004, "u32"),
    "mode":                 (0x010, "u32"),
    "error":                (0x014, "u32"),
    "gear_ratio":           (0x01C, "f32"),
    "position_target":      (0x05C, "f32"),
    "position_measured":    (0x060, "f32"),
    "encoder_n_rotations":  (0x130, "i32"),
    "encoder_position":     (0x134, "f32"),
    "encoder2_position":    (0x360, "f32"),
    "vernier_phase_offset": (0x56C, "f32"),
    # vernier_base_sector (u8 @0x570) and vernier_sector (u8 @0x571) share one word:
    "vernier_status_word":  (0x570, "u32"),
}


def make_id(func, device_id):
    return (func << 7) | (device_id & 0x7F)


def decode(value_bytes, kind):
    if kind == "f32":
        return struct.unpack("<f", value_bytes)[0]
    if kind == "i32":
        return struct.unpack("<i", value_bytes)[0]
    return struct.unpack("<I", value_bytes)[0]


def encode(value, kind):
    if kind == "f32":
        return struct.pack("<f", float(value))
    if kind == "i32":
        return struct.pack("<i", int(value))
    return struct.pack("<I", int(value) & 0xFFFFFFFF)


class RecoilCAN:
    def __init__(self, bus, device_id=DEFAULT_DEVICE_ID):
        self.bus = bus
        self.device_id = device_id

    def _send(self, func, data):
        msg = can.Message(
            arbitration_id=make_id(func, self.device_id),
            is_extended_id=False,
            data=bytes(data),
        )
        self.bus.send(msg)

    def set_mode(self, mode):
        if isinstance(mode, str):
            mode = MODES[mode]
        # NMT: data = [requested_mode, addressed_device_id]
        self._send(FUNC_NMT, [mode & 0xFF, self.device_id & 0xFF])

    def sdo_write(self, offset, value, kind):
        payload = bytes([0x20, offset & 0xFF, (offset >> 8) & 0xFF, 0x00]) + encode(value, kind)
        self._send(FUNC_RECEIVE_SDO, payload)

    def sdo_read_raw(self, offset, timeout=0.5):
        # flush any stale frames
        while self.bus.recv(timeout=0.0) is not None:
            pass
        req = bytes([0x40, offset & 0xFF, (offset >> 8) & 0xFF, 0, 0, 0, 0, 0])
        self._send(FUNC_RECEIVE_SDO, req)
        resp_id = make_id(FUNC_TRANSMIT_SDO, self.device_id)
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.bus.recv(timeout=deadline - time.time())
            if m is not None and m.arbitration_id == resp_id and len(m.data) >= 4:
                return bytes(m.data[:4])
        raise TimeoutError(f"no SDO reply for offset 0x{offset:03X}")

    def read(self, name):
        offset, kind = PARAMS[name]
        return decode(self.sdo_read_raw(offset), kind)

    def write(self, name, value):
        offset, kind = PARAMS[name]
        self.sdo_write(offset, value, kind)

    def flash_store(self):
        self._send(FUNC_FLASH, [1])

    def flash_load(self):
        self._send(FUNC_FLASH, [2])

    def set_position(self, pos, vel=0.0):
        # PDO2: [position_target f32, velocity_target f32]
        self._send(FUNC_RECEIVE_PDO_2, struct.pack("<ff", pos, vel))


def fmt_error(err):
    if err == 0:
        return "0x0000 (NO_ERROR)"
    bits = [name for mask, name in ERROR_BITS.items() if err & mask]
    return f"0x{err:04X} (" + " | ".join(bits) + ")"


def cmd_status(dev):
    mode = dev.read("mode")
    err = dev.read("error")
    word = int(dev.read("vernier_status_word"))
    base_sector = word & 0xFF
    sector = (word >> 8) & 0xFF
    print(f"  firmware:    0x{int(dev.read('firmware_version')):08X}")
    print(f"  mode:        0x{mode:02X} ({MODE_NAMES.get(mode, '?')})")
    print(f"  error:       {fmt_error(err)}")
    print(f"  gear_ratio:  {dev.read('gear_ratio'):.4f}")
    print(f"  phase_offset:{dev.read('vernier_phase_offset'):+.5f} rad")
    print(f"  base_sector: {base_sector}")
    print(f"  sector:      {sector}")
    print(f"  n_rotations: {int(dev.read('encoder_n_rotations'))}")
    print(f"  enc pos:     {dev.read('encoder_position'):+.5f} rad (motor)")
    print(f"  enc2 pos:    {dev.read('encoder2_position'):+.5f} rad")
    print(f"  arm pos:     {dev.read('position_measured'):+.5f} rad")


def cmd_monitor(dev, period):
    print("Live monitor (Ctrl-C to stop). Hand-rotate the shaft and watch enc/enc2.")
    try:
        while True:
            word = int(dev.read("vernier_status_word"))
            print(
                f"mode=0x{int(dev.read('mode')):02X} "
                f"err={fmt_error(int(dev.read('error')))} "
                f"sect={(word >> 8) & 0xFF} "
                f"n_rot={int(dev.read('encoder_n_rotations'))} "
                f"enc={dev.read('encoder_position'):+.4f} "
                f"enc2={dev.read('encoder2_position'):+.4f} "
                f"arm={dev.read('position_measured'):+.4f}",
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped.")


def main():
    p = argparse.ArgumentParser(description="Recoil ESC CAN control")
    p.add_argument("--interface", default="slcan")
    p.add_argument("--channel", default="/dev/tty.usbmodem1101")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    p.add_argument("--device-id", type=int, default=DEFAULT_DEVICE_ID)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="dump key params once")

    pm = sub.add_parser("monitor", help="poll key params continuously")
    pm.add_argument("--period", type=float, default=0.2)

    pmode = sub.add_parser("mode", help="set mode via NMT")
    pmode.add_argument("mode", choices=list(MODES.keys()))

    pr = sub.add_parser("read", help="read one param by name")
    pr.add_argument("name", choices=list(PARAMS.keys()))

    pw = sub.add_parser("write", help="write one param by name")
    pw.add_argument("name", choices=list(PARAMS.keys()))
    pw.add_argument("value")

    sub.add_parser("flash-store", help="persist config to flash")
    sub.add_parser("flash-load", help="reload config from flash")

    ppos = sub.add_parser("setpos", help="send a position target (PDO2)")
    ppos.add_argument("position", type=float)
    ppos.add_argument("--vel", type=float, default=0.0)

    args = p.parse_args()

    bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    dev = RecoilCAN(bus, device_id=args.device_id)
    try:
        if args.cmd == "status":
            cmd_status(dev)
        elif args.cmd == "monitor":
            cmd_monitor(dev, args.period)
        elif args.cmd == "mode":
            dev.set_mode(args.mode)
            print(f"sent NMT -> {args.mode} (0x{MODES[args.mode]:02X})")
        elif args.cmd == "read":
            print(f"{args.name} = {dev.read(args.name)}")
        elif args.cmd == "write":
            _, kind = PARAMS[args.name]
            val = float(args.value) if kind == "f32" else int(args.value, 0)
            dev.write(args.name, val)
            print(f"wrote {args.name} = {val} (no ack; read back to confirm)")
        elif args.cmd == "flash-store":
            dev.flash_store()
            print("FLASH store sent")
        elif args.cmd == "flash-load":
            dev.flash_load()
            print("FLASH load sent")
        elif args.cmd == "setpos":
            dev.set_position(args.position, args.vel)
            print(f"position_target = {args.position} (vel ff {args.vel})")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
