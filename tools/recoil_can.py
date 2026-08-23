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

ADAPTER:
  CANable / CANtact running candleLight firmware  -> gs_usb (raw USB, NO /dev/tty).
    macOS:  brew install libusb && pip install python-can gs_usb pyusb
    run:    --interface gs_usb --channel 0      (channel = scan index; 0 = first device)
  CANable / CANtact running slcan firmware        -> shows up as /dev/tty.usbmodem*.
    run:    --interface slcan --channel /dev/tty.usbmodem1234   (default)
  pcan:               --interface pcan       --channel PCAN_USBBUS1
  socketcan (Linux):  --interface socketcan  --channel can0

If `ioreg`/System Information shows "canable gs_usb" and there is NO /dev/tty.usb*
node, your dongle has candleLight firmware -> use --interface gs_usb.

Requires:  pip install python-can   (+ gs_usb pyusb and libusb for the gs_usb backend)
"""

import argparse
import math
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed.  Run:  pip install python-can")


DEFAULT_DEVICE_ID = 14   # this board's CAN id (override with --device-id)
DEFAULT_BITRATE = 1_000_000

# ---- Function codes (FrameFunction enum) ----
FUNC_NMT          = 0x0
FUNC_TRANSMIT_PDO_2 = 0x5
FUNC_RECEIVE_PDO_2  = 0x6
FUNC_TRANSMIT_SDO = 0xB
FUNC_RECEIVE_SDO  = 0xC
FUNC_FLASH        = 0xD
FUNC_HEARTBEAT    = 0xE
FUNC_SYSTEM       = 0xF

SYSTEM_CMD_RECOVER_I2C = 1

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
    0x10000: "OVERTRAVEL",
}

# ---- Parameter registry: name -> (byte_offset, type) ----
# type is one of: "f32", "u32", "i32".  uint8 fields are read via their aligned word.
PARAMS = {
    "device_id":            (0x000, "u32"),
    "firmware_version":     (0x004, "u32"),
    "mode":                 (0x010, "u32"),
    "error":                (0x014, "u32"),
    "gear_ratio":           (0x01C, "f32"),
    # PID gains + limits (used by recoil_jog.py's prerequisite gate):
    "position_kp":          (0x020, "f32"),
    "position_ki":          (0x024, "f32"),
    "velocity_kp":          (0x028, "f32"),
    "velocity_ki":          (0x02C, "f32"),
    "torque_limit":         (0x030, "f32"),
    "velocity_limit":       (0x034, "f32"),
    "position_limit_lower": (0x038, "f32"),   # RAW arm frame (NOT the host/zeroed frame)
    "position_limit_upper": (0x03C, "f32"),   # RAW arm frame
    "position_offset":      (0x040, "f32"),   # arm-frame zero (position_controller.position_offset)
    "position_target":      (0x05C, "f32"),
    "position_measured":    (0x060, "f32"),   # raw arm position (absolute, NO offset applied)
    "current_limit":        (0x074, "f32"),   # current_controller.i_limit
    "flux_offset":          (0x13C, "f32"),   # encoder.flux_offset (0 => not flux-calibrated)
    "undervoltage_threshold": (0x0F4, "f32"),
    "bus_voltage":          (0x100, "f32"),   # powerstage.bus_voltage_measured
    "encoder_n_rotations":  (0x130, "i32"),
    "encoder_position":     (0x134, "f32"),
    "encoder2_position":    (0x360, "f32"),
    "vernier_phase_offset": (0x56C, "f32"),
    # vernier_base_sector (u8 @0x570) and vernier_sector (u8 @0x571) share one word:
    "vernier_status_word":  (0x570, "u32"),
    "vernier_cal_magic":    (0x578, "u32"),
    # ---- Diagnostics block (RAM-only, read-only). Packed words decoded in cmd_diag. ----
    # diag_probe_word: [0]=enc probe,[1]=enc2 probe,[2]=enc STATUS,[3]=enc2 STATUS
    "diag_probe_word":       (0x57C, "u32"),
    # diag_agc_word:   [0]=enc AGC,[1]=enc2 AGC,[2]=fail_stage,[3]=pad
    "diag_agc_word":         (0x580, "u32"),
    "diag_ok_count":         (0x584, "u32"),
    "diag_frame_err_count":  (0x588, "u32"),
    "diag_start_fail_count": (0x58C, "u32"),
    "diag_i2c_err_count":    (0x590, "u32"),
    "diag_last_i2c_errcode": (0x594, "u32"),
    "diag_theta_p":          (0x598, "f32"),
    "diag_theta_s":          (0x59C, "f32"),
    "diag_psi":              (0x5A0, "f32"),
    "diag_psi_error":        (0x5A4, "f32"),
    "diag_q_raw":            (0x5A8, "i32"),
    "diag_consec_frame_err":     (0x5AC, "u32"),
    "diag_max_consec_frame_err": (0x5B0, "u32"),
    # [15:0]=primary bad reads, [31:16]=secondary bad reads, of ENC_BOOT_INTEGRITY_SAMPLES:
    "diag_boot_read_errors":     (0x5B4, "u32"),
}

# Must match ENCODER_BOOT_INTEGRITY_SAMPLES in motor_controller_conf.h.
ENC_BOOT_INTEGRITY_SAMPLES = 64

# AS5600/AS5600L STATUS reg 0x0B: MD=detected (good), ML=too weak, MH=too strong.
STATUS_BITS = {0x20: "MD", 0x10: "ML", 0x08: "MH"}
# STM32 HAL I2C ErrorCode bits.
I2C_ERR_BITS = {0x01: "BERR", 0x02: "ARLO", 0x04: "AF", 0x08: "OVR",
                0x10: "DMA", 0x20: "TIMEOUT", 0x40: "SIZE"}
FAIL_STAGE_NAMES = {0: "OK", 1: "UNCALIBRATED", 2: "DRAIN",
                    3: "PRIMARY_READ", 4: "SECONDARY_READ", 5: "PSI_MISMATCH"}


def fmt_probe(b):
    if b == 0xFF:
        return "not-read"
    return "on-bus" if b == 0 else f"FAIL(0x{b:02X})"


def fmt_status_reg(b):
    if b == 0xFF:
        return "not-read"
    flags = [name for mask, name in STATUS_BITS.items() if b & mask]
    healthy = (b & 0x20) and not (b & 0x10) and not (b & 0x08)  # MD set, ML/MH clear
    return f"0x{b:02X} [{'|'.join(flags) or 'none'}] {'ok' if healthy else 'BAD'}"


def fmt_agc(b):
    if b == 0xFF:
        return "not-read"
    # AGC range is 0-255 at 5V (this board powers the encoders at 5V); it would be 0-128 at 3.3V,
    # in which case halve these thresholds. Mid-range ideal; rails => airgap out of range.
    if b <= 32:
        hint = "LOW: magnet too close/strong"
    elif b >= 224:
        hint = "HIGH: magnet too far/weak"
    else:
        hint = "centered"
    return f"{b} ({hint})"


def fmt_i2c_errcode(v):
    if v == 0:
        return "NONE"
    bits = [name for mask, name in I2C_ERR_BITS.items() if v & mask]
    return f"0x{v:02X} (" + "|".join(bits) + ")"

VERNIER_CAL_MAGIC = 0x5645524E   # "VERN"; == means genuinely calibrated


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

    def recover_i2c(self):
        # Request I2C bus recovery; firmware performs it in the foreground (~1 cycle later).
        self._send(FUNC_SYSTEM, [SYSTEM_CMD_RECOVER_I2C])

    def set_zero(self):
        # Make the CURRENT arm position read zero in the control/telemetry frame, by setting
        # position_offset = raw position_measured. Does NOT move the wrap (that's the
        # vernier base_sector) — only shifts where "zero" lands. Persisted to flash.
        # Returns (raw_before, offset_written).
        pm = self.read("position_measured")     # 0x060: raw absolute arm angle (no offset)
        self.write("position_offset", pm)        # getter returns position_measured - offset -> 0 here
        self.flash_store()
        return pm

    def flash_store(self):
        self._send(FUNC_FLASH, [1])

    def flash_load(self):
        self._send(FUNC_FLASH, [2])

    def set_position(self, pos, vel=0.0):
        # PDO2: [position_target f32, velocity_target f32]
        self._send(FUNC_RECEIVE_PDO_2, struct.pack("<ff", pos, vel))

    def discover(self, timeout=1.0):
        """Find the device ID by sending a BROADCAST SDO read (device field = 0 in
        the arbitration ID). The firmware processes broadcasts regardless of its ID
        and replies on its real ID, so this reveals the ID and proves the bus works.
        Returns {device_id: device_id_value}.  Empty -> physical-layer problem
        (termination / bitrate / wiring), not an ID mismatch.
        """
        while self.bus.recv(timeout=0.0) is not None:
            pass
        req = bytes([0x40, 0x00, 0x00, 0, 0, 0, 0, 0])  # read PARAM_DEVICE_ID (offset 0)
        self.bus.send(can.Message(arbitration_id=make_id(FUNC_RECEIVE_SDO, 0),
                                  is_extended_id=False, data=req))
        found = {}
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = self.bus.recv(timeout=max(0.0, deadline - time.time()))
            if m is None:
                continue
            if (m.arbitration_id >> 7) == FUNC_TRANSMIT_SDO and len(m.data) >= 4:
                found[m.arbitration_id & 0x7F] = struct.unpack("<I", bytes(m.data[:4]))[0]
        return found


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
    magic = int(dev.read("vernier_cal_magic"))
    calibrated = "YES" if magic == VERNIER_CAL_MAGIC else f"NO (magic=0x{magic:08X})"
    print(f"  firmware:    0x{int(dev.read('firmware_version')):08X}")
    print(f"  mode:        0x{mode:02X} ({MODE_NAMES.get(mode, '?')})")
    print(f"  error:       {fmt_error(err)}")
    print(f"  calibrated:  {calibrated}")
    print(f"  gear_ratio:  {dev.read('gear_ratio'):.4f}")
    print(f"  phase_offset:{dev.read('vernier_phase_offset'):+.5f} rad")
    print(f"  base_sector: {base_sector}")
    print(f"  sector:      {sector}")
    print(f"  n_rotations: {int(dev.read('encoder_n_rotations'))}")
    print(f"  enc pos:     {dev.read('encoder_position'):+.5f} rad (motor)")
    print(f"  enc2 pos:    {dev.read('encoder2_position'):+.5f} rad")
    pm = dev.read("position_measured")
    off = dev.read("position_offset")
    print(f"  arm pos raw: {pm:+.5f} rad (absolute, no offset)")
    print(f"  position_offset: {off:+.5f} rad")
    print(f"  arm pos:     {pm - off:+.5f} rad (zeroed = host/PDO frame)")
    print("  --- encoder diagnostics ---")
    print_diag(dev, indent="  ")


def print_diag(dev, indent=""):
    probe = int(dev.read("diag_probe_word"))
    agcw = int(dev.read("diag_agc_word"))
    enc_probe   = probe & 0xFF
    enc2_probe  = (probe >> 8) & 0xFF
    enc_status  = (probe >> 16) & 0xFF
    enc2_status = (probe >> 24) & 0xFF
    enc_agc     = agcw & 0xFF
    enc2_agc    = (agcw >> 8) & 0xFF
    fail_stage  = (agcw >> 16) & 0xFF
    print(f"{indent}primary  : {fmt_probe(enc_probe)}  status={fmt_status_reg(enc_status)}  agc={fmt_agc(enc_agc)}")
    print(f"{indent}secondary: {fmt_probe(enc2_probe)}  status={fmt_status_reg(enc2_status)}  agc={fmt_agc(enc2_agc)}")
    bre = int(dev.read("diag_boot_read_errors"))
    n = ENC_BOOT_INTEGRITY_SAMPLES
    print(f"{indent}boot read errors: primary={bre & 0xFFFF}/{n}  secondary={(bre >> 16) & 0xFFFF}/{n}")
    print(f"{indent}resolve  : fail_stage={FAIL_STAGE_NAMES.get(fail_stage, fail_stage)}  "
          f"q_raw={int(dev.read('diag_q_raw'))}  psi_err={math.degrees(dev.read('diag_psi_error')):+.2f} deg")
    print(f"{indent}           theta_p={dev.read('diag_theta_p'):+.4f}  theta_s={dev.read('diag_theta_s'):+.4f}  "
          f"psi={dev.read('diag_psi'):+.4f} rad")
    # Counters are absolute (wrap ~5 days @10kHz); poll twice for a rate. frame_err is now a TRUE
    # distinct-glitch count (the read is re-armed each time). max_consec = worst burst: 1 means
    # every glitch was isolated and self-recovered; approaching the fault threshold means real bursts.
    print(f"{indent}primary live: ok={int(dev.read('diag_ok_count'))}  "
          f"frame_err={int(dev.read('diag_frame_err_count'))}  "
          f"consec={int(dev.read('diag_consec_frame_err'))}  "
          f"max_consec={int(dev.read('diag_max_consec_frame_err'))}  "
          f"start_fail={int(dev.read('diag_start_fail_count'))}  "
          f"i2c_err={int(dev.read('diag_i2c_err_count'))}  "
          f"last_i2c={fmt_i2c_errcode(int(dev.read('diag_last_i2c_errcode')))}")


def cmd_diag(dev):
    print("Encoder diagnostics:")
    print_diag(dev, indent="  ")


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


def open_bus(interface, channel, bitrate):
    """Open a python-can bus, handling the gs_usb backend's index-based addressing.

    gs_usb (candleLight) has no /dev node — the device is selected by scan index,
    and the bitrate is programmed onto the dongle here.
    """
    if interface == "gs_usb":
        idx = int(channel) if str(channel).isdigit() else 0
        # Pass both channel and index: depending on python-can version one or the
        # other selects the scanned device; the unused one is ignored.
        return can.Bus(interface="gs_usb", channel=idx, index=idx, bitrate=bitrate)
    return can.Bus(interface=interface, channel=channel, bitrate=bitrate)


def main():
    p = argparse.ArgumentParser(description="Recoil ESC CAN control")
    # Default to gs_usb (candleLight CANable). For an slcan dongle use:
    #   --interface slcan --channel /dev/tty.usbmodemXXXX
    p.add_argument("--interface", default="gs_usb")
    p.add_argument("--channel", default="0",
                   help="gs_usb: scan index (0=first); slcan: /dev/tty.usbmodemXXXX")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    p.add_argument("--device-id", type=int, default=DEFAULT_DEVICE_ID)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="dump key params once")
    sub.add_parser("diag", help="dump encoder bus/magnet/geometry diagnostics")
    sub.add_parser("discover", help="find device id(s) on the bus via broadcast SDO")

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
    sub.add_parser("recover", help="request I2C bus recovery, then poll status")
    sub.add_parser("set-zero", help="make the current arm position read zero (sets position_offset, flash)")

    ppos = sub.add_parser("setpos", help="send a position target (PDO2)")
    ppos.add_argument("position", type=float)
    ppos.add_argument("--vel", type=float, default=0.0)

    args = p.parse_args()

    bus = open_bus(args.interface, args.channel, args.bitrate)
    dev = RecoilCAN(bus, device_id=args.device_id)
    try:
        if args.cmd == "status":
            cmd_status(dev)
        elif args.cmd == "diag":
            cmd_diag(dev)
        elif args.cmd == "discover":
            found = dev.discover()
            if not found:
                print("no devices replied — check termination / bitrate / wiring "
                      "(this is NOT a device-id issue; broadcast ignores the id)")
            else:
                for did, val in sorted(found.items()):
                    print(f"  device id {did}  (PARAM_DEVICE_ID readback = {val})")
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
        elif args.cmd == "recover":
            dev.recover_i2c()
            print("I2C recovery requested; polling status...")
            time.sleep(0.3)   # let the foreground perform it (~20 Hz service loop)
            cmd_status(dev)
        elif args.cmd == "set-zero":
            pm = dev.set_zero()
            print(f"zeroed at raw arm pos {pm:+.5f} rad; current position now reads ~0 "
                  f"(persisted). re-run `status` to confirm.")
            print("NOTE: position_limit_lower/upper and the overtravel guard are evaluated in the "
                  "ABSOLUTE (raw) frame — they do NOT shift with this zero.")
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
