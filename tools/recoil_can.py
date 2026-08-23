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
  - HEARTBEAT:       func=0xE,  resets the safety watchdog (no reply)

SAFETY WATCHDOG: in every mode except DISABLED / IDLE / CALIBRATION / VERNIER_CALIBRATION, the
firmware faults to DAMPING + ERROR_WATCHDOG_TIMEOUT if no RPDO1/2/3 or HEARTBEAT frame arrives
within 1 s. SDO reads and writes do NOT feed it. So a one-shot `mode torque` followed by process
exit self-faults about a second later -- use the `run` subcommand (or the Keepalive class) to
hold any driving mode.

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
import threading
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
    "watchdog_timeout":     (0x008, "u32"),
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
    # Mode setpoints + feedback. Each driving mode consumes a DIFFERENT target field; whatever
    # value is already sitting there takes effect the instant you switch modes, so always write
    # the target BEFORE the `mode` command.
    "torque_target":        (0x044, "f32"),   # MODE_TORQUE cmd; feed-forward term in MODE_POSITION
    "torque_measured":      (0x048, "f32"),
    "torque_setpoint":      (0x04C, "f32"),   # post-EMA, post-clamp: what actually drives i_q
    "velocity_target":      (0x050, "f32"),   # MODE_VELOCITY cmd (clamped to velocity_limit)
    "velocity_measured":    (0x054, "f32"),
    "position_target":      (0x05C, "f32"),
    "position_measured":    (0x060, "f32"),   # raw arm position (absolute, NO offset applied)
    "torque_filter_alpha":  (0x070, "f32"),
    "current_limit":        (0x074, "f32"),   # current_controller.i_limit
    # MODE_CURRENT cmd. The position controller still RUNS in current mode (its else branch), but
    # MotorController_update gates the torque->i_q conversion on POSITION||VELOCITY||TORQUE, so its
    # torque_setpoint output is discarded and i_q_target is whatever the host wrote.
    "i_q_target":           (0x0B8, "f32"),   # PRE-clamp command
    "i_d_target":           (0x0BC, "f32"),
    "i_q_measured":         (0x0C0, "f32"),
    # POST-clamp (clampf against i_limit in CurrentController_update). This is the true analogue
    # of torque_setpoint: the value the current loop actually chases. Use it, not i_q_target, when
    # you want to see a clamp take effect.
    "i_q_setpoint":         (0x0C8, "f32"),
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


def as_f32(value):
    """The exact value a float32 field will hold after the firmware stores it.

    Readback checks must compare against this, not against the Python double: 12345.6 round-trips
    to 12345.5996..., an error of ~4e-4 that a fixed 1e-6 tolerance would reject as a write
    failure. Round-tripping makes the comparison exact at any magnitude.
    """
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


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
        # Serializes transmits only. A Keepalive thread sends heartbeats concurrently with the
        # main thread's SDO traffic, and python-can backends are not guaranteed safe against two
        # simultaneous send() calls. Deliberately NOT held across recv(): a heartbeat blocked
        # behind a 0.5 s SDO timeout would eat half the 1 s watchdog budget. recv() needs no
        # guard here because only one thread ever receives, and heartbeats draw no reply.
        self._tx_lock = threading.Lock()

    def _tx(self, msg):
        with self._tx_lock:
            self.bus.send(msg)

    def _send(self, func, data):
        self._tx(can.Message(
            arbitration_id=make_id(func, self.device_id),
            is_extended_id=False,
            data=bytes(data),
        ))

    def heartbeat(self):
        # FUNC_HEARTBEAT resets the firmware's TIM2 watchdog counter and nothing else.
        self._send(FUNC_HEARTBEAT, [self.device_id & 0xFF])

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
            m = self.bus.recv(timeout=max(0.0, deadline - time.time()))
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
        self._tx(can.Message(arbitration_id=make_id(FUNC_RECEIVE_SDO, 0),
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


class Keepalive:
    """Background heartbeat that feeds the firmware's safety watchdog.

    With SAFETY_WATCHDOG_ENABLED, TIM2 (160 MHz / 16000 = 10 kHz, period 10000 = exactly 1 s)
    faults the controller to MODE_DAMPING + ERROR_WATCHDOG_TIMEOUT if no watchdog-feeding frame
    arrives within 1 s, in every mode EXCEPT DISABLED / IDLE / CALIBRATION / VERNIER_CALIBRATION
    (app.c HAL_TIM_PeriodElapsedCallback).

    Only RPDO1/2/3 and FUNC_HEARTBEAT reset that counter (motor_controller.c
    handleCANMessage). SDO reads and writes do NOT -- so polling telemetry or sitting at an
    operator prompt keeps the host busy while the controller silently times out. Any tool that
    holds a driving mode must run one of these.

    Use as a context manager:
        with Keepalive(dev):
            ...                       # driving mode is safe to hold here
    """

    MIN_PERIOD = 0.01          # below this we are just flooding a 1 Mbit bus
    MAX_PERIOD = 0.5           # above this we are gambling against the deadline

    def __init__(self, dev, period=0.2, deadline=None):
        # A period of 0 or a negative makes Event.wait() return immediately, turning this into an
        # unthrottled send loop that saturates the bus while the motor is driving. Reject it.
        if not (self.MIN_PERIOD <= period <= self.MAX_PERIOD):
            raise ValueError(f"heartbeat period {period} outside "
                             f"[{self.MIN_PERIOD}, {self.MAX_PERIOD}] s")
        # The deadline is NOT fixed at 1 s: MotorController_init programs TIM2 from
        # controller->watchdog_timeout (ms), which is restored from flash. A board carrying
        # watchdog_timeout = 100 would fault mid-move at the default 0.2 s period. Callers pass
        # the value they read from the device; keep a 3x margin against poll jitter.
        if deadline is not None and period > deadline / 3.0:
            raise ValueError(f"heartbeat period {period:.3f} s is too slow for this board's "
                             f"{deadline:.3f} s watchdog deadline (need <= {deadline / 3.0:.3f})")
        self.dev = dev
        self.period = period
        self._stop = threading.Event()
        self._thread = None
        self.errors = 0            # transmit failures; a nonzero count means the link is sick

    def __enter__(self):
        # Feed once up front so the budget starts full even if the caller immediately blocks.
        self.dev.heartbeat()
        self._thread = threading.Thread(target=self._run, name="keepalive", daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        return False

    def _run(self):
        # Event.wait() returns True the moment stop is set, so shutdown is immediate rather than
        # sleeping out the last period.
        while not self._stop.wait(self.period):
            try:
                self.dev.heartbeat()
            except Exception:
                # Never let a transient transmit failure kill the thread: dropping the heartbeat
                # entirely would fault the controller mid-move. Count it and keep trying.
                #
                # NB this counts LOCAL failures only. bus.send() returns once the frame is queued
                # in the driver, so a frame that is never ACKed (ESC unplugged, broken wire) is
                # retried by the CAN controller forever without raising here. errors > 0 proves
                # the link is sick; errors == 0 does NOT prove it is healthy. The SDO replies in
                # the caller's telemetry loop are the real liveness evidence.
                self.errors += 1


def safe_read(dev, name):
    """Read a param, returning None on a bus timeout (so gates can report 'unreachable').

    Callers MUST test `is None` rather than falsiness: the healthy values are themselves falsy
    (error 0 = NO_ERROR, mode 0x00 = DISABLED), so `x or default` reports a clean controller as
    faulted.
    """
    try:
        return dev.read(name)
    except Exception as e:  # TimeoutError or backend error
        print(f"  ! failed to read {name}: {e}")
        return None


def check_prerequisites(dev, mode="position", verbose=True):
    """Hard gate that every energizing path must pass. Returns (all_ok, info dict).

    Shared by recoil_jog.py and `recoil_can.py run` on purpose: a second, weaker gate is how an
    uncalibrated or unlimited board ends up energized. `mode` selects the mode-specific checks.
    """
    checks = []   # (label, ok, detail)
    info = {}

    fw = safe_read(dev, "firmware_version")
    if fw is None:
        if verbose:
            print("  [FAIL] device unreachable on the CAN bus — check id/wiring/termination/bitrate.")
        return False, info
    checks.append(("device reachable", True, f"firmware 0x{int(fw):08X}"))

    raw_err = safe_read(dev, "error")
    err = 0xFFFFFFFF if raw_err is None else int(raw_err)
    checks.append(("no latched error", raw_err is not None and err == 0, fmt_error(err)))

    raw_mode = safe_read(dev, "mode")
    cur_mode = 0xFF if raw_mode is None else int(raw_mode)
    checks.append(("mode == IDLE", raw_mode is not None and cur_mode == MODES["idle"],
                   f"0x{cur_mode:02X} ({MODE_NAMES.get(cur_mode, '?')})"
                   + ("  (DISABLED usually means boot vernier resolution failed — see `status`)"
                      if cur_mode == MODES["disabled"] else "")))

    # flux_offset gates EVERY energizing mode. It feeds the commutation angle directly
    # (motor_controller.c theta calculation); at 0 the board commutates at an arbitrary
    # electrical offset. Torque/current modes have no outer loop to notice, so the commanded
    # torque can come out wrong-signed or near-zero with large current draw.
    flux = safe_read(dev, "flux_offset")
    checks.append(("flux (electrical) calibrated", flux is not None and abs(flux) > 1e-6,
                   f"flux_offset={flux:+.5f} rad" if flux is not None else "n/a"))

    uv = safe_read(dev, "undervoltage_threshold")
    bus_v = safe_read(dev, "bus_voltage")
    floor = max(6.0, uv if (uv is not None and math.isfinite(uv)) else 6.0)
    checks.append(("motor bus voltage present", bus_v is not None and bus_v >= floor,
                   f"{bus_v:.2f} V (>= {floor:.2f})" if bus_v is not None else "n/a"))

    vlim = safe_read(dev, "velocity_limit")
    tlim = safe_read(dev, "torque_limit")
    ilim = safe_read(dev, "current_limit")
    for label, v in (("velocity_limit", vlim), ("torque_limit", tlim), ("current_limit", ilim)):
        # Must be finite AND positive: clampf(x, -inf, inf) disables limiting entirely, and a
        # negative limit inverts clampf into a constant full-scale output of the wrong sign.
        checks.append((f"{label} finite and > 0",
                       v is not None and math.isfinite(v) and v > 0.0, f"{v}"))

    if mode == "position":
        pkp = safe_read(dev, "position_kp")
        checks.append(("position Kp nonzero", pkp is not None and pkp > 0.0,
                       f"position_kp={pkp}" if pkp is not None else "n/a"))

    lo = safe_read(dev, "position_limit_lower")
    hi = safe_read(dev, "position_limit_upper")
    limits_ok = (lo is not None and hi is not None
                 and math.isfinite(lo) and math.isfinite(hi) and lo < hi)
    # Finite limits are mandatory for position mode (they clamp the setpoint) and strongly wanted
    # everywhere else: the overtravel guard is a no-op while they are +/-INFINITY, which is the
    # firmware default. Without them, torque/velocity/current modes have NO position protection.
    checks.append(("finite position limits", limits_ok,
                   f"raw [{lo}, {hi}]" if (lo is not None and hi is not None) else "n/a"))

    if verbose:
        for label, ok, detail in checks:
            print(f"  [{'PASS' if ok else 'FAIL'}] {label:30s} {detail}")

    # Not a gate — both energizing callers clear this explicitly before switching mode, and
    # blocking on something the tool then fixes itself just trains operators to ignore failures.
    # Still worth surfacing: a stale torque_target is a live FEED-FORWARD added AFTER the position
    # clamp (position_controller.c), so it pushes the arm off its soft limit in position mode and
    # is the entire command in torque mode. Never cleared by setMode/PositionController_reset.
    tq_t = safe_read(dev, "torque_target")
    if verbose and tq_t is not None and abs(tq_t) > 1e-9:
        print(f"  [WARN] stale torque_target = {tq_t:+.4f} Nm (feed-forward) — will be zeroed "
              f"before energizing")

    offset = safe_read(dev, "position_offset")
    pm_raw = safe_read(dev, "position_measured")
    info.update(offset=offset, position_raw=pm_raw, limit_lo=lo, limit_hi=hi,
                limits_ok=limits_ok, torque_limit=tlim, current_limit=ilim)
    if offset is not None and pm_raw is not None:
        info["position_host"] = pm_raw - offset
        if verbose:
            print(f"\n  current position: {pm_raw - offset:+.4f} rad (host frame)   "
                  f"[raw {pm_raw:+.4f}, offset {offset:+.4f}]")
            if limits_ok:
                print(f"  host travel range: [{lo - offset:+.4f}, {hi - offset:+.4f}] rad")

    return all(ok for _, ok, _ in checks), info


def read_watchdog_deadline(dev):
    """Seconds the firmware allows between watchdog-feeding frames, or None if unreadable.

    TIM2's autoreload is programmed once in MotorController_init from controller->watchdog_timeout
    (ms), which loadConfig restores from flash — so it is NOT reliably 1 s. Note that writing this
    param over SDO has no runtime effect; it needs flash-store + reboot.
    """
    v = safe_read(dev, "watchdog_timeout")
    if v is None or int(v) <= 0:
        return None
    return int(v) / 1000.0


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


# Driving mode -> the parameter that mode consumes as its setpoint. MODE_POSITION is absent
# because position targets go over PDO2 (host frame; firmware adds position_offset), not SDO.
RUN_TARGET_PARAM = {
    "velocity": "velocity_target",
    "torque":   "torque_target",
    "current":  "i_q_target",
}


def _deenergize(dev, mode_name, why=""):
    """Return the controller to a safe state and VERIFY it, never trusting the unacked NMT.

    Ordering matters. For a mode that leaves the arm moving we pass through DAMPING (short-brake:
    MotorController_update drives 0,0,0 PWM) to bleed speed before IDLE, because IDLE fully
    de-energizes the powerstage and a gravity-loaded arm FREEWHEELS. Then the setpoint is zeroed
    so a later mode switch cannot re-apply this run's command.

    Wrapped against BaseException, not Exception: a second Ctrl-C landing inside this function
    would otherwise abandon an energized motor with a traceback on screen.
    """
    try:
        # Every driving mode, not just the obviously-moving ones. Torque and current mode command
        # a constant torque, i.e. constant acceleration — the arm is moving fastest exactly when
        # the run ends, so going straight to IDLE freewheels it at speed.
        if mode_name in ("position", "velocity", "torque", "current"):
            dev.set_mode("damping")
            time.sleep(0.2)
        dev.set_mode("idle")
        # Zero every field that can command or bias torque. torque_target is included even for
        # position mode, where it is a feed-forward term added AFTER the position clamp.
        for pname in ("torque_target", "velocity_target", "i_q_target", "i_d_target"):
            dev.write(pname, 0.0)
        time.sleep(0.05)
        m = safe_read(dev, "mode")
        if m is None:
            print("!! COULD NOT CONFIRM IDLE (no reply) — VERIFY THE MOTOR IS DE-ENERGIZED.")
        elif int(m) != MODES["idle"]:
            print(f"!! CONTROLLER IS NOT IDLE (0x{int(m):02X} {MODE_NAMES.get(int(m), '?')}) "
                  f"— DISABLE THE MOTOR MANUALLY.")
        else:
            print(f"Returned to MODE_IDLE (setpoints zeroed).{why}")
            print("    NOTE: IDLE de-energizes completely — the joint freewheels. Support the arm.")
    except BaseException as e:
        print(f"!! FAILED to de-energize ({e!r}) — DISABLE THE MOTOR MANUALLY.")


def cmd_run(dev, args):
    """Hold a driving mode for a bounded time with the watchdog fed, streaming telemetry.

    This exists because every other path into a driving mode is unsafe from a one-shot CLI:
    `mode torque` returns immediately, the process exits, no heartbeat follows, and ~1 s later
    the firmware faults to DAMPING with ERROR_WATCHDOG_TIMEOUT. Anything that holds a mode has
    to stay resident and keep feeding TIM2.
    """
    mode_name = args.mode

    if not (math.isfinite(args.duration) and 0 < args.duration <= 300):
        print(f"REFUSING: --duration {args.duration} must be finite and in (0, 300] s.")
        return 1
    if not (0.02 <= args.period <= 5.0):
        print(f"REFUSING: --period {args.period} must be in [0.02, 5.0] s.")
        return 1

    # ---- shared prerequisite gate (same one recoil_jog uses; `run` must not be the weak door) ----
    print(f"Checking prerequisites for {mode_name} mode:\n")
    ok, info = check_prerequisites(dev, mode=mode_name)
    if not ok:
        print("\nPREREQUISITES NOT MET — refusing to energize. Fix the FAIL items above.")
        return 1
    print("\nAll prerequisites PASS.")
    offset = info["offset"]

    # ---- watchdog deadline: NOT assumed to be 1 s (loadConfig restores it from flash) ----
    deadline = read_watchdog_deadline(dev)
    try:
        keepalive = Keepalive(dev, period=args.heartbeat_period, deadline=deadline)
    except ValueError as e:
        print(f"REFUSING: {e}")
        return 1

    # ---- optional limit overrides, applied and verified BEFORE energizing ----
    for pname, val in (("torque_limit", args.torque_limit), ("current_limit", args.current_limit)):
        if val is None:
            continue
        # clampf(x, -inf, inf) disables limiting outright; a NEGATIVE limit inverts clampf into a
        # constant full-scale output of the wrong sign (clampf(x, +1, -1) returns -1 for all
        # x > -1). Both are worse than no override at all.
        if not (math.isfinite(val) and val > 0.0):
            print(f"REFUSING: --{pname.replace('_', '-')} {val} must be finite and > 0.")
            return 1
        dev.write(pname, val)
        got = dev.read(pname)            # SDO writes are unacked; never trust one without a read
        if got != as_f32(val):
            print(f"REFUSING: {pname} readback {got} != requested {val}")
            return 1
        print(f"  {pname} = {got}")

    # ---- clear any stale torque feed-forward BEFORE energizing ----
    # Never cleared by setMode or PositionController_reset, and added after the position clamp,
    # so a leftover value pushes the arm off its soft limit the instant the mode changes.
    dev.write("torque_target", 0.0)
    if dev.read("torque_target") != 0.0:
        print("REFUSING: could not clear stale torque_target.")
        return 1

    # ---- stage the setpoint BEFORE the mode switch ----
    # position_target is likewise never reset, and position_setpoint is recomputed from it on
    # every 2 kHz tick — so entering MODE_POSITION with a stale target makes the arm lurch toward
    # it before we could possibly send the real one. PDO2 is accepted in IDLE (no mode gate in
    # handleCANMessage) and feeds the watchdog, so we can stage it safely while de-energized.
    if mode_name == "position":
        lo, hi = info["limit_lo"], info["limit_hi"]
        host_lo, host_hi = lo - offset, hi - offset
        target = args.target if args.target is not None else info["position_host"]
        if not (math.isfinite(target) and host_lo <= target <= host_hi):
            print(f"REFUSING: target {target:+.4f} outside host range "
                  f"[{host_lo:+.4f}, {host_hi:+.4f}] rad.")
            return 1
        dev.set_position(target, 0.0)
        got_raw = dev.read("position_target")
        if got_raw != as_f32(target + offset):
            print(f"REFUSING: position_target readback {got_raw} != "
                  f"expected raw {target + offset}")
            return 1
        print(f"  position target {target:+.4f} rad (host) = {got_raw:+.4f} raw; "
              f"range [{host_lo:+.4f}, {host_hi:+.4f}]")
    else:
        if args.target is None:
            print(f"REFUSING: {mode_name} mode needs --target.")
            return 1
        target = args.target
        if not math.isfinite(target):
            print(f"REFUSING: --target {target} must be finite.")
            return 1
        pname = RUN_TARGET_PARAM[mode_name]
        dev.write(pname, target)
        got = dev.read(pname)
        if got != as_f32(target):
            print(f"REFUSING: {pname} readback {got} != requested {target}")
            return 1
        print(f"  {pname} = {got}")

    # ---- operator confirmation ----
    print(f"\n*** MODE_{mode_name.upper()} ENERGIZES THE MOTOR AND MAY MOVE THE ARM. ***")
    if mode_name != "position":
        # position is the ONLY mode whose setpoint is clamped to position_limit_*; the velocity,
        # torque and current branches of PositionController_update never consult them. Velocity
        # is the most exposed of the three -- it commands continuous motion by definition.
        print("    NOTE: this mode does NOT enforce position_limit_*. Only MODE_POSITION does.")
        print(f"    Fallback is the overtravel guard, which trips ~15 deg PAST the limit "
              f"(raw [{info['limit_lo']}, {info['limit_hi']}]) and only into DAMPING.")
    if deadline is not None and abs(deadline - 1.0) > 1e-6:
        print(f"    Board watchdog deadline is {deadline:.3f} s (not the 1 s default).")
    print(f"    Duration {args.duration:.1f} s, then automatic return to IDLE. Ctrl-C = e-stop.")
    if not args.yes and input("    Type 'go' to proceed (anything else aborts): ").strip() != "go":
        print("Aborted — staying in IDLE.")
        return 0

    energized = False
    rc = 0
    try:
        with keepalive as ka:
            dev.set_mode(mode_name)
            energized = True
            time.sleep(0.1)
            m = int(dev.read("mode"))
            if m != MODES[mode_name]:
                print(f"  ! mode did not switch (now 0x{m:02X} "
                      f"{MODE_NAMES.get(m, '?')}); aborting.")
                return 1

            # torque_setpoint is only in the command path for POSITION/VELOCITY/TORQUE. Every
            # other mode -- MODE_CURRENT, and DAMPING, which is where a fault lands us -- takes
            # PositionController_update's else branch, so the firmware still computes
            # torque_setpoint and then DISCARDS it (MotorController_update gates the torque->i_q
            # conversion on POSITION||VELOCITY||TORQUE). Showing it there is a live-looking number
            # with no bearing on what drives the motor, so fall back to i_q_setpoint.
            #
            # i_q_setpoint, not i_q_target: the former is POST-clamp against i_limit, matching
            # torque_setpoint's post-clamp semantics, so "the column pins at the limit" means the
            # same thing in both branches. i_q_target would just echo what we wrote.
            TORQUE_PATH_MODES = (MODES["position"], MODES["velocity"], MODES["torque"])
            if MODES[mode_name] not in TORQUE_PATH_MODES:
                print("  (i_q_set is the post-clamp q-axis current setpoint; torque_setpoint is "
                      "not in the command path in this mode)")

            t0 = time.time()
            while (time.time() - t0) < args.duration:
                e = int(dev.read("error"))
                m = int(dev.read("mode"))
                # Keyed off the mode we just READ, not the one we asked for: once the controller
                # faults to DAMPING, torque_setpoint stops driving anything, and that last line
                # before the e-stop is exactly where a stale reading misleads most.
                drive = (f"tq_set={dev.read('torque_setpoint'):+.4f}"
                         if m in TORQUE_PATH_MODES
                         else f"i_q_set={dev.read('i_q_setpoint'):+.4f}")
                print(f"  t={time.time() - t0:5.1f}s "
                      f"mode=0x{m:02X} "
                      f"pos={dev.read('position_measured') - offset:+.4f} "
                      f"vel={dev.read('velocity_measured'):+.4f} "
                      f"{drive} "
                      f"i_q={dev.read('i_q_measured'):+.4f} "
                      f"err={fmt_error(e)}")
                if e:
                    print("  ! controller raised an error — e-stopping.")
                    rc = 1
                    break
                if m != MODES[mode_name]:
                    print(f"  ! controller left {mode_name} mode on its own — e-stopping.")
                    rc = 1
                    break
                time.sleep(args.period)
    except KeyboardInterrupt:
        print("\n^C — e-stop.")
        rc = 1
    finally:
        if energized:
            _deenergize(dev, mode_name)
        # Reported after de-energizing, and outside the `with`, so it is seen on every exit path
        # (early return, error abort, Ctrl-C) rather than only on a clean run.
        if keepalive.errors:
            print(f"  ! {keepalive.errors} heartbeat transmit failure(s) during the run — "
                  f"the CAN link is unreliable; treat the result as suspect.")
    return rc


def device_id_arg(s):
    """Valid device ids are 1..63 (motor_controller_conf.h: CAN ID range).

    Unchecked, make_id() masks to 0x7F, so --device-id 128 silently becomes 0 = BROADCAST, and
    the firmware accepts broadcast frames on EVERY node (`if (device_id && device_id != ...)`).
    An NMT or `run` issued that way would command every controller on the bus at once.
    """
    v = int(s, 0)
    if not (1 <= v <= 63):
        raise argparse.ArgumentTypeError(f"device id {v} out of range 1..63")
    return v


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
    p.add_argument("--device-id", type=device_id_arg, default=DEFAULT_DEVICE_ID)
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

    prun = sub.add_parser("run", help="ENERGIZE: hold a driving mode with the watchdog fed")
    prun.add_argument("mode", choices=["position", "velocity", "torque", "current"])
    prun.add_argument("--target", type=float, default=None,
                      help="setpoint: rad (host frame) / rad_s / Nm / A depending on mode. "
                           "Required except for position, which defaults to holding here.")
    prun.add_argument("--duration", type=float, default=3.0, help="seconds to hold (default 3)")
    prun.add_argument("--period", type=float, default=0.25, help="telemetry poll period")
    prun.add_argument("--heartbeat-period", type=float, default=0.2,
                      help="watchdog feed period; must stay well under the firmware's 1 s")
    prun.add_argument("--torque-limit", type=float, default=None,
                      help="set position_controller.torque_limit before energizing (not flashed)")
    prun.add_argument("--current-limit", type=float, default=None,
                      help="set current_controller.i_limit before energizing (not flashed)")
    prun.add_argument("--yes", action="store_true", help="skip the confirmation prompt")

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
        elif args.cmd == "run":
            return cmd_run(dev, args)
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
