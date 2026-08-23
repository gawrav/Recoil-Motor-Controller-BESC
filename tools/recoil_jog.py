#!/usr/bin/env python3
"""
recoil_jog.py — interactive, safety-gated position-jog tool for the Recoil B-G431B-ESC1 firmware.

  *** THIS IS THE ONLY TOOL THAT ENERGIZES THE MOTOR AND MOVES THE PHYSICAL ARM. ***

It (1) hard-checks every prerequisite for closed-loop position control, (2) refuses to switch into
MODE_POSITION unless ALL of them pass, then (3) drops into a REPL where you enter either an ABSOLUTE
target (host/zeroed frame, radians) or a DELTA (`+0.05` / `-0.05`). Every command is bounded by a
configurable per-step limit and clamped to the soft position limits. Ctrl-C / `q` is an e-stop that
returns the controller to MODE_IDLE.

It shares the CAN protocol layer with recoil_can.py (single source of truth) — no duplication.

Frames (this is the subtle part; see position_controller.h):
  - Commands (PDO2 / setpos) and this tool's targets are in the HOST / zeroed frame.
    Firmware converts: position_target_raw = host_target + position_offset.
  - The soft limits (position_limit_lower/upper) are in the RAW frame, so the allowed host range is
    [limit_lower - offset, limit_upper - offset]. This tool does that conversion for you.

Usage:
  python3 tools/recoil_jog.py --device-id 14 [--max-step 0.05] [--settle-tol 0.02]
  (CAN adapter options are the same as recoil_can.py: --interface/--channel/--bitrate.)
"""

import argparse
import math
import sys
import time

from recoil_can import (
    RecoilCAN, Keepalive, MODES, MODE_NAMES, fmt_error, open_bus, as_f32,
    check_prerequisites as _shared_prereqs, read_watchdog_deadline, _deenergize,
    device_id_arg,
    DEFAULT_DEVICE_ID, DEFAULT_BITRATE,
)


def check_prerequisites(dev):
    """Hard prerequisite gate. Returns (all_ok: bool, current_host_pos, offset).

    Delegates to recoil_can.check_prerequisites so this tool and `recoil_can.py run` cannot
    diverge — a second, weaker gate is exactly how an uncalibrated or unlimited board ends up
    energized.
    """
    print("Checking prerequisites for closed-loop position control:\n")
    ok, info = _shared_prereqs(dev, mode="position")
    print()
    if ok:
        print("All prerequisites PASS.")
    else:
        print("PREREQUISITES NOT MET — position mode is blocked. Fix the FAIL items above and re-run.")
    return ok, info.get("position_host"), info.get("offset")


def host_position(dev, offset):
    """Current measured position in the host/zeroed frame."""
    pm_raw = dev.read("position_measured")
    return pm_raw - offset


def jog_repl(dev, offset, host_lo, host_hi, max_step, settle_tol):
    print("\n=== JOG (position mode) ===")
    print("  enter an ABSOLUTE host-frame target (e.g. 0.25), or a DELTA (+0.05 / -0.05)")
    print(f"  per-command step limit: {max_step:.4f} rad   host range: [{host_lo:+.4f}, {host_hi:+.4f}]")
    print("  commands:  s = status   q / Ctrl-C = e-stop to IDLE and exit\n")

    while True:
        cur = host_position(dev, offset)
        raw = input(f"jog [{cur:+.4f}] > ").strip()
        if raw == "":
            continue
        if raw in ("q", "quit", "stop", "exit"):
            return
        if raw in ("s", "?", "status"):
            err = int(dev.read("error"))
            print(f"  pos={cur:+.4f} rad (host)   mode=0x{int(dev.read('mode')):02X}   err={fmt_error(err)}")
            continue

        # Parse target: leading +/- => delta from current; bare number => absolute.
        try:
            if raw[0] in "+-":
                target = cur + float(raw)
                kind = f"delta {float(raw):+.4f}"
            else:
                target = float(raw)
                kind = "absolute"
        except ValueError:
            print("  ? enter a number (absolute), +N / -N (delta), 's', or 'q'")
            continue

        step = target - cur
        if abs(step) > max_step + 1e-9:
            print(f"  REJECTED: step {step:+.4f} rad exceeds --max-step {max_step:.4f}. "
                  f"Use a smaller move or raise --max-step (deliberately).")
            continue
        if target < host_lo - 1e-9 or target > host_hi + 1e-9:
            print(f"  REJECTED: target {target:+.4f} outside host range [{host_lo:+.4f}, {host_hi:+.4f}].")
            continue

        # Command it (host frame; firmware adds offset). vel ff = 0 (unused in MODE_POSITION).
        print(f"  -> {kind}: commanding {target:+.4f} rad (host)")
        dev.set_position(target, 0.0)

        # Watch it settle.
        t0 = time.time()
        while time.time() - t0 < 2.0:
            time.sleep(0.05)
            now = host_position(dev, offset)
            if abs(now - target) <= settle_tol:
                print(f"     settled at {now:+.4f} rad (err {now - target:+.4f})")
                break
        else:
            now = host_position(dev, offset)
            print(f"     NOT settled after 2 s: at {now:+.4f} (err {now - target:+.4f}) — "
                  f"check gains/limits/load.")

        if int(dev.read("error")) != 0:
            err = int(dev.read("error"))
            print(f"  ! controller raised an error: {fmt_error(err)} — e-stopping.")
            return


def main():
    p = argparse.ArgumentParser(description="Recoil ESC interactive position-jog (energizes the motor)")
    p.add_argument("--interface", default="gs_usb")
    p.add_argument("--channel", default="0")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE)
    p.add_argument("--device-id", type=device_id_arg, default=DEFAULT_DEVICE_ID)
    p.add_argument("--max-step", type=float, default=0.05,
                   help="max per-command move in rad (host frame); default 0.05 (~3 deg at arm)")
    p.add_argument("--settle-tol", type=float, default=0.02,
                   help="position settle tolerance in rad; default 0.02")
    p.add_argument("--heartbeat-period", type=float, default=0.2,
                   help="watchdog feed period in s; must stay well under the firmware's 1 s")
    args = p.parse_args()

    bus = open_bus(args.interface, args.channel, args.bitrate)
    dev = RecoilCAN(bus, device_id=args.device_id)

    energized = False
    keepalive = None
    try:
        ok, _, offset = check_prerequisites(dev)
        if not ok:
            sys.exit(1)

        lo = dev.read("position_limit_lower")
        hi = dev.read("position_limit_upper")
        host_lo, host_hi = lo - offset, hi - offset

        # The board's deadline is NOT assumed to be 1 s — loadConfig restores watchdog_timeout
        # from flash, and TIM2's autoreload is programmed from it.
        deadline = read_watchdog_deadline(dev)
        try:
            keepalive = Keepalive(dev, period=args.heartbeat_period, deadline=deadline)
        except ValueError as e:
            print(f"REFUSING: {e}")
            sys.exit(1)

        # Clear the stale torque feed-forward before energizing: torque_target is added AFTER the
        # position clamp (position_controller.c), so a leftover value from an earlier torque test
        # pushes the arm straight off its soft limit. Never cleared by setMode.
        dev.write("torque_target", 0.0)
        if dev.read("torque_target") != 0.0:
            print("REFUSING: could not clear stale torque_target.")
            sys.exit(1)

        # Stage the position target BEFORE energizing. position_target is never reset by setMode
        # or PositionController_reset, and position_setpoint is recomputed from it every 2 kHz
        # tick — so entering MODE_POSITION with a stale target (power-on 0.0 raw, or whatever a
        # previous `setpos` left) makes the arm lurch toward it before the operator types
        # anything. PDO2 is accepted in IDLE (no mode gate) so staging here is safe.
        hold = host_position(dev, offset)
        dev.set_position(hold, 0.0)
        if dev.read("position_target") != as_f32(hold + offset):
            print("REFUSING: could not stage the hold-here position target.")
            sys.exit(1)
        print(f"\n  staged hold-here target {hold:+.4f} rad (host) while still de-energized.")

        print("\n*** Switching to MODE_POSITION will ENERGIZE the motor and may MOVE the arm. ***")
        print(f"    Ensure the arm is clear and on a safe jig. Per-command step limit: {args.max_step:.4f} rad.")
        if deadline is not None and abs(deadline - 1.0) > 1e-6:
            print(f"    Board watchdog deadline is {deadline:.3f} s (not the 1 s default).")
        if input("    Type 'go' to enable position mode (anything else aborts): ").strip() != "go":
            print("Aborted — staying in IDLE.")
            sys.exit(0)

        # The REPL blocks on input() indefinitely, and SDO polling does not feed the firmware's
        # watchdog — without this the controller faults to DAMPING while the operator is still
        # reading the prompt. Must wrap the whole energized region, not just the moves.
        with keepalive:
            dev.set_mode("position")
            energized = True
            time.sleep(0.1)
            m = int(dev.read("mode"))
            if m != MODES["position"]:
                print(f"  ! mode did not switch (now 0x{m:02X}); aborting.")
                return
            print("  position mode active.")

            jog_repl(dev, offset, host_lo, host_hi, args.max_step, args.settle_tol)

    except KeyboardInterrupt:
        print("\n^C — e-stop.")
    finally:
        if energized:
            # Shared with cmd_run: DAMPING to bleed motion, then IDLE, then zero every torque
            # command, then READ BACK to confirm — set_mode is an unacked NMT, so printing
            # "returned to IDLE" without verifying is a lie exactly when it matters most.
            _deenergize(dev, "position")
        if keepalive is not None and keepalive.errors:
            print(f"  ! {keepalive.errors} heartbeat transmit failure(s) during the session — "
                  f"the CAN link is unreliable.")


if __name__ == "__main__":
    main()
