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
    RecoilCAN, MODES, MODE_NAMES, fmt_error, open_bus,
    DEFAULT_DEVICE_ID, DEFAULT_BITRATE,
)


def _read(dev, name):
    """Read a param, returning None on a bus timeout (so the prereq gate can report 'unreachable')."""
    try:
        return dev.read(name)
    except Exception as e:  # TimeoutError or backend error
        print(f"  ! failed to read {name}: {e}")
        return None


def check_prerequisites(dev):
    """Hard prerequisite gate. Returns (all_ok: bool, current_host_pos, offset).

    Every check must pass before MODE_POSITION is allowed. Prints a PASS/FAIL checklist.
    """
    print("Checking prerequisites for closed-loop position control:\n")
    checks = []  # (label, ok, detail)

    fw = _read(dev, "firmware_version")
    if fw is None:
        print("  [FAIL] device unreachable on the CAN bus — check id/wiring/termination/bitrate.")
        return False, None, None
    checks.append(("device reachable", True, f"firmware 0x{int(fw):08X}"))

    err = int(_read(dev, "error") or 0xFFFFFFFF)
    checks.append(("no latched error", err == 0, fmt_error(err)))

    mode = int(_read(dev, "mode") or 0xFF)
    checks.append(("mode == IDLE", mode == MODES["idle"],
                   f"0x{mode:02X} ({MODE_NAMES.get(mode, '?')})"))

    flux = _read(dev, "flux_offset")
    checks.append(("flux (electrical) calibrated", flux is not None and abs(flux) > 1e-6,
                   f"flux_offset={flux:+.5f} rad" if flux is not None else "n/a"))

    uv = _read(dev, "undervoltage_threshold")
    bus = _read(dev, "bus_voltage")
    floor = max(6.0, uv if (uv is not None and math.isfinite(uv)) else 6.0)
    checks.append(("motor bus voltage present", bus is not None and bus >= floor,
                   f"{bus:.2f} V (>= {floor:.2f})" if bus is not None else "n/a"))

    pkp = _read(dev, "position_kp")
    checks.append(("position Kp nonzero", pkp is not None and pkp > 0.0,
                   f"position_kp={pkp}" if pkp is not None else "n/a"))

    vlim = _read(dev, "velocity_limit")
    tlim = _read(dev, "torque_limit")
    ilim = _read(dev, "current_limit")
    checks.append(("velocity_limit > 0", vlim is not None and vlim > 0.0, f"{vlim}"))
    checks.append(("torque_limit > 0",   tlim is not None and tlim > 0.0, f"{tlim}"))
    checks.append(("current_limit > 0",  ilim is not None and ilim > 0.0, f"{ilim}"))

    lo = _read(dev, "position_limit_lower")
    hi = _read(dev, "position_limit_upper")
    limits_ok = (lo is not None and hi is not None
                 and math.isfinite(lo) and math.isfinite(hi) and lo < hi)
    checks.append(("finite position limits", limits_ok,
                   f"raw [{lo}, {hi}]" if (lo is not None and hi is not None) else "n/a"))

    for label, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {label:30s} {detail}")

    all_ok = all(ok for _, ok, _ in checks)

    # Frame context for the operator (only meaningful once the basic reads succeeded).
    offset = _read(dev, "position_offset")
    pm_raw = _read(dev, "position_measured")
    host_pos = None
    if offset is not None and pm_raw is not None:
        host_pos = pm_raw - offset
        print(f"\n  current position: {host_pos:+.4f} rad (host frame)   "
              f"[raw {pm_raw:+.4f}, offset {offset:+.4f}]")
        if limits_ok:
            print(f"  host travel range: [{lo - offset:+.4f}, {hi - offset:+.4f}] rad")

    print()
    if all_ok:
        print("All prerequisites PASS.")
    else:
        print("PREREQUISITES NOT MET — position mode is blocked. Fix the FAIL items above and re-run.")
    return all_ok, host_pos, offset


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
    p.add_argument("--device-id", type=int, default=DEFAULT_DEVICE_ID)
    p.add_argument("--max-step", type=float, default=0.05,
                   help="max per-command move in rad (host frame); default 0.05 (~3 deg at arm)")
    p.add_argument("--settle-tol", type=float, default=0.02,
                   help="position settle tolerance in rad; default 0.02")
    args = p.parse_args()

    bus = open_bus(args.interface, args.channel, args.bitrate)
    dev = RecoilCAN(bus, device_id=args.device_id)

    energized = False
    try:
        ok, _, offset = check_prerequisites(dev)
        if not ok:
            sys.exit(1)

        lo = dev.read("position_limit_lower")
        hi = dev.read("position_limit_upper")
        host_lo, host_hi = lo - offset, hi - offset

        print("\n*** Switching to MODE_POSITION will ENERGIZE the motor and may MOVE the arm. ***")
        print(f"    Ensure the arm is clear and on a safe jig. Per-command step limit: {args.max_step:.4f} rad.")
        if input("    Type 'go' to enable position mode (anything else aborts): ").strip() != "go":
            print("Aborted — staying in IDLE.")
            sys.exit(0)

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
            try:
                dev.set_mode("idle")
                print("Returned to MODE_IDLE.")
            except Exception as e:
                print(f"!! FAILED to set IDLE on exit ({e}) — DISABLE THE MOTOR MANUALLY.")


if __name__ == "__main__":
    main()
