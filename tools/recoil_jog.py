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
    safe_read,
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
    """Current measured position in the host/zeroed frame, or None if the read dropped.

    Tolerant by design: every caller is on a path where one lost SDO reply must not turn a
    controlled e-stop into a traceback.
    """
    pm_raw = safe_read(dev, "position_measured")
    return None if pm_raw is None else pm_raw - offset


def _diagnose_stall(dev, tq_peak, iq_peak, tq_samples=1, iq_samples=1):
    """Say WHICH clamp is binding on a failed settle, or that neither is.

    Getting this right matters because the two prescriptions are opposite. torque_setpoint is
    computed entirely inside PositionController_update and knows NOTHING about i_limit: when
    current_limit is the real ceiling, the arm stalls, position error keeps growing, position_kp *
    error keeps rising, and torque_setpoint pins at torque_limit anyway. So "tq_set == torque_limit"
    alone does NOT mean the torque clamp is the constraint -- and telling the operator to raise
    torque_limit there does nothing except arm a larger command that will lurch the arm the moment
    the current limit is lifted.

    i_q_setpoint is the disambiguator: it is post-clamp against i_limit, so it pins at
    current_limit exactly when the current clamp is the binding one. clampf returns the bound
    bit-for-bit and both sides are the same float32 round-tripped over SDO, so compare exactly --
    no tolerance, which also avoids calling an idle motor 'saturated' when torque_limit is tiny.
    """
    if tq_samples == 0 or iq_samples == 0:
        # No usable samples means no evidence. Saying "neither clamp is binding" here would be a
        # positive claim built from nothing, and it is the opposite of what a pinned board needs.
        empty = " and ".join(n for n, c in (("torque_setpoint", tq_samples),
                                            ("i_q_setpoint", iq_samples)) if c == 0)
        return (f"no {empty} samples were readable during the move — cannot say which limit is "
                f"binding. Check the CAN link.")
    tlim = safe_read(dev, "torque_limit")
    ilim = safe_read(dev, "current_limit")
    if tlim is None or ilim is None:
        return "could not read the limits to diagnose the stall."
    detail = f"peak tq_set={tq_peak:.4f}/{tlim:.4f} Nm, peak i_q_set={iq_peak:.4f}/{ilim:.4f} A"
    # Current first: if both clamps are pinned, current is the effective ceiling.
    if iq_peak >= ilim:
        return (f"CURRENT_LIMIT is binding ({detail}) — raising torque_limit will NOT help. "
                f"Raise current_limit, or check Kt / gear_ratio.")
    if tq_peak >= tlim:
        return (f"TORQUE_LIMIT is binding ({detail}) — raise torque_limit, not the gains.")
    return (f"neither clamp is binding ({detail}) — look at the gains, the load, or the encoder.")


def jog_repl(dev, offset, host_lo, host_hi, max_step, settle_tol):
    print("\n=== JOG (position mode) ===")
    print("  enter an ABSOLUTE host-frame target (e.g. 0.25), or a DELTA (+0.05 / -0.05)")
    print(f"  per-command step limit: {max_step:.4f} rad   host range: [{host_lo:+.4f}, {host_hi:+.4f}]")
    print("  commands:  s = status   q / Ctrl-C = e-stop to IDLE and exit\n")

    lost = 0        # consecutive commands that provably did not reach the board
    while True:
        cur = host_position(dev, offset)
        raw = input(f"jog [{'??' if cur is None else f'{cur:+.4f}'}] > ").strip()
        if raw == "":
            continue
        # Case-insensitive: 'Q' falling through to the number parser on an e-stop path is not a
        # trade anyone should have to remember under pressure.
        if raw.lower() in ("q", "quit", "stop", "exit"):
            return
        if raw.lower() in ("s", "?", "status"):
            # Re-read the position: `cur` was captured BEFORE the blocking input() and may be
            # arbitrarily old (the operator can sit at this prompt for minutes while the arm sags
            # or is backdriven). Printing it beside four freshly-read live fields would read as a
            # coherent snapshot when it is not.
            now = host_position(dev, offset)
            # torque_setpoint IS in the command path here (MODE_POSITION feeds it to i_q via
            # Kt/gear_ratio), so it is meaningful -- unlike in current/damping modes. Every read
            # is tolerant: one dropped reply should print "??", not abort an energized session.
            vals = {n: safe_read(dev, n) for n in
                    ("mode", "torque_setpoint", "torque_limit", "i_q_measured", "error")}
            s_pos = "??" if now is None else f"{now:+.4f}"
            s_mode = "??" if vals["mode"] is None else f"0x{int(vals['mode']):02X}"
            s_tq = "??" if vals["torque_setpoint"] is None else f"{vals['torque_setpoint']:+.4f}"
            s_tlim = "??" if vals["torque_limit"] is None else f"{vals['torque_limit']:.4f}"
            s_iq = "??" if vals["i_q_measured"] is None else f"{vals['i_q_measured']:+.4f}"
            s_err = "??" if vals["error"] is None else fmt_error(int(vals["error"]))
            print(f"  pos={s_pos} rad (host)   mode={s_mode}   "
                  f"tq_set={s_tq}/{s_tlim} Nm   i_q={s_iq} A   err={s_err}")
            continue

        # Refuse to move without a position. Both guards below are computed from `cur`, and NaN
        # defeats them silently: abs(nan) > max_step is False, and so are both range comparisons,
        # so a NaN target would sail through and reach the firmware, where clampf(nan, lo, hi)
        # returns nan and the torque command becomes nan.
        if cur is None:
            print("  REJECTED: position is unreadable, so the step limit and the range check "
                  "cannot be evaluated. Fix the link, or 'q' to e-stop.")
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

        # NaN defeats every guard below: abs(nan) > x, nan < lo and nan > hi are all False, so a
        # NaN target would pass the step limit AND the range check and reach the board. There
        # clampf(nan, lo, hi) returns nan, position_integrator becomes nan, and because the
        # integrator and the torque EMA both feed on themselves it stays nan through every later
        # valid command -- only a mode change clears it. inf is caught by the step guard; NaN is
        # the only hole. (The landing check then reports "did not reach the board", which is the
        # exact opposite of what happened.)
        if not math.isfinite(target):
            print(f"  REJECTED: target must be finite ({raw!r} parsed as {target}). A non-finite "
                  f"target bypasses the step and range checks and poisons the firmware's "
                  f"position integrator until the next mode change.")
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

        # Verify the target actually landed. PDO2 is fire-and-forget, and a dropped frame is
        # indistinguishable from a joint that will not move: the firmware holds its previous
        # target, computes zero error, commands zero torque, and the arm sits still. That is
        # exactly the failure a lost-frame bug produced on this rig, and it cost a debugging
        # session to tell apart from a mechanical problem. One read makes it self-evident.
        #
        # The expectation must mirror the MCU's arithmetic EXACTLY, in single precision at every
        # step: set_position packs the target as f32 on the wire, and setPositionTarget then does
        # a float+float add on a single-precision FPU. Computing as_f32(target + offset) from
        # Python doubles instead disagrees by 1 ULP on ~29% of realistic jogs -- which would tell
        # the operator a MOVING arm is dead and skip all monitoring. Round the target to f32
        # first; `offset` is already exact, having been decoded from an f32 field.
        want_raw = as_f32(as_f32(target) + offset)
        got_raw = safe_read(dev, "position_target")
        if got_raw is None:
            # Could not verify != did not land. Losing the SDO reply on a flaky link is at least
            # as likely as losing the PDO2, and asserting stillness about a live joint is the
            # worse error. Keep monitoring.
            print("  ? could not verify the target landed (no SDO reply) — watching anyway.")
        elif got_raw != want_raw:
            lost += 1
            # Read the error word: the firmware sets ERROR_CAN_RX_FAULT on a malformed or
            # out-of-range frame, so "the command was lost" may have a reason recorded on the
            # board. This branch skips the settle loop, so without this the end-of-command error
            # check never runs and that reason is never shown.
            e = safe_read(dev, "error")
            why = ("  error unreadable" if e is None
                   else f"  controller error: {fmt_error(int(e))}" if int(e) else "")
            print(f"  ! TARGET DID NOT LAND: position_target reads {got_raw:.9g}, expected "
                  f"{want_raw:.9g} raw. The command did not reach the board (or another "
                  f"session moved the target) — the arm will not move. Not a mechanical "
                  f"fault.{why}")
            if lost >= 3:
                print("  ! three consecutive commands lost on an energized joint — e-stopping.")
                return
            continue
        else:
            lost = 0

        # Watch it settle, sampling BOTH clamps as we go. A single reading taken after the loop
        # misses saturation that occurred during the move and has since relaxed, which is the
        # common case for a move that overshoots its torque budget then creeps in.
        tq_peak = 0.0
        iq_peak = 0.0
        tq_samples = 0
        iq_samples = 0
        ticks = 0
        t0 = time.time()
        while time.time() - t0 < 2.0:
            time.sleep(0.05)
            # Watch the mode, not just the position. A watchdog timeout or an overtravel trip
            # drops the controller to DAMPING mid-move; without this the REPL keeps reporting
            # position against a de-energized joint and only notices at the end-of-command error
            # check, one whole command later.
            # Every other iteration: the mode read is a 4th round trip, and sampling it at the
            # full rate costs ~25% of the tq_peak/iq_peak density that _diagnose_stall depends on.
            # 100 ms is still far faster than the old end-of-command check.
            ticks += 1
            if ticks % 2 == 0:
                m = safe_read(dev, "mode")
                if m is not None and int(m) != MODES["position"]:
                    # Watchdog and overtravel BOTH land in DAMPING, so the mode alone cannot say
                    # which. The error word can, and it is the single most useful datum at the
                    # moment an arm stops mid-move.
                    e = safe_read(dev, "error")
                    why = fmt_error(int(e)) if e is not None else "error unreadable"
                    print(f"  ! controller left position mode mid-move "
                          f"(now 0x{int(m):02X} {MODE_NAMES.get(int(m), '?')}): {why} "
                          f"— e-stopping.")
                    return
            # Sample the clamps FIRST. These are independent reads that succeed even when the
            # position read drops, and skipping them leaves both peaks at 0.0 -- which makes
            # _diagnose_stall report "neither clamp is binding" for a board that is in fact
            # pinned at torque_limit. That is the exact opposite prescription, on the tool whose
            # whole purpose is getting that call right.
            tq_s = safe_read(dev, "torque_setpoint")
            iq_s = safe_read(dev, "i_q_setpoint")
            if tq_s is not None:
                tq_peak = max(tq_peak, abs(tq_s))
                tq_samples += 1
            if iq_s is not None:
                iq_peak = max(iq_peak, abs(iq_s))
                iq_samples += 1
            pm = safe_read(dev, "position_measured")
            if pm is None:
                continue                      # dropped reply; try again next tick
            now = pm - offset
            if abs(now - target) <= settle_tol:
                print(f"     settled at {now:+.4f} rad (err {now - target:+.4f})")
                break
        else:
            now = host_position(dev, offset)
            where = (f"at {now:+.4f} (err {now - target:+.4f})" if now is not None
                     else "position unreadable")
            print(f"     NOT settled after 2 s: {where}")
            print(f"     {_diagnose_stall(dev, tq_peak, iq_peak, tq_samples, iq_samples)}")

        err = safe_read(dev, "error")
        if err is not None and int(err) != 0:
            print(f"  ! controller raised an error: {fmt_error(int(err))} — e-stopping.")
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
    # cmd_run validates its analogous floats; these were unvalidated. --max-step nan disables the
    # per-command step limit outright, because abs(step) > nan is False -- a full-range move in
    # one keystroke on an energized arm.
    for name, v, lo, hi in (("--max-step", args.max_step, 1e-4, 1.0),
                            ("--settle-tol", args.settle_tol, 1e-4, 1.0)):
        if not (math.isfinite(v) and lo <= v <= hi):
            sys.exit(f"{name} must be finite and within [{lo}, {hi}] (got {v})")

    bus = open_bus(args.interface, args.channel, args.bitrate)
    dev = RecoilCAN(bus, device_id=args.device_id)

    energized = False
    keepalive = None
    try:
        ok, _, offset = check_prerequisites(dev)
        if not ok:
            sys.exit(1)
        if offset is None or not math.isfinite(offset):
            print(f"REFUSING: position_offset is "
                  f"{'unreadable' if offset is None else f'not finite ({offset})'} — every "
                  f"host/raw frame conversion below depends on it, and a NaN offset silently "
                  f"defeats both the step limit and the range check.")
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
        if hold is None or not math.isfinite(hold):
            # Finiteness matters as much as readability: pm - offset is NaN if either field is,
            # and set_position would put that on the wire BEFORE the readback caught it, leaving
            # position_target poisoned. PositionController_reset does not clear position_target,
            # so no mode change recovers it -- the next tool to enter MODE_POSITION without
            # staging a target re-poisons the integrator on the first tick.
            why = "unreadable" if hold is None else f"not finite ({hold})"
            print(f"REFUSING: current position is {why}; cannot stage a hold-here target.")
            sys.exit(1)
        dev.set_position(hold, 0.0)
        # f32-exact (see the jog_repl comment). The stage path subtracts the offset and adds
        # it back, which is the WORST case for double-rounding: ~1 startup in 3 would refuse.
        if dev.read("position_target") != as_f32(as_f32(hold) + offset):
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
        # This tool previously never closed the bus at all, leaving the gs_usb device started.
        # settle() first so no queued transmit is still in flight when the device goes away --
        # tearing the bus down mid-URB leaves the dongle receiving nothing until it is replugged.
        # try/FINALLY so a second Ctrl-C during settle (a BaseException, up to ~0.4 s window)
        # cannot skip the shutdown -- that window is reachable exactly during an e-stop.
        try:
            dev.settle()
        finally:
            bus.shutdown()


if __name__ == "__main__":
    main()
