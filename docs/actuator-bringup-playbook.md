# Actuator bring-up and test playbook

Operating procedure for bringing up a Recoil B-G431B-ESC1 joint from a bare board to closed-loop
motion, using the host tools in `tools/`. Covers vernier calibration, verification, running each
control mode, and torque-limit testing.

**Keep this updated when the firmware changes.** Every step below is tied to specific firmware
behaviour; the "Firmware ground truth" column in each section says what it depends on.

---

## Conventions and hard facts

Read this section once. Several of these have bitten us already.

### Host frame vs raw frame

Two different position frames are in play and confusing them will drive the arm somewhere you
did not intend.

| Frame | What lives there | Notes |
|---|---|---|
| **Raw / absolute** | `position_measured` (0x060), `position_limit_lower/upper` (0x038/0x03C), the overtravel guard | What the SDO `read` command returns |
| **Host / zeroed** | PDO2 targets (`setpos`), `recoil_jog` targets and display | `PositionController_getPositionMeasured()` returns `position_measured - position_offset`; `setPositionTarget()` adds it back |

Conversion: `host = raw - position_offset`, so the allowed host range is
`[limit_lower - offset, limit_upper - offset]`.

**`set-zero` does not move the limits.** It sets `position_offset = position_measured`, which
changes only where zero *reads*. `position_limit_*` and the overtravel guard are evaluated in the
raw frame (`motor_controller.c`, overtravel guard in `MotorController_update`), so they stay
exactly where they were.

### The safety watchdog will bite you

With `SAFETY_WATCHDOG_ENABLED`, TIM2 faults the controller to `MODE_DAMPING` +
`ERROR_WATCHDOG_TIMEOUT` in every mode **except** DISABLED / IDLE / CALIBRATION /
VERNIER_CALIBRATION.

The deadline is **not hardcoded to 1 s**. `MotorController_init` programs TIM2's autoreload from
`controller->watchdog_timeout` (ms, default 1000), and `loadConfig` restores that from flash — so
a board can carry a much shorter deadline. The tools read it and size the heartbeat accordingly;
`run` prints a line when the board's deadline is not the 1 s default.

> Writing `watchdog_timeout` over SDO has **no runtime effect** — the autoreload is programmed
> once at init. It needs `flash-store` + reboot.

Only **RPDO1/2/3 and FUNC_HEARTBEAT** reset that counter. **SDO reads and writes do not.**

Consequences:

- A one-shot `recoil_can.py mode torque` returns immediately, the process exits, no heartbeat
  follows, and ~1 s later the controller self-faults. **Never hold a driving mode this way.**
- A shell `for` loop that invokes `recoil_can.py` repeatedly has no persistent heartbeat either —
  same failure.
- Use the **`run` subcommand** (or `recoil_jog.py`), both of which run a `Keepalive` thread for
  the entire energized region.

### SDO writes are not acknowledged

`MotorController_handleSDO` sends no reply for a write (`ccs == 1`). **Always read back after
writing anything that matters.** The `run` subcommand does this automatically for the setpoint
and limit overrides.

### Stale setpoints apply instantly on mode switch

Each driving mode consumes a different field, and whatever value is already sitting there takes
effect the moment the mode changes:

| Mode | Consumes | Clamped by |
|---|---|---|
| `position` (0x13) | `position_target` via PDO2 | `position_limit_*`, then `torque_limit`, then `current_limit` |
| `velocity` (0x12) | `velocity_target` (0x050) | `velocity_limit`, then `torque_limit`, then `current_limit` |
| `torque` (0x11) | `torque_target` (0x044) | `torque_limit`, then `current_limit` |
| `current` (0x10) | `i_q_target` (0x0B8), `i_d_target` (0x0BC) | `current_limit` **only** |

This is not a soft warning. `position_target` and `torque_target` are **never** reset by
`setMode` or `PositionController_reset`, and `position_setpoint` is recomputed from
`position_target` on every 2 kHz tick — so entering `MODE_POSITION` with a stale target makes the
arm lurch toward it immediately. `run` and `recoil_jog` both stage the setpoint while still
de-energized (PDO2 is accepted in IDLE) and zero every torque command on exit.

`torque_target` deserves its own mention: in `MODE_POSITION` it is a **feed-forward term added
after the position clamp**, so a leftover value from an earlier torque test pushes the arm off its
soft limit and the position loop cannot reject it. Both tools clear it before energizing and warn
if it was non-zero.

### Only position mode enforces the position limits

`PositionController_update` clamps against `position_limit_*` **only** in `MODE_POSITION`. The
velocity, torque and current branches never consult them. Velocity mode is the most exposed —
it commands continuous motion by definition.

The fallback is the overtravel guard, and it has two conditions that are easy to miss: it requires
`vernier_initialized`, and it is a **no-op while the limits are ±INFINITY** (the firmware
default). It also trips ~15° *past* the limit, into DAMPING. Both tools now refuse to energize
without finite, ordered limits — see Step 6.

### `torque` vs `current` mode

Both end in the same 10 kHz FOC current loop; they differ only in how `i_q_target` gets
populated, and what protections apply on the way.

```
MODE_TORQUE:   torque_target → EMA (torque_filter_alpha) → clamp ±torque_limit
               = torque_setpoint → ÷Kt ÷gear_ratio → i_q_target,  i_d_target := 0

MODE_CURRENT:  i_q_target / i_d_target written directly by the host
```

Both then hit `i_q_setpoint = clampf(i_q_target, ±i_limit)` in `CurrentController_update`.

| | `torque` | `current` |
|---|---|---|
| Units | N·m at the **arm** (post-gearbox) | A, q-axis at the **motor** |
| `torque_limit` | applies | **does not apply** |
| `current_limit` | applies | applies — the only limit |
| `torque_filter_alpha` EMA | applies (τ ≈ 3.2 ms at the default alpha) | none — steps instantly |
| `i_d` | forced to 0 (pure quadrature) | yours to set |
| Needs valid `Kt` / `gear_ratio` | yes | no |

> ⚠️ **Current mode bypasses `torque_limit` entirely.** In torque mode the ceiling is
> `min(torque_limit, current_limit × Kt × gear_ratio)`; in current mode it is just
> `current_limit` (default 20 A). Use torque mode when you care about force at the joint;
> reserve current mode for motor characterization (Kt verification, current-loop tuning, `i_d`
> experiments).

**Telemetry caveat:** `PositionController_update` runs unconditionally, and `MODE_CURRENT` falls
into its `else` branch — so the firmware still computes `torque_setpoint` in current mode and then
**discards** it (the torque→`i_q` conversion is gated on POSITION/VELOCITY/TORQUE). `run`
therefore shows `i_q_set` (`i_q_setpoint`, 0x0C8 — the POST-clamp value, matching
`torque_setpoint`'s semantics) instead of `tq_set` in current mode; do not read `torque_setpoint`
over SDO there and believe it. The same applies to DAMPING, which is where a fault lands you.

### IDLE is freewheel, not brake

`MODE_IDLE` de-energizes the powerstage completely. For a gravity-loaded arm that means it
**falls**. `MODE_DAMPING` short-brakes (0,0,0 PWM). The tools pass through DAMPING before IDLE
when leaving **any** driving mode — including torque and current, where a constant torque command
means constant acceleration, so the arm is moving fastest exactly when the run ends. The final
state is still IDLE, so support the arm before the run finishes.

---

## Step 1 — Pre-flight

```bash
cd tools
python3 recoil_can.py discover                      # proves the bus; prints the real device id
python3 recoil_can.py --device-id 14 status
python3 recoil_can.py --device-id 14 diag
```

`discover` sends a **broadcast** SDO (device field 0), which the firmware answers regardless of its
configured ID. So an empty result means a physical-layer problem — termination, bitrate, wiring —
**not** an ID mismatch.

What `diag` must show before you touch anything:

| Field | Want | If not |
|---|---|---|
| `primary` / `secondary` | `on-bus`, `status=0x2_ [MD] ok` | `ML`/`MH` = magnet too weak/strong; adjust airgap |
| `agc` | mid-range (0–255 at 5 V) | Railed = airgap out of range |
| `boot read errors` | `0/64` both | Non-zero on **one** encoder = that device's wiring/magnet, not the shared bus |
| `max_consec` | `1` | 1 means every glitch was isolated and self-recovered. Approaching 10 (`ENCODER_FRAME_ERROR_FAULT_THRESHOLD`) means real bursts and you will hit `ENCODER_FAULT` |
| `fail_stage` | `OK` | See Step 5 |

If the bus is wedged (`start_fail` climbing, `last_i2c=AF/BERR`):

```bash
python3 recoil_can.py --device-id 14 recover
```

This issues `FUNC_SYSTEM`/`SYSTEM_CMD_RECOVER_I2C`. The ISR only sets a flag; the actual
bit-bang recovery runs in the foreground `updateService` and **only while de-energized**
(DISABLED/IDLE/DAMPING).

*Firmware ground truth: `MotorController_measureEncoderReadIntegrity`, `captureEncoderHealth`,
`Encoder_recoverBus`.*

---

## Step 2 — Verify the secondary encoder direction BEFORE calibrating

This is the failure mode the `VERNIER_SECONDARY_SIGN` comment warns about at length: a wrong sign
mis-scales delta to `(31/16)·alpha`, which **passes calibration at home and mis-resolves
everywhere else**. Check it now, not after you have baked a bad calibration into flash.

```bash
python3 recoil_can.py --device-id 14 monitor --period 0.2
```

Hand-rotate the motor shaft steadily in one direction and watch `enc` vs `enc2`.

With the current `VERNIER_SECONDARY_SIGN = -1`, the **raw** `enc2` must track **opposite** to
`enc` (the 15T/16T external mesh reverses rotation). If it tracks the same direction, the DIR pin
state does not match what the firmware assumes — fix the wiring, or flip the `#define` in
`motor_controller_conf.h` and rebuild.

> History on this rig: with the secondary breakout's DIR pad unsoldered (floating), the chip read
> forward and this was briefly `+1`. After fixing the missing DIR-to-VCC joint the count direction
> inverted, so software negates again. Any change to this sign or the DIR wiring means re-running
> Steps 2, 4 and 5 in full.

---

## Step 3 — Flux (electrical) calibration

Check `flux_offset` in `status`. If it is 0, commutation is not calibrated.

**Do this before vernier calibration.** `MotorController_runCalibrationSequence` calls
`Encoder_resetFluxOffset()`, which zeroes `n_rotations`, and it spins the motor — so the arm will
not be at home afterwards.

```bash
python3 recoil_can.py --device-id 14 mode calibration    # MOTOR SPINS
# wait for the LED to return to the IDLE rate, then:
python3 recoil_can.py --device-id 14 status              # expect flux_offset != 0, error == 0
```

`MODE_CALIBRATION` is watchdog-exempt, so no heartbeat is needed here.

---

## Step 4 — Vernier calibration

Motor stays **de-energized** throughout — `MODE_VERNIER_CALIBRATION` deliberately does not enable
PWM. Static reads only.

1. **Hand-place the arm at its home pose and let go.** Step 2 of the routine gates on
   `|encoder.velocity| < 2°/s` at the *motor* shaft, which is ≈0.13°/s at the arm through the
   15:1 gearbox. Genuinely stationary, not "close enough".

2. If the board booted into `MODE_DISABLED` with `ERROR_VERNIER_*` — expected on an uncalibrated
   unit, it fails closed by design — clear the latch. `error` is SDO-writable:

   ```bash
   python3 recoil_can.py --device-id 14 write error 0
   ```

3. Run it:

   ```bash
   python3 recoil_can.py --device-id 14 mode vernier_calibration
   sleep 1
   python3 recoil_can.py --device-id 14 status
   ```

### Reading the result

**The routine returns to `MODE_IDLE` on both success and failure — mode alone tells you nothing.**

| Check | Success |
|---|---|
| `calibrated:` | `YES` (`vernier_cal_magic == 0x5645524E`) — **this is the real success signal** |
| `error:` | `0x0000` |
| `phase_offset` | a real number, not NaN |
| `base_sector` | set |

`ERROR_VERNIER_CALIBRATION_FAILED` (0x8000) means one of:

- samples did not cluster (σ_ψ > 2°) — bad magnet, dropout, or **wrong `VERNIER_SECONDARY_SIGN`**
- the arm was not still
- the single-shot home read failed the 5° sector-residual check
- an I²C read failed

Calibration auto-persists to flash on success (`MotorController_storeConfig`) — no separate
`flash-store` needed.

*Firmware ground truth: `MotorController_runVernierCalibration`.*

---

## Step 5 — Verify resolution across the full travel

Calibration passing at home proves very little. This step is where a wrong sign or a marginal
`phase_offset` actually shows up.

**Power-cycle the board**, then:

```bash
python3 recoil_can.py --device-id 14 status
```

- Boot must reach `mode: 0x01 (idle)`. Still `0x00 (disabled)` means boot resolution failed.
- `fail_stage` names the stage that failed:

  | `fail_stage` | Meaning |
  |---|---|
  | `UNCALIBRATED` | magic word or `phase_offset` invalid — redo Step 4 |
  | `DRAIN` | I²C bus would not go idle — see Step 1 recovery |
  | `PRIMARY_READ` / `SECONDARY_READ` | that encoder failed a median-of-5 blocking read |
  | `PSI_MISMATCH` | geometry disagrees — wrong sign, slipped gear, or bad calibration |

- `psi_err` should be well under 5°. Creeping toward 5° means marginal geometry.
- `arm pos` should match the physical pose.

**Then repeat across the range:** hand-move the arm to several positions spanning its full
travel, power-cycling at each, and confirm `n_rotations` and `arm pos` are correct every time.
Do not skip this — resolution that works only near home is exactly the wrong-sign signature.

Optionally re-zero the reported frame at any pose:

```bash
python3 recoil_can.py --device-id 14 set-zero
```

Remember: this shifts only where zero reads, **not** the limits or the overtravel guard.

---

## Step 6 — Set the soft limits

**Mandatory before any energizing mode.** Both tools refuse to energize without finite, ordered
limits, because without them the overtravel guard is inert and torque/velocity/current modes have
no position protection whatsoever. They live in the **raw** frame, so convert from your desired
host range using `position_offset` from `status`:

```bash
python3 recoil_can.py --device-id 14 write position_limit_lower <host_lo + offset>
python3 recoil_can.py --device-id 14 write position_limit_upper <host_hi + offset>
python3 recoil_can.py --device-id 14 flash-store
python3 recoil_can.py --device-id 14 read position_limit_lower   # writes are unacked — read back
python3 recoil_can.py --device-id 14 read position_limit_upper
```

The overtravel guard trips at `POSITION_OVERTRAVEL_MARGIN` (~15° at the arm) **past** a limit and
faults to DAMPING. It is inert while the limits are the default ±INFINITY, and it requires
`vernier_initialized`.

---

## Step 7 — Running the modes

> ⚠️ Everything from here energizes the motor. Arm on a safe jig, clear of people and obstacles.

### Position — use the jog tool

`recoil_jog.py` runs the full prerequisite gate, does the host↔raw conversion for you, enforces a
per-command step limit, feeds the watchdog for the whole session, and e-stops to IDLE on exit.

```bash
python3 recoil_jog.py --device-id 14 --max-step 0.05
```

At the prompt: a bare number is an **absolute** host-frame target, `+0.02` / `-0.02` is a
**delta**, `s` prints status (position, mode, `tq_set`/`torque_limit`, `i_q`, error), `q` or
Ctrl-C e-stops (→ DAMPING → IDLE, verified by readback).

If a move reports **NOT settled**, check the `tq_set/torque_limit` pair it prints: saturated means
the torque clamp is the binding constraint and you should raise `torque_limit`, not the gains.
The tool says so explicitly when it detects saturation.

### Velocity / torque / current — use `run`

`run` stages the setpoint, verifies the readback, confirms with the operator, holds the mode for a
bounded duration with the watchdog fed, streams telemetry, aborts on any error, and always returns
to IDLE with the setpoint zeroed.

```bash
# velocity: rad/s at the arm, clamped to velocity_limit
python3 recoil_can.py --device-id 14 run velocity --target 0.5 --duration 3

# torque: N·m at the arm, clamped to torque_limit
python3 recoil_can.py --device-id 14 run torque --target 0.2 --duration 3

# current: A on the q-axis, clamped to current_limit ONLY — torque_limit does not apply
python3 recoil_can.py --device-id 14 run current --target 1.0 --duration 2

# position: host-frame rad; defaults to holding the current position if --target is omitted
python3 recoil_can.py --device-id 14 run position --target 0.1 --duration 3
```

### The gate `run` and `recoil_jog` share

Both run the **same** prerequisite check (`recoil_can.check_prerequisites`) — deliberately, so
`run` cannot become a weaker door into an energized motor. It refuses unless all of these hold:

| Check | Why |
|---|---|
| device reachable | — |
| `error == 0` | clear with `write error 0` after fixing the cause |
| `mode == IDLE` | DISABLED usually means boot vernier resolution failed |
| `flux_offset != 0` | uncalibrated commutation; in torque/current there is no outer loop to notice wrong-signed torque |
| `bus_voltage >= undervoltage_threshold` | — |
| `velocity_limit`, `torque_limit`, `current_limit` all **finite and > 0** | `inf` disables clamping; a **negative** limit inverts `clampf` into a constant full-scale output of the wrong sign |
| `position_kp > 0` | position mode only |
| finite, ordered `position_limit_*` | otherwise the overtravel guard is inert |

It additionally refuses on a bad limit-override readback, a non-finite target, an out-of-range
position target, an out-of-bounds `--duration`/`--period`/`--heartbeat-period`, or a heartbeat
period too slow for the board's actual watchdog deadline.

`--yes` skips only the final confirmation prompt, never the gate. Use it in scripted sweeps you
have already validated interactively.

---

## Step 8 — Testing torque limits

There are two independent ceilings and the effective one is the lower:

| Param | Offset | Units | Clamps |
|---|---|---|---|
| `torque_limit` | 0x030 | N·m at the arm | `torque_setpoint`, and the position integrator |
| `current_limit` | 0x074 | A | `i_q_setpoint` / `i_d_setpoint` |

The chain is `i_q_target = torque_setpoint / Kt / gear_ratio`, then
`i_q_setpoint = clamp(i_q_target, ±i_limit)`. So:

> **effective torque ceiling ≈ min(`torque_limit`, `current_limit` × Kt × `gear_ratio`)**

Firmware defaults are `torque_limit = 1.0` N·m and `i_limit = 20` A, so out of the box you are
almost certainly limited by `torque_limit` — and raising it alone keeps working until you hit the
current ceiling.

Check your headroom first:

```bash
python3 recoil_can.py --device-id 14 read gear_ratio
python3 recoil_can.py --device-id 14 read current_limit
```

### The sweep

Arm stalled against a fixture. Each `run` invocation is self-contained and watchdog-fed:

```bash
for T in 0.2 0.4 0.6 0.8 1.0; do
  echo "=== torque_limit = $T ==="
  python3 recoil_can.py --device-id 14 run torque \
      --target 5.0 --torque-limit "$T" --duration 2 --period 0.5 --yes
done
```

`--target 5.0` deliberately over-commands so the clamp is what you are measuring.
`--torque-limit` is applied and read back **before** energizing, and is **not** flashed — so a
power cycle restores the safe value. Non-finite and non-positive overrides are rejected: `inf`
disables clamping entirely, and a negative limit turns `clampf` into a constant full-scale output
of the wrong sign.

Each `run` is a separate process but each is self-contained and watchdog-fed, so the loop is safe.
A shell loop calling `mode torque` / `read` / `mode idle` would **not** be — no process lives long
enough to hold a heartbeat.

### Interpreting it

- `tq_set` in the telemetry (`torque_setpoint`, 0x04C) should saturate at your `torque_limit`.
  That is the torque clamp working. (Shown only for position/velocity/torque — in any other mode,
  including DAMPING after a fault, `run` shows `i_q_set` instead, because `torque_setpoint` is
  computed and discarded there.)
- **`tq_set` pinned at `torque_limit` does not prove the torque clamp is the binding one.**
  `torque_setpoint` is computed entirely inside `PositionController_update` and knows nothing
  about `i_limit`. If `current_limit` is the real ceiling, the arm stalls, position error keeps
  growing, `position_kp × error` keeps rising, and `tq_set` pins at `torque_limit` anyway. Raising
  `torque_limit` then changes nothing — and arms a larger command that will lurch the arm the
  moment you do lift the current limit.
- **The disambiguator is `i_q_setpoint` (0x0C8), not `i_q_measured`.** It is post-clamp against
  `i_limit`, so it pins at `current_limit` exactly when the current clamp is binding. `clampf`
  returns the bound bit-for-bit, so compare exactly. `recoil_jog` does this automatically and
  names the binding clamp on a failed settle.
- `torque_filter_alpha` (0x070) applies an EMA **before** the clamp, so `tq_set` ramps rather than
  stepping. At the default 0.145364 and 2 kHz that is τ ≈ 3.2 ms (the "50 Hz cutoff" in
  `position_controller.c`) — a few tens of ms to settle. It is a runtime param; 1.0 disables
  filtering.
- Watch `err` on every line. `OVER_CURRENT` (0x0100) means you exceeded the hardware protection,
  which is a separate and lower-level ceiling than either software limit.

---

## Quick reference

| Task | Command |
|---|---|
| Find the board | `recoil_can.py discover` |
| Full state dump | `recoil_can.py --device-id N status` |
| Encoder health | `recoil_can.py --device-id N diag` |
| Watch encoders live | `recoil_can.py --device-id N monitor` |
| Unwedge I²C | `recoil_can.py --device-id N recover` |
| Clear a latched error | `recoil_can.py --device-id N write error 0` |
| Flux calibration | `recoil_can.py --device-id N mode calibration` |
| Vernier calibration | `recoil_can.py --device-id N mode vernier_calibration` |
| Re-zero the host frame | `recoil_can.py --device-id N set-zero` |
| Persist config | `recoil_can.py --device-id N flash-store` |
| Jog interactively | `recoil_jog.py --device-id N` |
| Hold a driving mode | `recoil_can.py --device-id N run <mode> --target X` |
| E-stop | Ctrl-C in `run`/`jog` (→ DAMPING → IDLE, verified), or `mode damping` |

## Maintenance

When the firmware changes, re-check this playbook against:

| If this changes in firmware | Update here |
|---|---|
| `Parameter` enum offsets | `PARAMS` in `recoil_can.py` — the SDO handler does raw pointer arithmetic, so a stale offset writes to the wrong field |
| `Mode` / `ErrorCode` enums | `MODES` / `ERROR_BITS`, and the mode tables above |
| `SAFETY_WATCHDOG_ENABLED`, TIM2 config, or which frames reset it | the watchdog section and `Keepalive` |
| Which modes clamp `position_limit_*` | "Only position mode enforces the position limits" |
| The mode dispatch in `MotorController_update` (which modes convert torque→`i_q`) | "`torque` vs `current` mode", and `run`'s telemetry column choice |
| `POSITION_OVERTRAVEL_MARGIN`, `VERNIER_SECTORS`, `VERNIER_SECTOR_BIAS` | Steps 5–8 |
| `VERNIER_SECONDARY_SIGN` or the DIR wiring | **Re-run Steps 2, 4 and 5 in full** |
| `VERNIER_ENABLED = 0` | Steps 2, 4, 5 do not apply; the board boots to IDLE with relative position |

## Related docs

- `docs/secondary-encoder-vernier-phase1.md` — vernier math, design rationale, and the Level 0–6
  test plan this playbook operationalizes
- `docs/secondary-encoder-vernier-phase2.md` — deferred runtime slip detection
