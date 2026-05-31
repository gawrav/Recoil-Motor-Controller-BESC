# Add secondary AS5600L encoder for absolute arm position

## Context

The firmware currently uses a single AS5600 magnetic encoder on the motor shaft. The encoder is single-turn (12-bit absolute within one motor revolution), and software accumulates multi-turn position via `n_rotations` in `Encoder_update` (encoder.c:69). **This counter resets to 0 at every power cycle.**

Because the motor drives a robotic arm joint through a 15:1 cycloidal gearbox, one motor revolution = 24° of arm motion. So at power-up, the firmware doesn't know which 24° "sector" of the arm's range it's currently in — true absolute arm position is unrecoverable without homing.

**Goal**: add a second AS5600L encoder on an auxiliary shaft driven by a 15T/16T gear pair off the motor shaft. The two encoders form a **Nonius vernier**: their phase difference uniquely identifies which of 16 motor-revolution sectors we're in, giving absolute arm position over ~384° of arm motion at boot. Joint range is <360° so this covers the full mechanical envelope.

**Why AS5600L specifically**: it's pin-compatible with AS5600 (SOIC-8 footprint and pin order are identical per both datasheets) and has a **programmable I2C address** that defaults to `0x40` — distinct from AS5600's fixed `0x36`. This lets both encoders share the existing I2C1 bus on J8 with zero board modifications.

## Hardware setup (prerequisites before code work begins)

1. **Swap the chip on a spare AS5600 breakout**: hot-air desolder the AS5600 chip and replace with AS5600L SOIC-8. Pin-identical, all passives stay. Verify SDA/SCL pull-ups: if both breakouts (primary's and secondary's) have onboard pull-ups, remove the pull-ups from the secondary's breakout to avoid doubling them on the bus (cumulative pull-up should stay >2 kΩ).
2. **Mount the secondary AS5600L** over a diametrically-magnetized magnet on the 16-tooth gear's shaft. Magnet alignment within ±0.5 mm of the chip's sensitive center.
3. **Build a 4-wire Y-cable** from J8: VCC (J8 pin 2), GND (J8 pin 1), SCL (J8 pin 3 → PB8), SDA (J8 pin 4 → PB7) splits into two stubs, one to each encoder breakout. Total bus length <30 cm. J8 pin 5 (PB6) remains unconnected.
4. **No OTP burn needed** — verified against the AS5600L datasheet (DS000545, pages 12/15/18/20): the default 7-bit slave address is `0x40` and resets to that on every power-up (the I2CADDR OTP factory value). The primary AS5600 is fixed at `0x36`. They coexist on one bus out of the box — just swap, wire, and they answer at `0x36`/`0x40`. (If a non-default address were ever needed: write `I2CADDR` reg `0x20`, then Burn_Setting = write `0x40` to BURN reg `0xFF`. Not needed here.)

After hardware is in place, the firmware needs the changes below.

### Verified AS5600/AS5600L register reference (from datasheets)

- **ANGLE** (filtered, what the driver reads): `0x0E`/`0x0F`. **RAW ANGLE**: `0x0C`/`0x0D`. Identical on both parts.
- **STATUS** `0x0B`: `MD` = bit 5 (magnet detected), `ML` = bit 4 (too weak), `MH` = bit 3 (too strong).
- **AGC** `0x1A`: range 0–255 at 5 V, 0–128 at 3.3 V (mid-range = ideal). **MAGNITUDE** `0x1B`/`0x1C`.
- **CONF** `0x07`/`0x08`: `OUTS`(5:4) output mode (AS5600L: only `10`=PWM, no analog), `PWMF`(7:6), `SF`(9:8) slow filter, `FTH`(12:10), `WD`(13).
- **I2CADDR** `0x20` (R/W/P), **I2CUPDT** `0x21` (R/W, volatile address strobe).

## Branch strategy

All work happens on a dedicated feature branch off `main`:

- Branch name: `feature/secondary-as5600l-vernier`
- Created from current `main` (HEAD at `b11768f`).
- Commits scoped per logical step (encoder driver parameterization → struct additions → dedicated calibration mode → boot resolution → flash persistence → param/error IDs), so each is independently reviewable.
- No merge to `main` until the full bench testing plan below passes. Firmware that controls a motor on a physical arm must not land on `main` half-verified.

## Scope: Phase 1 (now) vs Deferred

**Phase 1 (this plan, to implement now): dedicated vernier calibration + boot-time resolution.**
Two pieces:
1. A dedicated **`MODE_VERNIER_CALIBRATION`** (independent of the electrical flux calibration) that produces the two calibration constants — run once per assembly.
2. **Boot-time resolution**: the secondary encoder is read **exclusively at boot**, before the motor is energized (`MODE_DISABLED`/pre-`MODE_IDLE`), to seed `n_rotations` from the stored constants. At boot the motor isn't spinning, so the blocking-read-with-TIM1-mask approach is completely safe — no async-I2C and no runtime-FOC-blip complexity.

**Deferred (kept in this doc as future work, NOT implemented now): runtime sanity check.**
The continuous drift-detection check (predict secondary from accumulated primary angle at ~10 Hz) is documented below but **not built in Phase 1**. If field experience shows we need live slip detection, it'll be added then — and at that point it must use the async single-in-flight read scheme (not blocking+mask) to avoid a torque blip while the motor runs. See "Deferred: runtime sanity check" section.

## Approach (Phase 1)

- Keep the existing `Encoder` struct and driver — parameterize it on I2C address so it can serve both primary and secondary.
- Add a `MotorController.encoder_secondary` member alongside the existing `encoder`.
- At boot, with the TIM1 update interrupt masked, run a one-shot **vernier resolution** routine: blocking-read both encoders, compute the current motor-revolution sector from the phase difference, and seed `encoder.n_rotations` (and `position_raw`/`position`) accordingly.
- **The 10 kHz FOC loop is genuinely untouched** — primary read and control math stay exactly as today; the secondary is never read from the FOC loop. Calibration is **static** (no motor motion), so `MODE_VERNIER_CALIBRATION` is a safe non-driving mode (like `MODE_IDLE`) — it only needs a `setMode` case, an `updateService` dispatch, and a watchdog exemption. No PWM-branch / current-controller / flux-cal wiring.

## Independent review — incorporated corrections

An independent subagent reviewed this plan against the actual source. The vernier math (below) was confirmed correct in full, including that the `−θ_p`-omitting sector formula is a genuine bug. The review found integration issues now folded into the plan:

- **[CRITICAL] Missing helpers**: `wrapToPi` and `deg2rad` do **not** exist — only `wrapTo2Pi` is in foc_math.h (line 60). Both must be added to `foc_math.c/h`. `wrapToPi` must return a signed result in [−π, π) for the `fabsf(error)` thresholds to work. See "New helpers" below.
- **[CRITICAL] Two-device I2C requires a state machine**, not a simple READY-gate. The primary read at motor_controller.c:343 is *unconditional*, and `HAL_I2C_MasterRxCpltCallback` (app.c:65) is empty — so the driver relies entirely on call ordering to know which device a completed transfer belongs to. Interleaving a second device breaks that invariant. See "I2C arbitration" below.
- **[CRITICAL] Boot-time ISR race**: TIM1 PWM/ISR is already running (`PowerStage_start` at motor_controller.c:86) before the proposed vernier-resolution call site. The 10 kHz ISR calls `Encoder_update` on the same bus and writes the same `encoder` struct that resolution reads and seeds → data race. Resolution + seed must run with the TIM1 update interrupt masked, OR be moved before `PowerStage_start`.
- **[MAJOR] Seed consistency**: seeding `n_rotations = q` while `position_raw` is stale (0 from init) can trip the multi-turn crossing logic (encoder.c:69) on the next update and corrupt the sector. Seed `position_raw`, `position`, and `n_rotations` together, only after a verified-good primary read.
- **[MAJOR] Struct field placement**: SDO access (motor_controller.c:714) is raw byte-offset addressing. New fields MUST be appended at the **end** of `MotorController` (after `encoder`, whose trailing `flux_offset_table[128]` is currently the last addressable region) or every existing `PARAM_*` offset shifts and silently breaks CAN + Flash. Flash fit confirmed OK: struct grows to ~1416 bytes, under the 2048-byte page.
- **[MAJOR → Phase 2] Sanity-check call rate**: this concerned the runtime sanity check, now **deferred to Phase 2**. The call-rate caveat (10 kHz site needs divisor 1000, not 200) is captured in the Phase 2 section.
- **[RESOLVED] Boot consistency threshold**: the boot consistency check now uses **5°** (see resolution routine), matching what the deferred runtime check will use. No 9° value remains in Phase 1.
- **[MINOR] `Encoder_init` redundancy**: it calls `HAL_I2C_Init` in a loop (encoder.c:32) — don't re-init the shared `hi2c1` when bringing up the second encoder.
- **[MINOR] Confirm `gear_ratio = 15`**: arm-angle conversion at motor_controller.c:350 divides by the runtime-loaded `position_controller.gear_ratio`. Confirm the deployed Flash config actually has it set to 15, independent of the vernier work.
- **[MINOR] Stale text**: Test 6.1 still references the removed `secondary_gear_ratio` field — corrected below.

### New helpers (add to `foc_math.c/h`)

```c
// signed wrap to [-π, π)
static inline float wrapToPi(float x) {
    x = wrapTo2Pi(x);
    if (x >= M_PI_F) x -= M_2PI_F;
    return x;
}
#define deg2rad(d)  ((d) * (M_PI_F / 180.0f))
```

### I2C access in Phase 1: blocking + TIM1 mask only (no async arbitration)

Phase 1 reads the secondary in exactly two places — boot resolution and `MODE_VERNIER_CALIBRATION` — and **both use a synchronous blocking read with the TIM1 update interrupt masked** for the ~78 µs duration. The 10 kHz FOC loop's primary read path is unchanged, and the secondary is never read during normal operation. So there is **no two-device-on-one-bus concurrency in Phase 1** and no async state machine is required.

The async "in-flight device" arbitration scheme (needed only when the secondary must be read *while the motor runs*, i.e. the runtime sanity check) is **Phase 2** — see the Phase 2 (Deferred) section at the end of this document.

## Vernier mathematics (detailed)

### Mechanical setup and notation

- Motor shaft carries the **primary** AS5600 and a **15-tooth** gear.
- The 15T gear meshes with a **16-tooth** gear; the **secondary** AS5600L sits on that 16T shaft.
- (Separately, the motor drives the arm through a **15:1 cycloidal** gearbox — this is NOT involved in the vernier math. The vernier resolves absolute *motor* angle; arm angle is then motor angle / 15.)

Symbols:
- `α` = continuous (unwrapped) motor shaft angle in radians.
- `θ_p = α mod 2π` = primary reading, ∈ [0, 2π).
- `θ_s` = secondary reading, ∈ [0, 2π).
- A 15T driver turning a 16T driven gear: the driven (secondary) shaft rotates at **15/16** the motor rate. So secondary shaft angle = `(15/16)·α`, and `θ_s = (15/16)·α mod 2π`.

### Why 16 sectors

Define the phase difference:

```
Δ = (θ_p − θ_s) mod 2π
  = (α − (15/16)·α) mod 2π
  = (α/16) mod 2π
```

So **Δ advances by exactly `2π/16` per motor revolution** (when α increases by 2π, Δ increases by 2π/16). After 16 motor revolutions Δ has swept a full 2π and the pattern repeats. Hence there are exactly **16 distinguishable sectors** — i.e. the vernier resolves the motor revolution count modulo 16.

- 16 motor revs = 16/15 arm revs ≈ **1.067 arm revolutions ≈ 384°** of unique absolute range. Joint travel is <360°, so this covers the full envelope with margin.

### Boot-time sector resolution (the correct formula)

Write the motor revolution count as `α = 2π·N + θ_p` where `N = floor(α/2π)` is the integer revolution count. Substituting into `Δ = (α/16) mod 2π` and writing `N = 16·m + q` with `q ∈ {0..15}`:

```
Δ = ( (2π/16)·N + θ_p/16 ) mod 2π
  = (2π/16)·q + θ_p/16          (the 2π·m term vanishes under mod 2π)
```

Both terms are non-negative and sum to less than 2π, so no wrap. Solving for the sector `q`:

```
16·Δ = 2π·q + θ_p
  ⟹  q = (16·Δ − θ_p) / (2π)
```

With measurement noise, round to the nearest integer:

```
q = round( (16·Δ − θ_p) / (2π) )  mod 16
```

**⚠ Correction to earlier draft**: the previous pseudo-code used `q = round(Δ·16/(2π))`, which **omits the `− θ_p` term**. That version is only correct at θ_p ≈ 0 and drifts by up to ±1 sector as θ_p approaches 2π — it would mis-identify the sector for roughly half of all boot positions. The `− θ_p` correction is required. Equivalent robust form: subtract the known fractional contribution first, then round —

```
ψ = wrapTo2Pi(Δ − θ_p/16)     // ≈ q·(2π/16), noise-centered
q = round( ψ · 16 / (2π) ) mod 16
```

### Error margin for sector ID

After the `− θ_p` correction, the rounding decision has **±½ sector = ±2π/32 = ±11.25°** of margin in the Δ domain. Total expected error in Δ from both encoders' INL + magnet mounting + 15T/16T backlash is ~1° typical, ~3.5° worst-case (per the accuracy analysis). So the margin is **~3–8×** — sector ID is extremely robust.

> **Note on the ×16 (preempting a common miscount):** the sector formula multiplies by 16 (`round(ψ·16/2π)`), which tempts one to divide the margin by 16. Don't. A Δ-error enters `ψ` at **coefficient 1** (`ψ = Δ − θ_p/16 − offset`, so `δ_ψ ≈ (15/16)e_p − e_s`, same order as the raw INL — *not* amplified). The 16× and the half-sector margin both live in the same conversion, so the margin expressed in the Δ/ψ domain is **±11.25°**, full stop. This matches the 20,000-boot simulation (1° noise → robust). The boot consistency check (`|psi_error| > 5°`, ψ-domain) fails closed well inside this margin.

### Seeding the primary accumulator

Once `q` is known, the primary's multi-turn counter is seeded:

```
controller->encoder.n_rotations = q;
```

After this, `Encoder_getPosition` returns the absolute motor angle `θ_p + 2π·q`, and dividing by the 15:1 cycloidal ratio (already done at motor_controller.c:350) yields absolute arm angle.

### Runtime sanity check → Phase 2

The continuous drift-detection math (predict the secondary from the accumulated primary angle and compare; one-rev slip = 22.5°) is **deferred to Phase 2** — see the Phase 2 (Deferred) section at the end.

### Constants to define (replaces the misleading `secondary_gear_ratio = 15.0`)

The earlier draft stored `secondary_gear_ratio = 15.0f`, which conflated the cycloidal ratio with the encoder gear pair. Use explicit, correctly-named constants instead:

```c
#define VERNIER_SECTORS                 16        // = larger tooth count (pattern repeats every 16 motor revs)
#define VERNIER_SECONDARY_NUM           15        // 15T driver
#define VERNIER_SECONDARY_DEN           16        // 16T driven
// secondary shaft rate = NUM/DEN of motor = 15/16
```

The 15:1 cycloidal ratio remains `position_controller.gear_ratio` (already in the codebase) and is unrelated to these.

## Implementation

### File: `Core/Inc/foc_math.h` and `Core/Src/foc_math.c`

Add `wrapToPi` (signed wrap to [−π, π)) and the `deg2rad` macro — neither currently exists, only `wrapTo2Pi` (foc_math.h:60). See "New helpers" in the review section above. Required by both the resolution routine and the sanity check; without them the firmware does not compile (review C1).

### File: `Core/Inc/encoder.h`

- Add `uint16_t i2c_address;` to `Encoder` struct (store the already-shifted form, i.e. `address << 1`).
- Add constant: `#define AS5600L_I2C_ADDR_SECONDARY 0x40U`.
- Update `Encoder_init` signature to take an `i2c_address` parameter, OR initialize the field from a caller-set value.

### File: `Core/Src/encoder.c`

- `Encoder_init` (currently encoder.c:11): replace hardcoded `AS5600_I2C_ADDR << 1` in the probe-loop `HAL_I2C_Mem_Read` call (encoder.c:37) with `encoder->i2c_address`. Accept address as parameter. **Do not re-run `HAL_I2C_Init` for the second encoder** (encoder.c:32 re-inits the shared `hi2c1` — harmless but redundant; guard it so it only runs once for the primary). (review m1)
- `Encoder_update` (currently encoder.c:49): replace hardcoded `AS5600_I2C_ADDR << 1` in `HAL_I2C_Master_Receive_IT` with `encoder->i2c_address`.
- No other changes — the rest of the driver is address-agnostic.

### File: `Core/Inc/motor_controller.h`

**Append all new fields at the END of the struct, after `encoder`** (review M4 — appending avoids shifting existing `PARAM_*` byte offsets):

- `Encoder encoder_secondary;` — same type as existing `encoder`
- **`float vernier_phase_offset;`** — calibrated magnet-fingerprint correction (radians). Persisted in Flash. See calibration section.
- **`uint8_t vernier_base_sector;`** — calibrated sector renumbering origin (0..15). Persisted in Flash.
- `uint8_t vernier_sector;` — live resolved sector 0..15, set at boot (not persisted)
- `uint8_t vernier_initialized;` — boolean flag (not persisted)
- `uint16_t vernier_sanity_counter;` — tick counter for the deferred sanity check; add now (cheap) or defer

The encoder gear pair ratio is fixed in hardware (15T/16T), so it's compile-time constants (`VERNIER_SECTORS`, `VERNIER_SECONDARY_NUM`, `VERNIER_SECONDARY_DEN`), not a runtime field. Do **not** add a `secondary_gear_ratio` field. The two **calibrated** constants (`vernier_phase_offset`, `vernier_base_sector`) ARE runtime/Flash values — produced by the dedicated calibration mode below.

### File: `Core/Inc/motor_controller_conf.h`

- Add to `Mode` enum (motor_controller_conf.h:116):
  - **`MODE_VERNIER_CALIBRATION = 0x06U`** — dedicated vernier calibration, separate from the electrical `MODE_CALIBRATION = 0x05U`. Keeps magnet/sector calibration independent of flux-offset calibration.
- Add to `ErrorCode` enum:
  - `ERROR_VERNIER_INCONSISTENT = (1 << 14)` — boot-time sector ID failed (cross-check error > threshold). **Phase 1.**
  - `ERROR_VERNIER_CALIBRATION_FAILED = (1 << 15)` — calibration routine couldn't fit `vernier_phase_offset` or reach the reference. **Phase 1.**
  - (`ERROR_ENCODER_DRIFT` for the deferred runtime sanity check — reserve a bit when that's built.)
- Add parameter IDs (appended after existing `PARAM_*`) for the secondary encoder fields and `PARAM_VERNIER_PHASE_OFFSET` (read-only telemetry of the float). Each is a byte offset into `MotorController` (SDO mechanism at motor_controller.c:700 exposes them).
- **Decision (review MN-3)**: the SDO write path does a full **32-bit** store (`*((uint32_t*)((uint8_t*)controller + parameter_id)) = ...`, motor_controller.c:721). Writing a `uint8` field via SDO would clobber the 3 adjacent bytes. So `vernier_base_sector`/`vernier_sector` (uint8) and `vernier_phase_offset` are **calibration-owned and NOT exposed as writable SDO parameters** — same model as `flux_offset`. Expose them read-only for telemetry only (or place the two uint8s in their own padded `uint32`-aligned word so a stray write can't corrupt neighbors). Calibration writes them directly in RAM, not via SDO.

### File: `Core/Src/motor_controller.c`

#### `MotorController_init` (currently motor_controller.c:26)

After existing primary encoder init:
- Set `controller->encoder.i2c_address = AS5600_I2C_ADDR << 1;` (preserves existing behavior).
- Initialize secondary encoder fields: `controller->encoder_secondary.i2c_address = AS5600L_I2C_ADDR_SECONDARY << 1;` plus the same configuration block as primary (CPR, filter alpha, etc.).
- Call `Encoder_init(&controller->encoder_secondary, &hi2c1);` after primary init succeeds.
- Set `controller->vernier_initialized = 0;` and `controller->vernier_sanity_counter = 0;`. (Gear pair ratio is compile-time, not a runtime field.)

Placement matters (review pass 3): `MotorController_clearError` runs at motor_controller.c:116, immediately before the `setMode(MODE_IDLE)` at line 117. Call resolution **after** line 116's `clearError` — otherwise a `clearError` after resolution would wipe `ERROR_VERNIER_INCONSISTENT`. So the order is: `calibratePhaseCurrentOffset` → `clearError` (line 116) → **`resolveAbsolutePosition()` + error-set** → guarded `setMode`.
- Call new function `MotorController_resolveAbsolutePosition()`.
- **Fail-closed wiring (review): the existing line 117 unconditionally sets `MODE_IDLE`.** Leaving the mode alone on failure is NOT enough — line 117 would still force `MODE_IDLE`. So **guard line 117**: only `setMode(MODE_IDLE)` if resolution succeeded AND the unit is calibrated (`vernier_initialized`); otherwise set `ERROR_VERNIER_INCONSISTENT` (or `ERROR_VERNIER_CALIBRATION_FAILED` if uncalibrated) and stay in `MODE_DISABLED`. An uncalibrated unit (no valid `vernier_phase_offset` in Flash) also fails closed here and requires `MODE_VERNIER_CALIBRATION` first. (Line 117 is the last statement in `MotorController_init` — nothing after it assumes IDLE, so staying DISABLED is safe.)

#### New function: `MotorController_resolveAbsolutePosition`

~40 lines. **Must run with the TIM1 update interrupt masked** (the 10 kHz ISR is already live at this call site — see review C3). Bracket the whole routine:

```
__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);   // stop the 10 kHz Encoder_update preemption
// CRITICAL (review C-1): masking only stops the NEXT Encoder_update; an IT-driven
// receive kicked off by the last ISR may still be in flight on hi2c1. A blocking
// HAL_I2C_Mem_Read would then return HAL_BUSY and read garbage. Drain the bus first:
uint32_t t0 = HAL_GetTick();
while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY && (HAL_GetTick() - t0) < 5) { }
if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    HAL_I2C_Master_Abort_IT(&hi2c1, AS5600_I2C_ADDR << 1);   // in-flight device = primary
    t0 = HAL_GetTick();
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY && (HAL_GetTick() - t0) < 5) { }
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)     // still stuck → fail closed
        { SET_BITS(error, ERROR_VERNIER_INCONSISTENT); return HAL_ERROR; }
}
... resolution body ...
__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
```

Logic (use blocking reads here, not the async IT path, so we control ordering):
```
// blocking read both encoders directly (bus is ours — ISR masked AND drained)
read_primary_blocking(&theta_p_raw);     // HAL_I2C_Mem_Read from 0x36, register 0x0E
read_secondary_blocking(&theta_s_raw);   // HAL_I2C_Mem_Read from 0x40, register 0x0E

theta_p = (float)theta_p_raw / cpr * M_2PI_F;   // ∈ [0, 2π) motor frame
theta_s = (float)theta_s_raw / cpr * M_2PI_F;   // ∈ [0, 2π) 16T-gear frame

delta = wrapTo2Pi(theta_p - theta_s);                          // = (α/16) mod 2π = q·(2π/16) + θ_p/16

// Apply the calibrated magnet-fingerprint correction (vernier_phase_offset), then the
// −θ_p correction. See Vernier mathematics + Vernier calibration sections.
psi   = wrapTo2Pi(delta - theta_p / 16.0f - controller->vernier_phase_offset);   // ≈ q_raw·(2π/16)
q_raw = (int)lroundf(psi * (float)VERNIER_SECTORS / M_2PI_F);
q_raw = ((q_raw % VERNIER_SECTORS) + VERNIER_SECTORS) % VERNIER_SECTORS;

// Consistency check: reconstruct ψ from the chosen raw sector, compare residual.
psi_expected = q_raw * (M_2PI_F / (float)VERNIER_SECTORS);
psi_error    = wrapToPi(psi - psi_expected);
if (fabsf(psi_error) > deg2rad(5.0f)) {
    SET_BITS(controller->error, ERROR_VERNIER_INCONSISTENT);
    return HAL_ERROR;   // (interrupt re-enabled by the caller's bracket regardless)
}

// Renumber so the supercycle wrap falls outside the arm's travel (vernier_base_sector,
// set during calibration). n_rotations climbs monotonically across the operating range.
n_rot = (q_raw - controller->vernier_base_sector + VERNIER_SECTORS) % VERNIER_SECTORS;

// Seed atomically so the next Encoder_update's crossing logic (encoder.c:69) doesn't corrupt it (review M1):
controller->encoder.position_raw = theta_p_raw;                       // consistent with seed
controller->encoder.n_rotations  = n_rot;
controller->encoder.position     = theta_p + n_rot * M_2PI_F;         // absolute motor angle (continuous in-range)
controller->vernier_sector       = q_raw;
controller->vernier_initialized  = 1;
return HAL_OK;
```

Reuses existing `wrapTo2Pi` and `wrapToPi` from `foc_math.c/h`.

#### New function: `MotorController_runVernierCalibration` — dedicated `MODE_VERNIER_CALIBRATION`

A standalone routine, **independent of the electrical flux calibration** (`MotorController_runCalibrationSequence` / `MODE_CALIBRATION`). Triggered deliberately, once per assembly (re-run only if gears are re-meshed or a magnet is disturbed). Invoked from `MotorController_updateService` (motor_controller.c:425) when `mode == MODE_VERNIER_CALIBRATION`, mirroring how `MODE_CALIBRATION` is dispatched there.

**STATIC — the motor is never driven during vernier calibration.** `vernier_phase_offset` is a position-independent magnet fingerprint (so a single static reading captures it; no spin needed), and the home reference is established by the operator manually placing the arm at home (no hardstop seek). This makes calibration a pure read-and-compute routine and removes ALL motor-spin wiring. **No dependency on electrical flux calibration** either (we don't commutate here).

**Workflow**: operator manually moves the (unpowered, free-spinning) arm to its known home pose, then issues the calibrate command. Both steps then read at the current position. Because `vernier_phase_offset` is position-independent, Step 1 doesn't care where the arm is; Step 2 assumes it's at home.

It produces **`vernier_phase_offset`** (magnet fingerprint) and **`vernier_base_sector`** + **`position_controller.position_offset`** (the joint home).

**Design decision (one zero per joint):** calibration writes the **existing** `position_controller.position_offset` (arm frame), treated as "the joint home, set at commissioning, not runtime-rewritten" — same trust model as `flux_offset`. (`encoder.position_offset` stays 0 — reserved encoder-mount trim.)

```
// ===== Step 1: vernier_phase_offset — STATIC, position-independent, no motion =====
// Take a few readings at the current (arbitrary) position to average out electronic noise.
// frac(x) is the same at every position (magnet fingerprint), so no spin is needed.
accumulate over N samples (N ~ 16, motor still):
   __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
   drain_i2c_bus();                                  // wait HAL_I2C_STATE_READY + abort (review C-1)
   read_primary_blocking(&theta_p_raw_i);            // HAL_I2C_Mem_Read 0x36, reg 0x0E
   read_secondary_blocking(&theta_s_raw_i);          // HAL_I2C_Mem_Read 0x40, reg 0x0E
   __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
   theta_p_i = theta_p_raw_i / cpr * 2π;  theta_s_i = theta_s_raw_i / cpr * 2π;
   delta_i = wrapTo2Pi(theta_p_i - theta_s_i)
   x_i     = (16*delta_i - theta_p_i) / (2π)
   frac_i  = x_i - floorf(x_i)                        // ∈ [0,1)
vernier_phase_offset = 2π * circular_mean(frac_i)      // average only beats down electronic noise
if (circular spread of frac_i too large) → ERROR_VERNIER_CALIBRATION_FAILED   // bad magnet / dropout

// ===== Step 2: vernier_base_sector + position_controller.position_offset — arm at operator-placed home =====
// (arm is already at its known home pose)
// Settling gate (review): the operator may issue capture while the free arm is still
// drifting under hand/gravity. A single-shot read could latch a transient → wrong
// base_sector/offset. Require the joint to be still before latching:
if (fabsf(controller->encoder.velocity) > deg2rad(2.0f) /*per sec, tune*/)
    → ERROR_VERNIER_CALIBRATION_FAILED;   // "hold the arm still and retry"
__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE); drain_i2c_bus();
read_primary_blocking(&theta_p_raw); read_secondary_blocking(&theta_s_raw);  // RAW blocking — not the velocity-filtered encoder.position
__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
theta_p = theta_p_raw/cpr*2π;  theta_s = theta_s_raw/cpr*2π;
delta  = wrapTo2Pi(theta_p - theta_s)
psi    = wrapTo2Pi(delta - theta_p/16 - vernier_phase_offset)
q_raw  = lroundf(psi * VERNIER_SECTORS / 2π) mod VERNIER_SECTORS
vernier_base_sector = q_raw            // home → renumbered sector 0; wrap sits just below it

// fine zero — write the EXISTING arm-frame field (one zero per joint):
n_rot_ref = (q_raw - vernier_base_sector + VERNIER_SECTORS) % VERNIER_SECTORS;   // = 0 by construction
raw_absolute_arm = (theta_p + n_rot_ref*2π) / gear_ratio;   // = theta_p/gear_ratio
position_controller.position_offset = raw_absolute_arm - known_reference_arm_angle;   // arm frame

// ===== Step 3: persist =====
MotorController_storeConfig(controller);     // writes phase_offset, base_sector, position_controller.position_offset to Flash
MotorController_setMode(controller, MODE_IDLE);
```

**Note**: Steps 1 and 2 read at the same position (arm at home), so they can share one set of readings — Step 1's samples can be taken right at home. Kept separate above for clarity. The motor is in a non-driving state throughout (PWM off); the only hardware action is I2C reads with brief TIM1 masks.
```

Notes:
- Step 1 needs no knowledge of arm position — `frac(x)` is the magnet fingerprint, identical everywhere — so it works at the home position too (Steps 1 and 2 can share readings).
- Step 2's reference is the **operator-placed home**: a human moves the free (unpowered) arm to its known home pose and issues the capture command. It must be a repeatable physical pose (a fixture or alignment mark helps). No motor-driven hardstop seek in Phase 1 (that would reintroduce motion/commutation; deferred as a possible future enhancement).
- The whole routine is static reads + compute — the motor is never energized.

#### Wiring `MODE_VERNIER_CALIBRATION` — a NON-DRIVING (safe) mode

Because calibration is static (no motor spin — see above), `MODE_VERNIER_CALIBRATION` is a **safe, non-driving mode like `MODE_IDLE`**, not a motor-driving mode. This removes all the motor-spin wiring that earlier drafts needed:

1. **`MotorController_setMode` switch** — add `case MODE_VERNIER_CALIBRATION:` modeled on the **`MODE_IDLE`** case (motor_controller.c:147-151), NOT the `MODE_CALIBRATION` case at 153: set the LED, **do NOT enable PWM** (PWM stays disabled — `PowerStage_disablePWM` was already called at the top of `setMode`, motor_controller.c:137). Insert the new case near the IDLE case (147). Without its own case it hits `default` (line 197) → `ERROR_INVALID_MODE`.

2. **`MotorController_updateService` (motor_controller.c:425)** — add `if (mode == MODE_VERNIER_CALIBRATION) { MotorController_runVernierCalibration(controller); return; }` alongside the `MODE_CALIBRATION` branch.

3. **TIM2 watchdog exemption (app.c:43)** — add `MODE_VERNIER_CALIBRATION` to the exempt list (`DISABLED`/`IDLE`/`CALIBRATION`). Not because of a spin (there is none) but because it's a safe non-driving mode and the operator may take time positioning the arm before issuing the capture command; we don't want a spurious watchdog `MODE_DAMPING` transition.

**NOT needed anymore** (these were only for the deleted motor-spin design):
- ❌ No change to the PWM-output branch in `MotorController_update` — it stays untouched; `MODE_VERNIER_CALIBRATION` correctly falls into the `else` that keeps PWM disabled (motor_controller.c:419), which is exactly what we want for a non-driving mode.
- ❌ No change to `current_controller.c:103` — we never use the flux-angle override.
- ❌ No flux-calibration prerequisite — we don't commutate.

**`MotorController_update` is genuinely untouched in Phase 1.** The FOC read path and control math are unchanged; seeding `n_rotations` at boot shifts electrical angle by an exact multiple of 2π, which vanishes under `wrapTo2Pi` at motor_controller.c:385 (confirmed).

#### Runtime sanity check function → Phase 2

`MotorController_sanityCheckVernier` is **not built in Phase 1**. Full spec (logic, call-rate caveat, async read scheme, `ERROR_ENCODER_DRIFT` handling) lives in the Phase 2 (Deferred) section at the end of this document, which will be extracted to a separate `docs/` file on implementation.

#### `MotorController_loadConfig` / `MotorController_storeConfig` (motor_controller.c:216, 287)

Add round-trip for the new persistent fields:
- **`vernier_phase_offset`** (float) — the magnet fingerprint from calibration. Guard with `isnan` on load (like existing fields at motor_controller.c:231+); if NaN/unset, the vernier must refuse to resolve and stay in `MODE_DISABLED` (uncalibrated). This single NaN guard is the calibration-validity gate.
- **`vernier_base_sector`** (uint8) — sector renumbering origin. **`isnan` is meaningless on a uint8** (review M-3) — do NOT add an independent guard; treat it as valid whenever `vernier_phase_offset` passed its NaN check (they're written together by calibration).
- `encoder_secondary.flux_offset`, `encoder_secondary.position_offset`, `encoder_secondary.velocity_filter_alpha` — same pattern as the primary block. **⚠ Do not blindly clone that block (review M-3):** the existing primary block has bugs — it `isnan`-checks `position_offset` *twice* (motor_controller.c:276 and 278) and loads `velocity_filter_alpha` (line 279) with **no** guard. Write the secondary block correctly: one guard per field, including `velocity_filter_alpha`.

(No gear-ratio field — compile-time. `vernier_sector`/`vernier_initialized` are live state, NOT persisted.) An uncalibrated unit (no valid `vernier_phase_offset` in Flash) must require `MODE_VERNIER_CALIBRATION` before it can resolve absolute position — boot resolution fails closed.

**Ordering note (review MN-4):** calibration's `MotorController_storeConfig` (calibration Step 3) must persist before any later `loadConfig`. `position_controller.position_offset` is restored from Flash on every boot/`FUNC_FLASH` load — it is a commissioning-time field, NOT a runtime-zeroable one. The host must not expect to re-zero it during operation.

### File: `Core/Src/app.c`

No changes. `APP_init` calls `MotorController_init`; the new vernier work happens inside that.

### File: `Core/Src/main.c`

No changes. I2C1 is already initialized; both encoders share the existing peripheral configuration.

## Critical files (paths)

- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Inc/foc_math.h` and `Core/Src/foc_math.c` (add `wrapToPi`, `deg2rad`)
- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Src/app.c` (add `MODE_VERNIER_CALIBRATION` to the TIM2 watchdog exemption at app.c:43 — it's a safe non-driving mode)
- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Inc/encoder.h`
- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Src/encoder.c`
- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Inc/motor_controller.h`
- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Src/motor_controller.c`
- `Recoil-Motor-Controller-B-G431B-ESC1/Core/Inc/motor_controller_conf.h`

## Testing plan

Tests are gated: each level must pass before moving to the next. Motor power is only applied starting at Level 3. Levels 0–2 are safe to run with the arm attached and unpowered.

### Level 0 — Build & static verification (no hardware)

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 0.1 | Clean build | `cd Recoil-Motor-Controller-B-G431B-ESC1/Debug && make clean && make -j$(nproc)` | Compiles with zero new warnings; `.elf` and `.map` produced |
| 0.2 | Struct size / Flash fit | Inspect `sizeof(MotorController)` vs `FLASH_PAGE_SIZE` (2 KB at `0x0801F800`) | Must be ≤ 2048. Review computed ~1416 B after adding a second `Encoder` (560 B) + the new scalars — fits with margin. Store/load loop (motor_controller.c:307) always writes exactly 2048 B regardless |
| 0.3 | Parameter offset audit | Verify each new `PARAM_*` (secondary encoder, `PARAM_VERNIER_PHASE_OFFSET`, `PARAM_VERNIER_BASE_SECTOR`) equals the actual `offsetof`; confirm new fields are appended at struct END | SDO (motor_controller.c:700) addresses the right field AND existing offsets are unshifted; mismatch = silent corruption over CAN |

### Level 1 — I2C bus & encoder presence (powered by USB/VDD only, no motor bus voltage)

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 1.1 | Dual enumeration | Boot; log both raw angle reads to USART2 before vernier resolution | Primary responds at `0x36`, secondary at `0x40`; both return plausible 12-bit values |
| 1.2 | Magnet presence | Read STATUS register `0x0B` on each encoder | MD bit (bit 5) set on both; ML/MH (bits 3,4) clear on both — confirms magnet gap on each shaft |
| 1.3 | Bus integrity under load | Run continuous dual reads for 60 s, count NAK/timeout errors | Zero I2C errors; confirms pull-up value and cable length are acceptable with two devices |
| 1.4 | Independent angle response | Hand-rotate motor shaft only; confirm primary angle changes while secondary changes per the 15/16 gear ratio | Both track; ratio of angular rates ≈ 15:16 — confirms gear meshing and correct shaft assignment |

### Level 1.5 — Vernier calibration (one-time; STATIC, NO motor power; `MODE_VERNIER_CALIBRATION`)

Must complete before Level 2 — the resolution tests assume `vernier_phase_offset` and `vernier_base_sector` are calibrated and stored. **No motor bus voltage needed** — calibration is static reads. The operator hand-positions the (free) arm at its known home, then issues the capture command.

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 1.5.1 | Phase-offset fit | Hand-place arm anywhere; enter `MODE_VERNIER_CALIBRATION`; Step 1 takes N static reads | `vernier_phase_offset` converges; circular spread of `frac(x)` small (e.g. <2° equiv); no `ERROR_VERNIER_CALIBRATION_FAILED` |
| 1.5.2 | Phase-offset position-independence | Re-run Step 1 with the arm hand-placed at 3 different positions | `vernier_phase_offset` agrees across runs within ~1° — confirms it's a position-independent magnet fingerprint (static reads, no motion) |
| 1.5.3 | Base-sector + zero | Hand-place arm at known home, run Step 2 | `vernier_base_sector` recorded; `position_controller.position_offset` set so home reads its known angle; values persisted to Flash |
| 1.5.4 | Non-driving confirmation | Watch phase currents/PWM during `MODE_VERNIER_CALIBRATION` | PWM stays disabled, motor never energized — confirms it's a safe non-driving mode |

### Level 2 — Vernier resolution accuracy (post-calibration; unpowered, hand-positioned)

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 2.1 | Sector ID at known positions | Hand-set joint to ~0°, 90°, 180°, 270°; read `vernier_sector` and `psi_error` at each | Distinct sectors reported; `psi_error` < 3° at every position (margin to the 11.25° half-sector boundary) |
| 2.2 | Boot repeatability | Power-cycle 10× at a fixed joint position | Same `vernier_sector` every time; computed absolute position varies < ±0.1° |
| 2.3 | Boot across full range | Power-cycle at 8 positions spanning the joint's full <360° travel | Correct sector at every position; no boundary mis-ID |
| 2.4 | Inconsistency detection | Deliberately misalign secondary magnet, then boot | `ERROR_VERNIER_INCONSISTENT` set; controller stays in `MODE_DISABLED` |

### Level 3 — Closed-loop functional (motor bus voltage applied, arm on a safe jig)

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 3.1 | Absolute position at boot | Power-cycle with arm at a random angle; read `PARAM_POSITION_CONTROLLER_POSITION_MEASURED` via CAN SDO; compare to a digital angle-gauge reference | Match within ±2 arcmin (encoder limit) or ±gearbox-kinematic-error, whichever dominates |
| 3.2 | Position trajectory | Command a slow sweep through full joint range in `MODE_POSITION` | Smooth tracking; **no position jumps at sector boundaries** (every 24° of arm motion) |
| 3.2b | Supercycle-wrap placement | Sweep full range while logging raw position; confirm the q=15→0 wrap is NOT inside travel | No 384°-magnitude discontinuity anywhere in the operating range — verifies `vernier_base_sector` placed the wrap outside (calibration Step 2 worked) |
| 3.3 | Velocity mode | Command constant velocity in `MODE_VELOCITY` | Steady arm-frame velocity; no encoder-induced ripple at sector crossings |
| 3.4 | FOC commutation quality | Run at moderate speed; observe phase currents on scope | Clean sinusoidal currents — confirms primary encoder commutation path unaffected by the changes |

### Level 4 — Fault detection & recovery (Phase 1: boot-time only)

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 4.1 | Boot inconsistency | Misalign secondary magnet or disconnect secondary cable, then boot | `ERROR_VERNIER_INCONSISTENT` set; controller refuses to leave `MODE_DISABLED` |
| 4.2 | Recovery after fix | Restore magnet/cable, power-cycle | Clean boot; vernier resolves; reaches `MODE_IDLE` |
| 4.3 (DEFERRED) | Runtime drift trip | *Future work* — only when the deferred sanity check is implemented | `ERROR_ENCODER_DRIFT` + `MODE_DAMPING` |
| 4.4 (DEFERRED) | Mid-run secondary disconnect | *Future work* — deferred sanity check | trips without disrupting primary FOC |

### Level 5 — Performance & regression

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 5.1 | FOC loop timing | Measure commutation loop duration (existing baseline ~20.6 µs / 100 µs per motor_controller.c:325) in normal operation | **Unchanged from baseline** — Phase 1 adds NO per-cycle secondary read (the secondary is read only at boot/calibration). Never exceeds 100 µs |
| 5.2 | Watchdog integrity | Confirm TIM2 safety watchdog still fires correctly in normal operation | Watchdog behavior unchanged (motor_controller.c:40 path) — no added runtime I2C traffic in Phase 1 |
| 5.3 | Single-encoder regression | Build with secondary disabled (compile flag) and confirm original behavior is byte-for-byte preserved | No behavioral change when secondary is off — protects existing deployments |

### Level 6 — Persistence

| # | Test | Procedure | Pass criteria |
|---|---|---|---|
| 6.1 | Flash round-trip | Save config via CAN `FUNC_FLASH` (data[0]=1), power-cycle, reload | Secondary encoder calibration and offsets persist (load path motor_controller.c:216). Also confirm `position_controller.gear_ratio == 15.0` in the loaded config (review m3) |
| 6.2 | Fresh-Flash safety | Erase Flash, boot with `LOAD_CONFIG_FROM_FLASH` defaults | `isnan` sentinel checks (motor_controller.c:231+) handle uninitialized secondary fields gracefully — no boot hang |

### Merge gate

Merge `feature/secondary-as5600l-vernier` → `main` only after **Levels 0–6 all pass**. Level 5.3 (single-encoder regression) is the hard gate protecting any existing single-encoder deployments.

## Open decisions to make during implementation

1. **What to do if vernier resolution fails at boot**: current plan flags `ERROR_VERNIER_INCONSISTENT` and stays in `MODE_DISABLED`. Alternative: fall back to primary-only and accept multi-turn ambiguity. The strict default is safer for a robotic arm — going operational with wrong absolute position could drive into a hardstop. Recommend keeping the strict default.
2. **OTP burn for secondary's I2C address**: skip initially since default `0x40` works. If you ever need multiple ESCs sharing one CAN bus where each has its own pair of encoders, you may want to OTP-burn unique addresses to avoid accidental swaps; defer this until needed.
3. **Calibration sequence updates**: `MotorController_runCalibrationSequence` (motor_controller.c:432) currently calibrates only the primary's `flux_offset_table`. Strictly the secondary doesn't need flux calibration (it's not used for FOC commutation), so leave it uncalibrated. Document that the secondary's `flux_offset` should remain 0.

## Estimated effort

**Phase 1 (dedicated vernier calibration + boot resolution — what we build now):**

| Component | LoC (approx) | Effort |
|---|---|---|
| `foc_math.c/h` add `wrapToPi` + `deg2rad` | ~8 | 15 min |
| `encoder.h/c` parameterize I2C address | ~10 | 30 min |
| `motor_controller.h` struct additions (appended at end) | ~12 | 20 min |
| `motor_controller_conf.h` `MODE_VERNIER_CALIBRATION`, errors, params | ~30 | 30 min |
| `MotorController_resolveAbsolutePosition` (blocking + TIM1 mask + calibrated constants) | ~50 | 1.5 hr |
| `MotorController_runVernierCalibration` (STATIC, 2-step read-and-compute) | ~50 | 1.5 hr |
| Wire `MODE_VERNIER_CALIBRATION` as a non-driving mode (setMode case like IDLE + updateService dispatch + watchdog exemption) | ~8 | 30 min |
| Flash load/save updates (incl. `vernier_phase_offset`, `vernier_base_sector`) | ~25 | 30 min |
| Fail-closed guard on init `setMode(MODE_IDLE)` (motor_controller.c:117) | ~5 | 15 min |
| `MotorController_update` | **0 (untouched)** | — |
| **Phase 1 total** | **~205 lines** | **~5.5 hr code + ~1 day bench test** |

The FOC loop is genuinely untouched — no async-I2C, no FOC-timing risk, no motor-spin wiring. Static calibration makes `MODE_VERNIER_CALIBRATION` a safe non-driving mode, independent of electrical flux calibration. Assumes the AS5600L (default `0x40`) is installed on the 16T gear shaft.

**Deferred (future, only if field experience demands live slip detection):** runtime sanity check with async single-in-flight read + `ERROR_ENCODER_DRIFT` handling + I2C callback routing + FOC-timing re-test — est. ~1 day.

