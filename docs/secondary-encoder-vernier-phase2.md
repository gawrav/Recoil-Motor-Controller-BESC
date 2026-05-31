# Phase 2 (Deferred) — runtime sanity check

> **Not implemented in Phase 1.** On implementation this section is extracted to its own file (`docs/secondary-encoder-vernier-phase2.md`); it's kept here only to preserve design intent. Build only if field experience shows we need live slip detection while the motor runs.

## Goal

Continuously verify that the primary's accumulated `n_rotations` hasn't slipped (lost counts at high speed, gear-tooth jump, magnet displacement) by cross-checking against the secondary at ~10 Hz during normal operation.

## Why it can't reuse the Phase-1 approach

Phase 1 reads the secondary with the TIM1 ISR masked (blocking). That's fine at boot/calibration (motor stopped or crawling) but **not while the motor runs at speed** — masking for a ~78 µs blocking read skips a 10 kHz FOC cycle, causing a torque blip (at 30k ERPM, ~36° of electrical-angle staleness). So Phase 2 must read the secondary **without pausing FOC** → async single-in-flight arbitration.

## Async I2C arbitration (the in-flight-device scheme)

Add `volatile uint8_t i2c_inflight_device;` to `MotorController` (0 = none, 1 = primary, 2 = secondary).

- Before any `HAL_I2C_Master_Receive_IT`, set `i2c_inflight_device` to the target.
- In `HAL_I2C_MasterRxCpltCallback` (app.c:65, currently empty), route the completed 2 bytes to the correct encoder's buffer based on `i2c_inflight_device`, then clear it to 0.
- In `MotorController_update`: issue a read only when `i2c_inflight_device == 0`. The primary read becomes conditional (no longer unconditional). Prioritize primary every cycle; let the secondary take a slot only when its sanity schedule is due AND the bus is idle. The FOC loop uses the primary value from the previous cycle on the rare cycle the secondary takes the slot (100 µs stale — negligible).
- This makes buffer→device routing explicit instead of relying on call ordering, which breaks with two devices.

## `MotorController_sanityCheckVernier` (full spec)

**Call-rate caveat (review M5)**: `MotorController_update` runs at 10 kHz and calls `PositionController_update` every cycle. To get 10 Hz, either use divisor 1000 at the 10 kHz site, or gate inside `PositionController_update`'s 2 kHz downsampled block (position_controller.c:46) with divisor 200.

```
if (!controller->vernier_initialized) return;
controller->vernier_sanity_counter++;
if (controller->vernier_sanity_counter < <divisor>) return;
controller->vernier_sanity_counter = 0;

// secondary value comes from the async single-in-flight read (NOT blocking+mask):
alpha_p      = controller->encoder.position;                          // accumulated motor angle (rad)
theta_s_meas = wrapTo2Pi(controller->encoder_secondary.position);     // from async read buffer
theta_s_exp  = wrapTo2Pi((float)VERNIER_SECONDARY_NUM / (float)VERNIER_SECONDARY_DEN * alpha_p);  // 15/16·α_p
error        = wrapToPi(theta_s_meas - theta_s_exp);

if (fabsf(error) > deg2rad(5.0f)) {     // one-rev slip moves θ_s_exp by 22.5°; 5° sits between ~1° noise and 22.5°
    SET_BITS(controller->error, ERROR_ENCODER_DRIFT);
    MotorController_setMode(controller, MODE_DAMPING);   // match ERROR_ENCODER_FAULT (motor_controller.c:346)
}
```

Math: a one-motor-rev primary slip moves `α_p` by 2π → moves `θ_s_exp` by `2π·(15/16) = 2π − 2π/16` → a `22.5°` shift in predicted secondary angle. The 5° threshold reliably catches single-rev slips without false-tripping on the ~1° noise floor.

## Phase 2 additions checklist

- `ERROR_ENCODER_DRIFT` error bit (reserve in Phase 1, set only here).
- `i2c_inflight_device` field + callback routing in `HAL_I2C_MasterRxCpltCallback`.
- Conditional primary read in `MotorController_update` (the one Phase-1-untouched function this changes).
- `MotorController_sanityCheckVernier` + its call site.
- Re-run Level 5 FOC-timing tests (the FOC loop changes here, unlike Phase 1).
- Test 4.3 (runtime drift trip) and 4.4 (mid-run secondary disconnect).
