/*
 * encoder.c
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#include "encoder.h"


HAL_StatusTypeDef Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c, uint16_t i2c_address, uint8_t init_bus) {
  encoder->hi2c = hi2c;
  encoder->i2c_address = i2c_address << 1;  // store already-shifted 7-bit address

  encoder->cpr = ENCODER_DIRECTION * (1 << ENCODER_PRECISION_BITS);  // 12 bit precision

  encoder->position_offset = 0.f;

  // defaults to be 2000 Hz cutoff, out of 10 kHz loop
  encoder->velocity_filter_alpha = 0.7153904566639707f;

  encoder->position_raw = 0;
  encoder->n_rotations = 0;
  encoder->last_start_status = HAL_OK;

  encoder->position = 0.f;
  encoder->velocity = 0.f;

  Encoder_resetFluxOffset(encoder);

  // Bounded probe so a missing/dead device fails gracefully instead of hanging boot.
  // init_bus: only the first (primary) encoder initializes the shared I2C peripheral.
  HAL_StatusTypeDef status = HAL_ERROR;
  for (uint8_t attempt = 0; attempt < 10 && status != HAL_OK; attempt += 1) {
    if (init_bus) {
      HAL_I2C_Init(encoder->hi2c);
    }
    // wait for I2C device to power up
    HAL_Delay(100);

    status = HAL_I2C_Mem_Read(encoder->hi2c, encoder->i2c_address, AS5600_ANGLE_ADDR, I2C_MEMADD_SIZE_8BIT, encoder->i2c_buffer, 2, 100);
  }

  return status;
}

void Encoder_resetFluxOffset(Encoder *encoder) {
  encoder->n_rotations = 0;
  encoder->flux_offset = 0.f;
  memset((uint8_t *)encoder->flux_offset_table, 0, ENCODER_LUT_ENTRIES*sizeof(float));
}

HAL_StatusTypeDef Encoder_update(Encoder *encoder) {
  // commutation frequency (10 kHz) should be slower than I2C transfer speed (~12.86 kHz)

  // Read the raw reading from the I2C sensor, the range should be [0, cpr-1].
  // safety check to handle encoder data frame mismatch error
  uint16_t raw_reading = (((uint16_t)encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1];
  if (raw_reading >= abs(encoder->cpr)) {
    // Corrupted/out-of-range frame. Re-arm the read so i2c_buffer REFRESHES next cycle instead of
    // freezing on this garbage (a frozen buffer would wedge the stream until reboot). The caller
    // counts the bad frame and only faults after N consecutive. Position is left unchanged (the
    // caller keeps using the last good value for this one cycle).
    encoder->last_start_status = HAL_I2C_Master_Receive_IT(encoder->hi2c, encoder->i2c_address, encoder->i2c_buffer, 2);
    return 0x04;
  }

  // TODO: implement encoder lut-table Linearization

  // I2C takes ~77.75 us (12.86 kHz) to finish one transaction. Capture the kickoff status so the
  // caller can count hung-bus events (HAL_BUSY/HAL_ERROR = next read never started -> stale buffer).
  encoder->last_start_status = HAL_I2C_Master_Receive_IT(encoder->hi2c, encoder->i2c_address, encoder->i2c_buffer, 2);


  // Calculate the change in reading
  int16_t reading_delta = encoder->position_raw - raw_reading;

  // Handle multi-rotation crossing.
  if (abs(reading_delta) >= abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = raw_reading;

  // Convert the raw position to position in radians (rad)
  float position = (((float)raw_reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);

  // Update the delta position
  float delta_position = position - encoder->position;
  encoder->position = position;

  // Update the filtered velocity
  float velocity = delta_position * (float)COMMUTATION_FREQ;
  encoder->velocity += encoder->velocity_filter_alpha * (velocity - encoder->velocity);

  return HAL_OK;
}

// Approximate busy-wait for the bit-banged recovery clock. Exact rate is NOT critical: any
// ~10-400 kHz clocking frees a stuck slave, so a conservative (slow) delay is fine. volatile
// keeps the loop from being optimized away.
static void Encoder_busDelay(void) {
  for (volatile uint32_t i = 0; i < 800; i += 1) {
    __NOP();
  }
}

HAL_StatusTypeDef Encoder_recoverBus(Encoder *encoder) {
  // Free a hung I2C bus: a slave stuck mid-byte holds SDA low and won't release until it is
  // clocked through the rest of its byte. Sequence: de-init the peripheral, bit-bang up to 9 SCL
  // pulses until SDA releases, issue a STOP, then re-init. I2C1 is SDA=PB7, SCL=PB8 on GPIOB.
  // MUST be called from the foreground with the commutation ISR masked and the motor de-energized
  // (caller's responsibility). Encoder n_rotations is preserved across recovery.
  GPIO_InitTypeDef gpio = {0};

  // A primary-encoder IT receive is almost always in flight (the 10 kHz loop issues one every
  // cycle in all modes), and the I2C1_EV ISR (priority 1) preempts this foreground code. Disable
  // it BEFORE tearing the peripheral down so its completion handler can't race HAL_I2C_DeInit on a
  // half-reset handle (review C1). The in-flight transfer is simply abandoned — we re-init below.
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  HAL_I2C_DeInit(encoder->hi2c);          // disables PE (aborts xfer at peripheral) + MspDeInit
  HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn); // drop any stale event from the abandoned transfer
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // SCL (PB8) open-drain output, SDA (PB7) input; pull-ups keep the bus idling high.
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL idle high
  Encoder_busDelay();

  // Up to 9 clocks; stop early once the slave releases SDA (reads high).
  for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; i += 1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // SCL low
    Encoder_busDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);    // SCL high
    Encoder_busDelay();
  }
  uint8_t recovered = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);

  // Manual STOP: SDA low->high while SCL high, to reset every slave's state machine.
  gpio.Pin = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); Encoder_busDelay();  // SDA low
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   Encoder_busDelay();  // SCL high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   Encoder_busDelay();  // SDA high = STOP

  // Restore the I2C peripheral + AF pins (MspInit reconfigures PB7/PB8 to AF4_I2C1 and re-enables
  // the I2C1_EV IRQ). (void)recovered: SDA-release is informational; the verification read below
  // is the real success criterion.
  (void)recovered;
  if (HAL_I2C_Init(encoder->hi2c) != HAL_OK) {
    // Re-enable the IRQ even on Init failure, so a later recovery attempt can still complete an
    // async read instead of being permanently wedged with the EV interrupt masked (review MINOR-1).
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    return HAL_ERROR;
  }

  // Verify the device actually responds AND prime i2c_buffer with a fresh, in-range reading.
  // This both confirms recovery (vs. just SDA being electrically free) and prevents the next
  // async Encoder_update from parsing a stale/out-of-range buffer and re-asserting the fault
  // (review M2). Resync position_raw to the primed value so that update sees delta=0 (no spurious
  // multi-turn crossing); n_rotations is preserved (assumes no full-rev motion while hung).
  if (HAL_I2C_Mem_Read(encoder->hi2c, encoder->i2c_address, AS5600_ANGLE_ADDR,
                       I2C_MEMADD_SIZE_8BIT, encoder->i2c_buffer, 2, 10) != HAL_OK) {
    return HAL_ERROR;
  }
  uint16_t raw_reading = (((uint16_t)encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1];
  if (raw_reading >= abs(encoder->cpr)) {
    return HAL_ERROR;
  }
  encoder->position_raw = raw_reading;
  return HAL_OK;
}

HAL_StatusTypeDef Encoder_updateBlocking(Encoder *encoder) {
  // Synchronous (blocking) single read + position/n_rotations update. Unlike Encoder_update
  // (which streams via interrupt and is for the 10 kHz loop), this owns the bus for one
  // transaction and is meant for foreground/low-rate use (boot, calibration, debug telemetry)
  // where the caller has masked the commutation ISR. Does NOT touch velocity (call rate varies).
  uint8_t buf[2];
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(encoder->hi2c, encoder->i2c_address,
                                              AS5600_ANGLE_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 2, 10);
  if (status != HAL_OK) {
    return status;
  }
  uint16_t raw_reading = (((uint16_t)buf[0]) << 8) | buf[1];
  if (raw_reading >= abs(encoder->cpr)) {
    return HAL_ERROR;
  }

  int16_t reading_delta = encoder->position_raw - raw_reading;
  if (abs(reading_delta) >= abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = raw_reading;
  encoder->position = (((float)raw_reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);

  return HAL_OK;
}
