/*
 * encoder.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32g4xx_hal.h"
#include "foc_math.h"
#include "motor_controller_conf.h"


#define AS5600_I2C_ADDR             0x36U
#define AS5600L_I2C_ADDR_SECONDARY  0x40U   // AS5600L default address (secondary on shared bus)

#define AS5600_ZMCO_ADDR            0x00U
#define AS5600_ZPOS_ADDR            0x01U
#define AS5600_MPOS_ADDR            0x03U
#define AS5600_MANG_ADDR            0x05U
#define AS5600_CONF_ADDR            0x07U
#define AS5600_RAW_ANGLE_ADDR       0x0CU
#define AS5600_ANGLE_ADDR           0x0EU
#define AS5600_STATUS_ADDR          0x0BU
#define AS5600_AGC_ADDR             0x1AU
#define AS5600_MAGNITUDE_ADDR       0x1BU
#define AS5600_BURN_ADDR            0xFFU


/**
 * @brief Encoder object.
 */
typedef struct {
  I2C_HandleTypeDef *hi2c;

  uint8_t   i2c_buffer[2];
  uint8_t   UNUSED_0[2];

  uint16_t  i2c_address;  // already-shifted 7-bit addr (addr << 1); repurposed from UNUSED_1
  uint8_t   UNUSED_2[2];

  int32_t   cpr;
  float     position_offset;      // in range (-inf, inf)

  float     velocity_filter_alpha;

  uint16_t  position_raw;         // in range [0, cpr-1]
  uint8_t   UNUSED_3[2];
  int32_t   n_rotations;

  float     position;             // in range (-inf, inf), with offset
  float     velocity;

  float     flux_offset;
  float     flux_offset_table[128];
} Encoder;


/**
 * @brief Get the position offset of the encoder.
 *
 * @param encoder Pointer to the Encoder struct.
 * @return The offset value in radians (rad).
 */
static inline float Encoder_getPositionOffset(Encoder *encoder) {
  return encoder->position_offset;
}

/**
 * @brief Set the position offset of the encoder.
 *
 * @param encoder Pointer to the Encoder struct.
 * @param encoder The offset value in radians (rad).
 */
static inline void Encoder_setPositionOffset(Encoder *encoder, float offset) {
  encoder->position_offset = offset;
}

/**
 * @brief Get the measured position of the encoder.
 *
 * This method returns the actual position value of the encoder without offset.
 *
 * @param encoder Pointer to the Encoder struct.
 * @return The current measured position in radians (rad).
 */
static inline float Encoder_getPositionMeasured(Encoder *encoder) {
  return encoder->position;
}

/**
 * @brief Get the position of the encoder.
 *
 * This method returns the position value of the encoder compensated with offset.
 *
 * @param encoder Pointer to the Encoder struct.
 * @return The current position in radians (rad).
 */
static inline float Encoder_getPosition(Encoder *encoder) {
  return encoder->position + encoder->position_offset;
}

/**
 * @brief Get the velocity of the encoder.
 *
 * @param encoder Pointer to the Encoder struct.
 * @return The current velocity in radians per second (rad/s).
 */
static inline float Encoder_getVelocity(Encoder *encoder) {
  return encoder->velocity;
}

/**
 * @brief Initialize the Encoder instance with default values.
 *
 * This function initializes an Encoder instance by setting various parameters and performing initializations
 * required for its operation. It configures the I2C interface and initializes other variables and settings.
 *
 * It initiates a position memory read request on the I2C. The subsequent updates can therefore stream the
 * measured position with only I2C read frames.
 *
 * @param encoder Pointer to the Encoder struct.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure that configures the I2C interface.
 * @param i2c_address 7-bit device address (e.g. AS5600_I2C_ADDR); stored shifted internally.
 * @param init_bus Non-zero to (re)initialize the shared I2C peripheral; pass 0 for additional
 *                 devices on an already-initialized bus (e.g. the secondary encoder).
 * @return Status of the initialization process. HAL_OK if successful, HAL_ERROR if the device
 *         did not respond within the bounded probe (e.g. absent secondary).
 */
HAL_StatusTypeDef Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c, uint16_t i2c_address, uint8_t init_bus);

/**
 * @brief Reset the flux offset and rotation count of the Encoder instance.
 *
 * This function resets the rotation count and flux offset of the provided Encoder object.
 * It sets the rotation count to 0 and clears the flux offset and the flux offset table.
 *
 * @param encoder Pointer to the Encoder struct.
 */
void Encoder_resetFluxOffset(Encoder *encoder);

/**
 * @brief Update encoder readings.
 *
 * @param encoder Pointer to the Encoder struct.
 */
HAL_StatusTypeDef Encoder_update(Encoder *encoder);

/**
 * @brief Synchronous (blocking) single-shot update of position / n_rotations.
 *
 * For foreground/low-rate use (boot, calibration, debug telemetry) where the caller
 * owns the bus (commutation ISR masked). Does not stream via interrupt or update velocity.
 *
 * @param encoder Pointer to the Encoder struct.
 * @return HAL_OK on a valid read, else HAL_ERROR (no device / bad frame).
 */
HAL_StatusTypeDef Encoder_updateBlocking(Encoder *encoder);

/**
 * @brief Recover a hung I2C bus (slave holding SDA low) by bit-banging up to 9 SCL pulses + STOP,
 *        then re-initializing the peripheral. Frees whichever device is stuck on the shared bus.
 *
 * Must be called from the foreground with the commutation ISR masked and the motor de-energized.
 * Encoder accumulator state is preserved.
 *
 * @param encoder Pointer to an Encoder on the bus to recover (uses its hi2c handle).
 * @return HAL_OK if SDA was released and the peripheral re-initialized, else HAL_ERROR.
 */
HAL_StatusTypeDef Encoder_recoverBus(Encoder *encoder);

#endif /* INC_ENCODER_H_ */
