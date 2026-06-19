/*
 * motor_controller.h
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_H_
#define INC_MOTOR_CONTROLLER_H_

#include "stm32g4xx_hal.h"

#include "motor_controller_conf.h"
#include "can.h"
#include "current_controller.h"
#include "encoder.h"
#include "motor_profiles.h"
#include "motor.h"
#include "position_controller.h"
#include "powerstage.h"


#define FLASH_CONFIG_ADDRESS    0x0801F800U  // Bank 1, Page 63
#define FLASH_CONFIG_BANK       FLASH_BANK_1
#define FLASH_CONFIG_PAGE       63


/**
 * @brief Last-failure stage recorded by MotorController_resolveAbsolutePosition (diag_last_fail_stage).
 *        Un-collapses the single ERROR_VERNIER_INCONSISTENT bit for diagnosis over CAN.
 */
typedef enum {
  VERNIER_FAIL_OK             = 0,
  VERNIER_FAIL_UNCALIBRATED   = 1,
  VERNIER_FAIL_DRAIN          = 2,
  VERNIER_FAIL_PRIMARY_READ   = 3,
  VERNIER_FAIL_SECONDARY_READ = 4,
  VERNIER_FAIL_PSI_MISMATCH   = 5,
} VernierFailStage;

/**
 * @brief MotorController object.
 */
typedef struct {
  uint32_t       device_id;
  uint32_t       firmware_version;

  uint32_t       watchdog_timeout;
  uint32_t       fast_frame_frequency;

  Mode           mode;
  uint32_t       error;

  PositionController  position_controller;
  CurrentController   current_controller;

  PowerStage          powerstage;
  Motor               motor;
  Encoder             encoder;

  // ===== Secondary AS5600L vernier encoder (appended at END to preserve existing PARAM offsets) =====
  Encoder             encoder_secondary;
  float               vernier_phase_offset;     // calibrated magnet fingerprint (rad), persisted
  uint8_t             vernier_base_sector;      // calibrated sector renumbering origin (0..15), persisted
  uint8_t             vernier_sector;           // live resolved sector (NOT persisted)
  uint8_t             vernier_initialized;      // live boolean (NOT persisted)
  uint8_t             UNUSED_vernier_pad;       // alignment pad
  uint16_t            vernier_sanity_counter;   // deferred runtime sanity check (Phase 2)
  uint16_t            UNUSED_vernier_pad2;      // alignment pad
  uint32_t            vernier_cal_magic;        // == VERNIER_CAL_MAGIC iff genuinely calibrated; persisted

  // ===== Diagnostics block (live telemetry; read-only over SDO) =====
  // Re-derived every boot, never loaded from flash (loadConfig ignores these; init re-zeroes
  // them; the whole-page storeConfig serializes the bytes but they are never read back).
  // Lets the CAN status call separate bus vs magnet vs geometry faults. Offsets pinned via
  // _Static_assert in motor_controller.c. uint8s packed into 32-bit words because an SDO read
  // returns a full word. Counters are monotonic (u32, wrap ~every 5 days at 10 kHz) - the host
  // reads them as deltas/rates, never as absolutes.
  uint8_t   diag_enc_probe_status;       // primary Encoder_init() HAL status (0 = on bus)        @0x57C
  uint8_t   diag_enc2_probe_status;      // secondary Encoder_init() HAL status (0 = on bus)
  uint8_t   diag_enc_status_reg;         // primary STATUS 0x0B (MD/ML/MH); 0xFF = not read
  uint8_t   diag_enc2_status_reg;        // secondary STATUS 0x0B; 0xFF = not read

  uint8_t   diag_enc_agc;                // primary AGC 0x1A; 0xFF = not read                      @0x580
  uint8_t   diag_enc2_agc;               // secondary AGC 0x1A; 0xFF = not read
  uint8_t   diag_last_fail_stage;        // VernierFailStage of the last resolveAbsolutePosition
  uint8_t   diag_pad0;                   // alignment

  uint32_t  diag_enc_ok_count;            // primary Encoder_update HAL_OK count (10 kHz)          @0x584
  uint32_t  diag_enc_frame_error_count;   // primary out-of-range frames (raw >= cpr)
  uint32_t  diag_enc_i2c_start_fail_count;// primary Master_Receive_IT kickoff failures (hung bus)
  uint32_t  diag_enc_i2c_error_count;     // I2C transfer errors (HAL_I2C_ErrorCallback)
  uint32_t  diag_enc_last_i2c_errorcode;  // last hi2c1->ErrorCode (AF/BERR/ARLO/...)

  float     diag_theta_p;                // last resolution primary angle [0,2pi)                  @0x598
  float     diag_theta_s;                // last resolution secondary angle [0,2pi) (SIGN applied)
  float     diag_psi;                    // last resolution sector residual (~ q*2pi/16)
  float     diag_psi_error;              // last resolution residual to nearest sector centre (rad)
  int32_t   diag_q_raw;                  // last resolution resolved sector (0..15)
} MotorController;

/**
 * @brief Get the error status.
 *
 * @param controller Pointer to the MotorController struct.
 * @return The error code.
 */
static inline ErrorCode MotorController_getError(MotorController *controller) {
  return controller->error;
}

/**
 * @brief Clear the error.
 *
 * @param controller Pointer to the MotorController struct.
 */
static inline void MotorController_clearError(MotorController *controller) {
  controller->error = ERROR_NO_ERROR;
}

/**
 * @brief Get the current operating mode of the motor controller.
 *
 * @param controller Pointer to the MotorController struct.
 * @return The current mode.
 */
static inline Mode MotorController_getMode(MotorController *controller) {
  return controller->mode;
}

/**
 * @brief Initialize the MotorController instance.
 *
 * @param controller Pointer to the MotorController struct.
 */
void MotorController_init(MotorController *controller);

/**
 * Reset the Motor Controller to the initial state.
 *
 * This function clears all intermediate states and sets various controller
 * parameters and variables to their initial values. It is used between mode
 * switches to reset the controller state.
 *
 * @param controller Pointer to the MotorController struct.
 */
void MotorController_reset(MotorController *controller);

void MotorController_setMode(MotorController *controller, Mode mode);

void MotorController_setFluxAngle(MotorController *controller, float angle_setpoint, float voltage_setpoint);

HAL_StatusTypeDef MotorController_loadConfig(MotorController *controller);

HAL_StatusTypeDef MotorController_storeConfig(MotorController *controller);

void MotorController_update(MotorController *controller);

void MotorController_updateService(MotorController *controller);

void MotorController_runCalibrationSequence(MotorController *controller);

/**
 * Resolve absolute motor position at boot from the vernier (primary + secondary encoders).
 * Runs with the TIM1 update interrupt masked. Seeds encoder.n_rotations from the stored
 * calibration constants. Fails closed (HAL_ERROR + ERROR_VERNIER_INCONSISTENT) if the sector
 * cross-check fails or the unit is uncalibrated.
 */
HAL_StatusTypeDef MotorController_resolveAbsolutePosition(MotorController *controller);

/**
 * Static (non-driving) vernier calibration. Computes vernier_phase_offset (magnet fingerprint)
 * and vernier_base_sector + position_controller.position_offset (operator-placed home), then
 * persists to Flash. The motor is never energized.
 */
void MotorController_runVernierCalibration(MotorController *controller);

void MotorController_handleCANMessage(MotorController *controller, CAN_Frame *rx_frame);

void MotorController_handleNMT(MotorController *controller, CAN_Frame *rx_frame);

void MotorController_handleSDO(MotorController *controller, CAN_Frame *rx_frame, CAN_Frame *tx_frame);

#endif /* INC_MOTOR_CONTROLLER_H_ */
