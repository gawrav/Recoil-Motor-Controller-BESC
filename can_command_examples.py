#!/usr/bin/env python3
"""
Recoil Motor Controller CAN Command Examples
=============================================

This script demonstrates how to send various CAN commands to the Recoil motor controller
running on the B-G431B-ESC1 board.

CAN ID Format:
--------------
The CAN ID is constructed as: (FUNC_ID << 7) | DEVICE_ID
- FUNC_ID: 4-bit function identifier (see FrameFunction enum)
- DEVICE_ID: 7-bit device identifier (1-127, typically 20 for your motor)

Usage:
------
    python3 can_command_examples.py -c can0 -i 20

Requirements:
-------------
    pip install python-can

Author: Claude AI Assistant
Date: 2025-12-08
"""

import can
import struct
import time
import argparse

# ======== CAN Frame Functions (from motor_controller_conf.h) ========
FUNC_NMT             = 0b0000  # Network Management (mode control)
FUNC_SYNC_EMCY       = 0b0001  # Sync/Emergency
FUNC_TIME            = 0b0010  # Time sync
FUNC_TRANSMIT_PDO_1  = 0b0011  # PDO1 motor -> host
FUNC_RECEIVE_PDO_1   = 0b0100  # PDO1 host -> motor (echo test)
FUNC_TRANSMIT_PDO_2  = 0b0101  # PDO2 motor -> host
FUNC_RECEIVE_PDO_2   = 0b0110  # PDO2 host -> motor (position + velocity)
FUNC_TRANSMIT_PDO_3  = 0b0111  # PDO3 motor -> host
FUNC_RECEIVE_PDO_3   = 0b1000  # PDO3 host -> motor (position + torque)
FUNC_TRANSMIT_PDO_4  = 0b1001  # PDO4 motor -> host (fast frame)
FUNC_RECEIVE_PDO_4   = 0b1010  # PDO4 host -> motor
FUNC_TRANSMIT_SDO    = 0b1011  # SDO motor -> host
FUNC_RECEIVE_SDO     = 0b1100  # SDO host -> motor (parameter read/write)
FUNC_FLASH           = 0b1101  # Flash save/load commands
FUNC_HEARTBEAT       = 0b1110  # Heartbeat/watchdog reset

# ======== Motor Modes (from motor_controller_conf.h) ========
MODE_DISABLED             = 0x00
MODE_IDLE                 = 0x01
MODE_DAMPING              = 0x02  # Passive braking
MODE_CALIBRATION          = 0x05
MODE_CURRENT              = 0x10  # Direct current control
MODE_TORQUE               = 0x11  # Torque control
MODE_VELOCITY             = 0x12  # Velocity control
MODE_POSITION             = 0x13  # Position control
MODE_VABC_OVERRIDE        = 0x20  # Direct voltage control (ABC frame)
MODE_VALPHABETA_OVERRIDE  = 0x21  # Direct voltage control (alpha-beta)
MODE_VQD_OVERRIDE         = 0x22  # Direct voltage control (QD frame)
MODE_DEBUG                = 0x80

# ======== Parameter IDs (from motor_controller_conf.h) ========
PARAM_DEVICE_ID                       = 0x000
PARAM_FIRMWARE_VERSION                = 0x004
PARAM_WATCHDOG_TIMEOUT                = 0x008
PARAM_FAST_FRAME_FREQUENCY            = 0x00C
PARAM_MODE                            = 0x010
PARAM_ERROR                           = 0x014
PARAM_POSITION_CONTROLLER_GEAR_RATIO  = 0x01C
PARAM_POSITION_CONTROLLER_POSITION_KP = 0x020
PARAM_POSITION_CONTROLLER_VELOCITY_KP = 0x028
PARAM_POSITION_CONTROLLER_TORQUE_LIMIT = 0x030
PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT = 0x034
PARAM_POSITION_MEASURED               = 0x060
PARAM_VELOCITY_MEASURED               = 0x054
PARAM_TORQUE_MEASURED                 = 0x048
PARAM_BUS_VOLTAGE_MEASURED            = 0x100
PARAM_ENCODER_POSITION                = 0x134
PARAM_ENCODER_VELOCITY                = 0x138

# ======== SDO Command Specifiers ========
SDO_WRITE = 1  # Download (write to controller)
SDO_READ  = 2  # Upload (read from controller)


class RecoilMotorController:
    """Python interface for Recoil Motor Controller over CAN"""

    def __init__(self, can_interface, device_id):
        """
        Initialize motor controller interface

        Args:
            can_interface: CAN interface name (e.g., 'can0')
            device_id: Motor controller CAN ID (1-127)
        """
        self.bus = can.Bus(interface='socketcan', channel=can_interface, bitrate=1000000)
        self.device_id = device_id

    def _make_can_id(self, func_id):
        """Construct CAN ID from function ID and device ID"""
        return (func_id << 7) | self.device_id

    def send_heartbeat(self):
        """Send heartbeat to reset watchdog timer"""
        can_id = self._make_can_id(FUNC_HEARTBEAT)
        msg = can.Message(arbitration_id=can_id, data=[], is_extended_id=False)
        self.bus.send(msg)
        print(f"✓ Sent heartbeat to motor ID {self.device_id}")

    def set_mode(self, mode):
        """
        Set motor operating mode using NMT command

        Args:
            mode: Mode value (see MODE_* constants)
        """
        can_id = self._make_can_id(FUNC_NMT)
        data = struct.pack('BB', mode, self.device_id)
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)

        mode_names = {
            MODE_DISABLED: "DISABLED",
            MODE_IDLE: "IDLE",
            MODE_DAMPING: "DAMPING",
            MODE_CALIBRATION: "CALIBRATION",
            MODE_CURRENT: "CURRENT",
            MODE_TORQUE: "TORQUE",
            MODE_VELOCITY: "VELOCITY",
            MODE_POSITION: "POSITION"
        }
        print(f"✓ Set mode to {mode_names.get(mode, f'0x{mode:02X}')}")

    def write_parameter(self, param_id, value):
        """
        Write parameter using SDO

        Args:
            param_id: Parameter ID (see PARAM_* constants)
            value: 32-bit float or int value
        """
        can_id = self._make_can_id(FUNC_RECEIVE_SDO)

        # SDO command byte: bits 5-7 = command specifier (1 = write)
        command = SDO_WRITE << 5

        # Pack: command byte, param_id (2 bytes), value (4 bytes)
        if isinstance(value, float):
            data = struct.pack('<BHf', command, param_id, value)
        else:
            data = struct.pack('<BHI', command, param_id, value)

        msg = can.Message(arbitration_id=can_id, data=data[:8], is_extended_id=False)
        self.bus.send(msg)
        print(f"✓ Wrote parameter 0x{param_id:03X} = {value}")

    def read_parameter(self, param_id):
        """
        Read parameter using SDO

        Args:
            param_id: Parameter ID (see PARAM_* constants)

        Returns:
            Parameter value as 32-bit int (you may need to reinterpret as float)
        """
        can_id = self._make_can_id(FUNC_RECEIVE_SDO)

        # SDO command byte: bits 5-7 = command specifier (2 = read)
        command = SDO_READ << 5

        # Pack: command byte, param_id (2 bytes), padding
        data = struct.pack('<BHxxxxx', command, param_id)

        msg = can.Message(arbitration_id=can_id, data=data[:8], is_extended_id=False)
        self.bus.send(msg)

        # Wait for response
        response = self.bus.recv(timeout=1.0)
        if response and response.arbitration_id == self._make_can_id(FUNC_TRANSMIT_SDO):
            value = struct.unpack('<I', response.data[:4])[0]
            print(f"✓ Read parameter 0x{param_id:03X} = 0x{value:08X} ({value})")
            return value
        else:
            print(f"✗ No response for parameter 0x{param_id:03X}")
            return None

    def set_position_velocity(self, position, velocity):
        """
        Send position and velocity targets using PDO2

        Args:
            position: Target position (radians)
            velocity: Target velocity (rad/s)
        """
        can_id = self._make_can_id(FUNC_RECEIVE_PDO_2)
        data = struct.pack('<ff', position, velocity)
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)

        # Wait for response
        response = self.bus.recv(timeout=1.0)
        if response and response.arbitration_id == self._make_can_id(FUNC_TRANSMIT_PDO_2):
            pos_meas, vel_meas = struct.unpack('<ff', response.data)
            print(f"✓ Sent: pos={position:.3f} rad, vel={velocity:.3f} rad/s")
            print(f"  Got:  pos={pos_meas:.3f} rad, vel={vel_meas:.3f} rad/s")
            return pos_meas, vel_meas
        else:
            print(f"✗ No response to PDO2 command")
            return None, None

    def set_position_torque(self, position, torque):
        """
        Send position and torque targets using PDO3

        Args:
            position: Target position (radians)
            torque: Target torque (Nm)
        """
        can_id = self._make_can_id(FUNC_RECEIVE_PDO_3)
        data = struct.pack('<ff', position, torque)
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)

        # Wait for response
        response = self.bus.recv(timeout=1.0)
        if response and response.arbitration_id == self._make_can_id(FUNC_TRANSMIT_PDO_3):
            pos_meas, torque_meas = struct.unpack('<ff', response.data)
            print(f"✓ Sent: pos={position:.3f} rad, torque={torque:.3f} Nm")
            print(f"  Got:  pos={pos_meas:.3f} rad, torque={torque_meas:.3f} Nm")
            return pos_meas, torque_meas
        else:
            print(f"✗ No response to PDO3 command")
            return None, None

    def save_config_to_flash(self):
        """Save current configuration to flash memory"""
        can_id = self._make_can_id(FUNC_FLASH)
        data = struct.pack('B', 1)  # 1 = save
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        print("✓ Saved configuration to flash")

    def load_config_from_flash(self):
        """Load configuration from flash memory"""
        can_id = self._make_can_id(FUNC_FLASH)
        data = struct.pack('B', 2)  # 2 = load
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        print("✓ Loaded configuration from flash")

    def get_status(self):
        """Read common status parameters"""
        print("\n=== Motor Controller Status ===")

        # Read firmware version
        fw = self.read_parameter(PARAM_FIRMWARE_VERSION)
        if fw:
            print(f"  Firmware: 0x{fw:08X} (date format: YYYYmmdd)")

        # Read mode
        mode = self.read_parameter(PARAM_MODE)
        mode_names = {
            MODE_DISABLED: "DISABLED",
            MODE_IDLE: "IDLE",
            MODE_DAMPING: "DAMPING",
            MODE_POSITION: "POSITION",
            MODE_VELOCITY: "VELOCITY",
            MODE_TORQUE: "TORQUE"
        }
        if mode is not None:
            print(f"  Mode: {mode_names.get(mode, f'0x{mode:02X}')}")

        # Read error
        error = self.read_parameter(PARAM_ERROR)
        if error is not None:
            if error == 0:
                print(f"  Error: None (0x{error:04X})")
            else:
                print(f"  Error: 0x{error:04X}")

        # Read bus voltage
        bus_v_raw = self.read_parameter(PARAM_BUS_VOLTAGE_MEASURED)
        if bus_v_raw:
            bus_v = struct.unpack('<f', struct.pack('<I', bus_v_raw))[0]
            print(f"  Bus Voltage: {bus_v:.2f} V")

        print()


# ======== Example Usage ========

def main():
    parser = argparse.ArgumentParser(description='Recoil Motor Controller CAN Examples')
    parser.add_argument('-c', '--can', default='can0', help='CAN interface (default: can0)')
    parser.add_argument('-i', '--id', type=int, default=20, help='Motor CAN ID (default: 20)')
    args = parser.parse_args()

    print(f"\nRecoil Motor Controller CAN Command Examples")
    print(f"==============================================")
    print(f"CAN Interface: {args.can}")
    print(f"Motor ID: {args.id}\n")

    # Initialize motor controller
    motor = RecoilMotorController(args.can, args.id)

    # Example 1: Send heartbeat
    print("Example 1: Send Heartbeat")
    print("-" * 40)
    motor.send_heartbeat()
    time.sleep(0.5)

    # Example 2: Get status
    print("\nExample 2: Get Status")
    print("-" * 40)
    motor.get_status()
    time.sleep(0.5)

    # Example 3: Set motor to IDLE mode
    print("\nExample 3: Set Mode to IDLE")
    print("-" * 40)
    motor.set_mode(MODE_IDLE)
    time.sleep(0.5)

    # Example 4: Read some parameters
    print("\nExample 4: Read Parameters")
    print("-" * 40)
    pos_raw = motor.read_parameter(PARAM_ENCODER_POSITION)
    if pos_raw:
        position = struct.unpack('<f', struct.pack('<I', pos_raw))[0]
        print(f"  Current position: {position:.3f} rad")

    vel_raw = motor.read_parameter(PARAM_ENCODER_VELOCITY)
    if vel_raw:
        velocity = struct.unpack('<f', struct.pack('<I', vel_raw))[0]
        print(f"  Current velocity: {velocity:.3f} rad/s")
    time.sleep(0.5)

    # Example 5: Write a parameter (set velocity limit)
    print("\nExample 5: Write Parameter (Velocity Limit)")
    print("-" * 40)
    motor.write_parameter(PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT, 10.0)  # 10 rad/s
    time.sleep(0.5)

    # Example 6: Send position/velocity command (requires POSITION or VELOCITY mode)
    print("\nExample 6: Send Position + Velocity Command")
    print("-" * 40)
    print("(Motor must be in POSITION mode for this to work)")
    # Uncomment to test (WARNING: motor will move!)
    # motor.set_mode(MODE_POSITION)
    # time.sleep(0.5)
    # motor.set_position_velocity(0.0, 1.0)  # 0 rad position, 1 rad/s velocity
    print("  Skipped (uncomment code to test)")

    print("\nDone! See code for more examples.\n")


if __name__ == '__main__':
    main()
