#pragma once
#include <Arduino.h>

// Development-only Wi-Fi terminal switch. PlatformIO sets this per environment.
// Competition builds MUST use MM3_DEV_WIFI=0.
#ifndef MM3_DEV_WIFI
#define MM3_DEV_WIFI 0
#endif

namespace MM3 {

namespace Pins {
constexpr uint8_t MAIN_SDA = 8;
constexpr uint8_t MAIN_SCL = 9;
constexpr uint8_t DIST_SDA = 7;
constexpr uint8_t DIST_SCL = 15;

constexpr uint8_t XSHUT_1 = 4;
constexpr uint8_t XSHUT_2 = 2;
constexpr uint8_t XSHUT_3 = 16;
constexpr uint8_t XSHUT_4 = 6; // rewired on the real robot
constexpr uint8_t XSHUT[4] = {XSHUT_1, XSHUT_2, XSHUT_3, XSHUT_4};

constexpr uint8_t MOTOR_STBY = 42;
constexpr uint8_t MOTOR_A_PWM = 11;
constexpr uint8_t MOTOR_B_PWM = 48;
constexpr uint8_t MOTOR_A_IN1 = 12;
constexpr uint8_t MOTOR_A_IN2 = 13;
constexpr uint8_t MOTOR_B_IN1 = 21;
constexpr uint8_t MOTOR_B_IN2 = 47;

constexpr uint8_t ENC_A_C1 = 41;
constexpr uint8_t ENC_A_C2 = 40;
constexpr uint8_t ENC_B_C1 = 39;
constexpr uint8_t ENC_B_C2 = 38;

constexpr uint8_t EEPROM_WP = 10;      // HIGH = read-only, LOW = read/write
constexpr uint8_t IO_INT = 14;         // PCF8574 interrupt, active LOW
constexpr uint8_t RGB_LED = 46;
}

namespace Addr {
constexpr uint8_t PCF8574 = 0x20; // confirmed by real-board I2C scan
constexpr uint8_t MPU6050 = 0x68;
constexpr uint8_t EEPROM_BLOCK0 = 0x50;
constexpr uint8_t EEPROM_BLOCK1 = 0x51;
constexpr uint8_t VL53_DEFAULT = 0x29;
constexpr uint8_t VL53[4] = {0x30, 0x31, 0x32, 0x33};
}

// IMPORTANT: The uploaded hardware document gives four XSHUT pins but does not
// state which physical sensor is left/front-left/front-right/right. These are
// safe defaults only. Verify with Debug mode -> "tof" and change these four
// indices if your PCB wiring is different.
namespace SensorMap {
constexpr uint8_t LEFT = 1;        // confirmed on real robot
constexpr uint8_t FRONT_LEFT = 0;  // confirmed on real robot
constexpr uint8_t FRONT_RIGHT = 3; // confirmed on real robot
constexpr uint8_t RIGHT = 2;       // confirmed on real robot
}

// Motor A/B is named by the PCB schematic, not by robot side. If the physical
// wiring is opposite, change this one constant instead of rewriting motion code.
constexpr bool LEFT_MOTOR_IS_A = false;
constexpr bool LEFT_MOTOR_INVERT = false;
constexpr bool RIGHT_MOTOR_INVERT = false;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t MAIN_I2C_HZ = 400000;
constexpr uint32_t DIST_I2C_HZ = 400000;
constexpr uint32_t PWM_HZ = 20000;
constexpr uint8_t PWM_BITS = 8;
constexpr int PWM_MAX = 255;

constexpr float CELL_MM = 192.0f; // lattice-point / cell-center pitch; clear corridor width is 180 mm
constexpr uint8_t MAZE_N = 10;
constexpr uint8_t MAZE_GOAL_X = 6;
constexpr uint8_t MAZE_GOAL_Y = 6;
static_assert(MAZE_GOAL_X < MAZE_N && MAZE_GOAL_Y < MAZE_N, "Goal must be inside the maze");
constexpr uint16_t MAX_PATH = 512;

// Starting engineering values. These are NOT competition booklet limits.
// Calibration mode should replace the values that depend on your real robot.
constexpr int SLOW_PWM = 90;
constexpr int FAST_PWM = 155;
constexpr int TURN_PWM = 105;
// Calibrated on the real robot: encoder-only pivot is currently more reliable than MPU yaw.
// Tune these independently with `turn_ticks <ticks>` (left) and
// `turn_ticks -<ticks>` (right). Real drivetrains are asymmetric.
constexpr int32_t TURN_LEFT_90_TICKS = 330;
constexpr int32_t TURN_RIGHT_90_TICKS = 330;
constexpr int ENCODER_TURN_PWM = 68;
constexpr int MIN_MOVE_PWM = 58;
constexpr uint16_t COLLISION_STOP_MM = 24;

// Front-wall docking/alignment. Calibrate FRONT_TURN_TARGET_MM by placing the
// robot at the true centre of a cell, facing a front wall, and reading F with
// the `walls` command. This robot is currently calibrated to 80 mm.
constexpr uint16_t FRONT_TURN_TARGET_MM = 83;
// Continuous front-wall reference. While driving, the controller watches the
// front ToF continuously. If a stable front wall is acquired within this
// distance, encoder distance becomes a coarse reference and the physical
// FRONT_TURN_TARGET_MM becomes the final longitudinal stop reference.
constexpr uint16_t FRONT_DOCK_TRIGGER_MM = 150;
// Brake before the final 80 mm docking point, then let the slow front-align
// controller finish the last few centimetres. This prevents a fast approach
// from physically contacting the wall before the motors can stop.
constexpr uint16_t FRONT_PREALIGN_MM = 92; // travel almost to target; final align mainly squares the nose
constexpr uint16_t FRONT_PREALIGN_MIN_PROGRESS_MM = 45;
constexpr uint16_t FRONT_REFERENCE_CONFIRM_SAMPLES = 2;
constexpr uint16_t FRONT_MAX_EXTRA_TRAVEL_MM = 90;
constexpr uint16_t FRONT_REFERENCE_TOL_MM = 3;

constexpr uint16_t FRONT_ALIGN_TOL_MM = 5;
constexpr uint16_t FRONT_SQUARE_TOL_MM = 5;// stopped alignment: 65-95 mm at 80 mm target
// Grid-phase localization from a visible wall.  At a true cell centre the front
// sensor should read FRONT_TURN_TARGET_MM + N*CELL_MM.  While a cell move is in
// progress we add encoder progress back to the live front reading to estimate
// where the move started, snap that estimate to the nearest lattice centre, then
// aim for the next centre.  This removes accumulated 192-mm odometry error.
constexpr uint16_t FRONT_LOCALIZE_PHASE_TOL_MM = 72;
constexpr uint8_t FRONT_LOCALIZE_CONFIRM_SAMPLES = 2;
constexpr uint16_t FRONT_LOCALIZE_MAX_EXTRA_MM = 110;
constexpr uint16_t FRONT_LOCALIZE_BRAKE_MM = 90;
constexpr int FRONT_ALIGN_MAX_PWM = 72;
constexpr float FRONT_ALIGN_KP = 1.25f;
constexpr float FRONT_ALIGN_KI = 0.20f;
constexpr float FRONT_ALIGN_KD = 0.08f;
constexpr float FRONT_SQUARE_KP = 0.90f;
constexpr float FRONT_SQUARE_KI = 0.15f;
constexpr float FRONT_SQUARE_KD = 0.05f;
constexpr float FRONT_ALIGN_I_LIMIT_PWM = 12.0f;
constexpr float FRONT_ALIGN_D_FILTER_S = 0.08f;
constexpr uint32_t FRONT_ALIGN_TIMEOUT_MS = 5000;
constexpr uint32_t FRONT_ALIGN_CONTROL_MS = 20;
constexpr uint32_t FRONT_ALIGN_STALE_MS = 200;

// Corridor anti-contact / stall recovery. These values are deliberately
// conservative for the competition-deadline build.
constexpr uint16_t SIDE_HARD_MARGIN_MM = 12;
constexpr int SIDE_ESCAPE_BOOST = 9;
constexpr uint32_t DRIVE_STALL_MS = 550;
constexpr uint8_t DRIVE_MAX_RECOVERIES = 2;
constexpr int DRIVE_RECOVERY_PWM = 78;
constexpr uint32_t DRIVE_RECOVERY_REVERSE_MS = 120;
constexpr uint32_t SENSOR_PERIOD_MS = 20;
constexpr uint32_t TOF_STALE_MS = 180;

// Front ToF mounting/measurement offsets in millimetres.
// Put the robot perfectly square to a wall, average 10-20 samples, then make
// corrected FL and FR equal. Example: FL=88, FR=78 -> FL=-5, FR=+5.
constexpr int16_t FRONT_LEFT_OFFSET_MM  = +10;
constexpr int16_t FRONT_RIGHT_OFFSET_MM = -10;

constexpr uint32_t COUNTDOWN_MS = 5000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 35;

constexpr uint32_t CAL_MAGIC = 0x4D4D3343;   // "MM3C"
constexpr uint16_t CAL_VERSION = 3;
constexpr uint32_t MAZE_MAGIC = 0x4D4D334D;  // "MM3M"
constexpr uint16_t MAZE_VERSION = 3; // invalidate saved paths to the previous goal

} // namespace MM3
