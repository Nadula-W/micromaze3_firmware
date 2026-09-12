#pragma once
#include <Arduino.h>
#include "Hardware.h"
#include "RobotTypes.h"

namespace MM3 {

// v36: side opening evidence collected continuously while entering a cell.
// We do not compare against an OPEN calibration value. A reading is either
// CLOSE (wall) or NOT-CLOSE. Three consecutive fresh NOT-CLOSE samples latch
// an opening for the destination cell.
enum class SideObservation : uint8_t { Unknown = 0, Wall = 1, Open = 2 };

struct MoveSideObservation {
  SideObservation left = SideObservation::Unknown;
  SideObservation right = SideObservation::Unknown;
  uint8_t leftNoCloseStreak = 0;
  uint8_t rightNoCloseStreak = 0;
  bool valid = false;
};

using KillCheckFn = bool (*)();

class MotionController {
public:
  MotionController(MotorSystem &motors, Imu6050 &imu, SensorHub &sensors, CalibrationData &cal)
      : _motors(motors), _imu(imu), _sensors(sensors), _cal(cal) {}

  void setKillCheck(KillCheckFn fn) { _kill = fn; }
  bool calibrationReady() const;

  bool driveDistanceMm(float distanceMm, int basePwm, bool useWallCentering = true,
                       bool collisionGuard = true, bool latticeFrontLock = false);
  bool driveCell(int basePwm, bool useWallCentering = true) {
    return driveDistanceMm(CELL_MM, basePwm, useWallCentering, true, false);
  }
  // Legacy one-cell helper used by diagnostics. Maze navigation itself uses the
  // anchored-segment controller below.
  bool driveLocalizedCell(int basePwm) {
    return driveDistanceMm(CELL_MM, basePwm, true, true, false);
  }
  bool driveKnownOpenCell(int basePwm) {
    return driveDistanceMm(CELL_MM, basePwm, true, true, false);
  }

  // Maze navigation uses an ABSOLUTE straight-segment anchor. After a turn the
  // anchor is reset once. Consecutive straight cells target 192, 384, 576... mm
  // from that same physical anchor, so a small stop/overshoot in one cell is
  // corrected by the next cell instead of accumulating.
  bool driveAnchoredMazeCell(int basePwm, bool resetAnchor);
  void invalidateMazeSegmentAnchor() { _mazeAnchorValid = false; _mazeAnchorCells = 0; }

  // Positive degrees = left turn, negative = right turn. Kept for diagnostics.
  bool turnDegrees(float degrees, int maxPwm = TURN_PWM);

  // Competition navigation uses the encoder-calibrated pivot because it is
  // repeatable on this robot. Positive ticks = left, negative = right.
  bool turnEncoderTicks(int32_t signedTicks, int pwm = ENCODER_TURN_PWM);

  // If a front wall exists, dock to a repeatable distance and square the robot
  // using the two front ToF sensors before a 90/180-degree turn.
  bool alignFrontToWall(uint16_t targetMm = FRONT_TURN_TARGET_MM,
                        int maxPwm = FRONT_ALIGN_MAX_PWM);

  bool turnToHeading(Heading &current, Heading target, int maxPwm = TURN_PWM);

  bool wallLeft() const;
  bool wallFront() const;
  bool wallRight() const;
  uint16_t distanceLeft() const;
  uint16_t distanceFront() const;
  uint16_t distanceRight() const;

  const MoveSideObservation &lastMoveSideObservation() const { return _lastMoveSideObs; }
  void clearLastMoveSideObservation() { _lastMoveSideObs = MoveSideObservation{}; }

  void emergencyStop() { _motors.stop(true); }

private:
  MotorSystem &_motors;
  Imu6050 &_imu;
  SensorHub &_sensors;
  CalibrationData &_cal;
  KillCheckFn _kill = nullptr;

  bool _mazeAnchorValid = false;
  int32_t _mazeAnchorLeftTicks = 0;
  int32_t _mazeAnchorRightTicks = 0;
  uint16_t _mazeAnchorCells = 0;
  MoveSideObservation _lastMoveSideObs;

  bool killed() const { return _kill && _kill(); }
  bool wallByIndex(uint8_t index) const;
  int wallSteeringCorrection(const SensorSnapshot &s) const;
};

} // namespace MM3
