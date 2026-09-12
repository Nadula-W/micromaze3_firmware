#pragma once
#include <Arduino.h>
#include "Config.h"

namespace MM3 {

enum class Heading : uint8_t { North = 0, East = 1, South = 2, West = 3 };

enum class RunMode : uint8_t {
  Calibration = 0b000,
  Fast = 0b001,
  Slow = 0b010,
  FastReturnSameSpeed = 0b011,
  Debug = 0b100,
  FastReturnSlow = 0b101,
  SlowReturn = 0b110,
  Exploration = 0b111
};

enum WallBit : uint8_t {
  WALL_N = 1u << 0,
  WALL_E = 1u << 1,
  WALL_S = 1u << 2,
  WALL_W = 1u << 3
};

struct SensorSnapshot {
  uint16_t mm[4] = {8190, 8190, 8190, 8190};
  bool valid[4] = {false, false, false, false};
  uint32_t stampMs = 0;
  // Diagnostic metadata only; navigation continues to use mm/valid above.
  int8_t readyApi[4] = {0, 0, 0, 0};
  int8_t readApi[4] = {0, 0, 0, 0};
  uint8_t rangeStatus[4] = {255, 255, 255, 255};
  uint16_t rawMm[4] = {65535, 65535, 65535, 65535};
  uint32_t readStampMs[4] = {0, 0, 0, 0};
};

struct Pose {
  int8_t x = 0;
  int8_t y = 0;
  Heading heading = Heading::North;
};

struct CalibrationData {
  uint32_t magic = MM3::CAL_MAGIC;
  uint16_t version = MM3::CAL_VERSION;

  // Encoder calibration. 0 means "not calibrated" and distance moves are blocked.
  float ticksPerMmLeft = 0.0f;
  float ticksPerMmRight = 0.0f;

  // MPU6050 Z gyro bias in degrees/second.
  float gyroBiasZDps = 0.0f;

  // Wall/no-wall thresholds for the four ToF sensors by XSHUT index.
  // These defaults are only bring-up values; guided calibration replaces them.
  uint16_t wallThresholdMm[4] = {120, 120, 120, 120};

  // Desired side clearances when centered in a cell. 0 disables single-wall
  // centering until calibrated.
  uint16_t sideTargetLeftMm = 0;
  uint16_t sideTargetRightMm = 0;

  // Single drive-steering PID. The existing terminal command `pid a b c`
  // now maps directly to Kp, Ki, Kd used by cell/DFS straight driving.
  // The three floats remain in the same place in this struct, so all other
  // saved calibration fields keep the same binary layout as v32.
  float pidKp = 1.8f;
  float pidKi = 0.0f;
  float pidKd = 0.03f;

  // Appended so existing calibration can load with default front gains.
  float frontKp = FRONT_ALIGN_KP;
  float frontKi = FRONT_ALIGN_KI;
  float frontKd = FRONT_ALIGN_KD;
  float squareKp = FRONT_SQUARE_KP;
  float squareKi = FRONT_SQUARE_KI;
  float squareKd = FRONT_SQUARE_KD;
};

struct StoredMaze {
  uint8_t walls[MM3::MAZE_N * MM3::MAZE_N] = {0};
  uint8_t known[MM3::MAZE_N * MM3::MAZE_N] = {0};
  uint16_t pathLen = 0;
  uint8_t path[MM3::MAX_PATH] = {0}; // absolute Heading values, one per cell move
  uint8_t goalX = 0;
  uint8_t goalY = 0;
};

inline Heading turnLeft(Heading h) {
  return static_cast<Heading>((static_cast<uint8_t>(h) + 3) & 0x03);
}
inline Heading turnRight(Heading h) {
  return static_cast<Heading>((static_cast<uint8_t>(h) + 1) & 0x03);
}
inline Heading opposite(Heading h) {
  return static_cast<Heading>((static_cast<uint8_t>(h) + 2) & 0x03);
}
inline uint8_t headingWallBit(Heading h) {
  return 1u << static_cast<uint8_t>(h);
}

} // namespace MM3
