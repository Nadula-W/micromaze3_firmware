#include "Motion.h"
#include <math.h>

namespace MM3 {

namespace {
constexpr bool PID_TRACE_ENABLED = true;
constexpr uint32_t PID_TRACE_PERIOD_MS = 250; // keep low-rate so Serial does not disturb control timing

uint16_t correctedFrontMm(const SensorSnapshot &s, uint8_t index, int16_t offset) {
  if (index >= 4 || !s.valid[index]) return 8190;
  int value = (int)s.mm[index] + (int)offset;
  return (uint16_t)constrain(value, 1, 8189);
}

bool tofFresh(const SensorSnapshot &s, uint8_t index, uint32_t now = millis()) {
  return index < 4 && s.valid[index] && s.readStampMs[index] != 0 &&
         now - s.readStampMs[index] <= TOF_STALE_MS;
}

uint32_t pairedFrontStamp(const SensorSnapshot &s) {
  const uint8_t fl = SensorMap::FRONT_LEFT;
  const uint8_t fr = SensorMap::FRONT_RIGHT;
  if (s.readStampMs[fl] == 0 || s.readStampMs[fr] == 0) return 0;
  return min(s.readStampMs[fl], s.readStampMs[fr]);
}
}

bool MotionController::calibrationReady() const {
  return isfinite(_cal.ticksPerMmLeft) && isfinite(_cal.ticksPerMmRight) &&
         _cal.ticksPerMmLeft > 0.01f && _cal.ticksPerMmRight > 0.01f && _imu.present();
}

bool MotionController::wallByIndex(uint8_t index) const {
  SensorSnapshot s = _sensors.snapshot();
  if (index >= 4 || !tofFresh(s, index)) return false;
  return s.mm[index] < _cal.wallThresholdMm[index];
}

bool MotionController::wallLeft() const { return wallByIndex(SensorMap::LEFT); }
bool MotionController::wallRight() const { return wallByIndex(SensorMap::RIGHT); }

bool MotionController::wallFront() const {
  SensorSnapshot s = _sensors.snapshot();
  const uint8_t a = SensorMap::FRONT_LEFT;
  const uint8_t b = SensorMap::FRONT_RIGHT;
  const uint32_t now = millis();
  const bool av = tofFresh(s, a, now);
  const bool bv = tofFresh(s, b, now);
  const uint16_t am = av ? correctedFrontMm(s, a, FRONT_LEFT_OFFSET_MM) : 8190;
  const uint16_t bm = bv ? correctedFrontMm(s, b, FRONT_RIGHT_OFFSET_MM) : 8190;

  // Maze topology: require BOTH front sensors to agree. One sensor by itself can
  // see a post/edge or produce a transient ToF error and create a false dead end.
  bool normalWall = av && bv &&
                    am < _cal.wallThresholdMm[a] &&
                    bm < _cal.wallThresholdMm[b];

  // Collision safety stays conservative: either sensor extremely close stops us.
  bool emergency = (av && am < COLLISION_STOP_MM + 12) ||
                   (bv && bm < COLLISION_STOP_MM + 12);
  return normalWall || emergency;
}

uint16_t MotionController::distanceLeft() const {
  SensorSnapshot s = _sensors.snapshot();
  return tofFresh(s, SensorMap::LEFT) ? s.mm[SensorMap::LEFT] : 8190;
}
uint16_t MotionController::distanceRight() const {
  SensorSnapshot s = _sensors.snapshot();
  return tofFresh(s, SensorMap::RIGHT) ? s.mm[SensorMap::RIGHT] : 8190;
}
uint16_t MotionController::distanceFront() const {
  SensorSnapshot s = _sensors.snapshot();
  const uint32_t now = millis();
  const uint8_t fl = SensorMap::FRONT_LEFT;
  const uint8_t fr = SensorMap::FRONT_RIGHT;
  uint16_t a = tofFresh(s, fl, now) ? correctedFrontMm(s, fl, FRONT_LEFT_OFFSET_MM) : 8190;
  uint16_t b = tofFresh(s, fr, now) ? correctedFrontMm(s, fr, FRONT_RIGHT_OFFSET_MM) : 8190;
  if (a == 8190) return b;
  if (b == 8190) return a;
  return (uint16_t)((a + b) / 2u);
}

float MotionController::wallSteeringErrorMm(const SensorSnapshot &s) const {
  const uint8_t li = SensorMap::LEFT;
  const uint8_t ri = SensorMap::RIGHT;
  bool lwall = s.valid[li] && s.mm[li] < _cal.wallThresholdMm[li];
  bool rwall = s.valid[ri] && s.mm[ri] < _cal.wallThresholdMm[ri];

  // RAW centering error only. There is no separate wall gain anymore.
  // Positive error means steer left (left wheel slower, right wheel faster).
  if (lwall && rwall && _cal.sideTargetLeftMm && _cal.sideTargetRightMm) {
    float lErr = (float)s.mm[li] - _cal.sideTargetLeftMm;
    float rErr = (float)s.mm[ri] - _cal.sideTargetRightMm;
    return (lErr - rErr) * 0.5f;
  }
  // When one wall disappears at an opening, do not suddenly give the remaining
  // wall full control. That step change was a major source of left-right wobble.
  if (lwall && _cal.sideTargetLeftMm) {
    float e = ((float)s.mm[li] - _cal.sideTargetLeftMm) * 0.35f;
    return constrain(e, -8.0f, 8.0f);
  }
  if (rwall && _cal.sideTargetRightMm) {
    float e = -((float)s.mm[ri] - _cal.sideTargetRightMm) * 0.35f;
    return constrain(e, -8.0f, 8.0f);
  }
  return 0.0f;
}

bool MotionController::driveDistanceMm(float distanceMm, int basePwm, bool useWallCentering,
                                           bool collisionGuard, bool latticeFrontLock) {
  if (_cal.ticksPerMmLeft <= 0.01f || _cal.ticksPerMmRight <= 0.01f) {
    Serial.println("MOVE BLOCKED: ticks/mm is not calibrated. Run Calibration mode first.");
    return false;
  }
  if (fabsf(distanceMm) < 0.5f) return true;

  const int direction = distanceMm >= 0 ? +1 : -1;
  const float target = fabsf(distanceMm);
  basePwm = constrain(abs(basePwm), MIN_MOVE_PWM, PWM_MAX);

  int32_t startL = _motors.leftTicks();
  int32_t startR = _motors.rightTicks();
  uint32_t startMs = millis();
  // Extra time is allowed because a real front wall may intentionally become
  // the final longitudinal reference a few centimetres after the encoder target.
  uint32_t timeoutMs = (uint32_t)(target * 35.0f) + 3000u;
  timeoutMs = constrain(timeoutMs, 3000u, 20000u);

  bool success = false;
  float lastProgressMm = 0.0f;
  uint32_t lastProgressMs = millis();
  uint8_t recoveryCount = 0;

  // Front-reference confirmation is based on NEW ToF snapshots, not repeated
  // reads of the same sample in the fast motor-control loop.
  uint32_t lastFrontStamp = 0;
  uint8_t frontNearSamples = 0;

  // Physical grid-phase localization.  This is deliberately independent of the
  // assumption that the robot started exactly at a cell centre.
  bool latticeLocked = false;
  int latticeCandidateIndex = -1;
  uint8_t latticeCandidateSamples = 0;
  float latticeTargetFrontMm = -1.0f;
  uint32_t latticeLastValidMs = 0;
  uint8_t latticeStopSamples = 0;

  // One PID controller only. `pid Kp Ki Kd` controls these terms directly.
  float pidIntegral = 0.0f;
  float pidPrevError = 0.0f;
  bool pidHasPrev = false;
  uint32_t lastPidUs = micros();
  uint32_t lastPidTraceMs = 0;
  if (PID_TRACE_ENABLED) {
    Serial.printf("[PID DRIVE ACTIVE] target=%.1fmm pwm=%d wallCenter=%s Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  target, basePwm, useWallCentering ? "ON" : "OFF",
                  _cal.pidKp, _cal.pidKi, _cal.pidKd);
  }

  while (!killed() && millis() - startMs < timeoutMs) {
    int32_t dL = _motors.leftTicks() - startL;
    int32_t dR = _motors.rightTicks() - startR;
    float leftMm = fabsf((float)dL) / _cal.ticksPerMmLeft;
    float rightMm = fabsf((float)dR) / _cal.ticksPerMmRight;
    float avgMm = 0.5f * (leftMm + rightMm);

    // Progress watchdog / anti-stall recovery.
    if (avgMm > lastProgressMm + 1.5f) {
      lastProgressMm = avgMm;
      lastProgressMs = millis();
    } else if (millis() - lastProgressMs > DRIVE_STALL_MS && avgMm < target - 8.0f) {
      _motors.stop(true);
      if (recoveryCount >= DRIVE_MAX_RECOVERIES) {
        Serial.println("MOVE FAIL: encoder progress stalled after recovery attempts.");
        success = false;
        break;
      }

      ++recoveryCount;
      Serial.print("MOVE RECOVERY #");
      Serial.println(recoveryCount);
      _motors.setWheels(-direction * DRIVE_RECOVERY_PWM,
                        -direction * DRIVE_RECOVERY_PWM);
      delay(DRIVE_RECOVERY_REVERSE_MS);
      _motors.stop(true);
      delay(70);
      lastProgressMm = avgMm;
      lastProgressMs = millis();
      continue;
    }

    // Encoder synchronization error. If side-wall centering is available, its
    // raw error is added below. ONE Kp/Ki/Kd controller handles the result.
    float syncErrMm = leftMm - rightMm;
    float wallErrMm = 0.0f;
    float driveErrorMm = syncErrMm;
    int correction = 0;

    bool frontCandidate = false;
    bool frontReferenceActive = false;
    uint16_t frontAvg = 8190;

    if (direction > 0 && useWallCentering) {
      SensorSnapshot s = _sensors.snapshot();
      wallErrMm = wallSteeringErrorMm(s);
      driveErrorMm += wallErrMm;

      // Strong side-wall escape if the chassis is already too close.
      const uint8_t sli = SensorMap::LEFT;
      const uint8_t sri = SensorMap::RIGHT;
      if (s.valid[sli] && _cal.sideTargetLeftMm > 0 &&
          s.mm[sli] + SIDE_HARD_MARGIN_MM < _cal.sideTargetLeftMm) {
        correction -= SIDE_ESCAPE_BOOST;
      }
      if (s.valid[sri] && _cal.sideTargetRightMm > 0 &&
          s.mm[sri] + SIDE_HARD_MARGIN_MM < _cal.sideTargetRightMm) {
        correction += SIDE_ESCAPE_BOOST;
      }

      // CONTINUOUS front-wall tracking.  Do not stop merely because the encoder
      // says 192 mm if a real front wall is already visible.  In that situation
      // the physical 80 mm wall distance is the stronger longitudinal reference.
      const uint8_t fl = SensorMap::FRONT_LEFT;
      const uint8_t fr = SensorMap::FRONT_RIGHT;
      const uint32_t nowMs = millis();
      const bool flFresh = tofFresh(s, fl, nowMs);
      const bool frFresh = tofFresh(s, fr, nowMs);
      uint16_t f1 = flFresh ? correctedFrontMm(s, fl, FRONT_LEFT_OFFSET_MM) : 8190;
      uint16_t f2 = frFresh ? correctedFrontMm(s, fr, FRONT_RIGHT_OFFSET_MM) : 8190;
      bool bothFrontValid = (f1 < 8190 && f2 < 8190);
      bool anyFrontValid = (f1 < 8190 || f2 < 8190);
      uint16_t frontDiff = bothFrontValid ? (uint16_t)abs((int)f1 - (int)f2) : 999u;
      frontAvg = bothFrontValid ? (uint16_t)((f1 + f2) / 2u) : min(f1, f2);
      // Docking/localization needs a coherent PAIR. One valid front sensor is
      // kept only for emergency collision protection.
      bool frontLooksUsable = bothFrontValid && frontDiff <= 45u;
      const uint32_t frontPairStamp = frontLooksUsable ? pairedFrontStamp(s) : 0;

      // Infer the real grid phase from the front wall whenever line-of-sight to a
      // wall exists.  If we have already moved avgMm, then frontAvg + avgMm is an
      // estimate of the front distance at the beginning of this cell move.  True
      // cell centres lie at 80 + N*192 mm from that wall.  Snap to the nearest
      // such centre and target the NEXT one.  This is the key correction for the
      // "three nominal cells becomes three-and-a-half" failure mode.
      if (latticeFrontLock && frontLooksUsable) {
        latticeLastValidMs = millis();
        float estimatedStartFront = (float)frontAvg + avgMm;
        float nFloat = (estimatedStartFront - (float)FRONT_TURN_TARGET_MM) / CELL_MM;
        int n = (int)lroundf(nFloat);
        float snappedStart = (float)FRONT_TURN_TARGET_MM + (float)n * CELL_MM;
        float phaseErr = fabsf(estimatedStartFront - snappedStart);

        // n>=1 means there is at least one cell centre ahead before the wall.
        // Reject a reading that is more than ~1/3 cell away from any legal phase;
        // that is usually a side-wall/diagonal reflection rather than the corridor
        // end wall.
        bool candidateOkay = n >= 1 && phaseErr <= FRONT_LOCALIZE_PHASE_TOL_MM;
        if (!latticeLocked && candidateOkay) {
          if (n == latticeCandidateIndex) {
            if (frontPairStamp != 0 && frontPairStamp != lastFrontStamp && latticeCandidateSamples < 10)
              ++latticeCandidateSamples;
          } else {
            latticeCandidateIndex = n;
            latticeCandidateSamples = 1;
          }

          if (latticeCandidateSamples >= FRONT_LOCALIZE_CONFIRM_SAMPLES) {
            latticeLocked = true;
            latticeTargetFrontMm = (float)FRONT_TURN_TARGET_MM + (float)(n - 1) * CELL_MM;
            Serial.print("CELL LOCALIZE: estimated start F=");
            Serial.print(estimatedStartFront, 1);
            Serial.print(" -> snapped F=");
            Serial.print(snappedStart, 1);
            Serial.print(" -> next-center F=");
            Serial.println(latticeTargetFrontMm, 1);
          }
        }

        if (latticeLocked) {
          float physicalRemaining = (float)frontAvg - latticeTargetFrontMm;
          if (physicalRemaining <= FRONT_REFERENCE_TOL_MM) {
            if (frontPairStamp != 0 && frontPairStamp != lastFrontStamp) {
              if (latticeStopSamples < 10) ++latticeStopSamples;
            }
            if (latticeStopSamples >= FRONT_REFERENCE_CONFIRM_SAMPLES) {
              Serial.print("CELL CENTER STOP: F=");
              Serial.print(frontAvg);
              Serial.print(" target=");
              Serial.print(latticeTargetFrontMm, 1);
              Serial.print(" encoder=");
              Serial.print(avgMm, 1);
              Serial.println(" mm");
              success = true;
              break;
            }
          } else {
            latticeStopSamples = 0;
          }
        }
      }

      frontCandidate = collisionGuard && frontLooksUsable &&
                       frontAvg <= FRONT_DOCK_TRIGGER_MM &&
                       avgMm >= target * 0.55f;

      if (frontPairStamp != 0 && frontPairStamp != lastFrontStamp) {
        lastFrontStamp = frontPairStamp;
        if (frontCandidate) {
          if (frontNearSamples < 10) ++frontNearSamples;
        } else {
          frontNearSamples = 0;
        }
      }

      frontReferenceActive = frontCandidate &&
                             frontNearSamples >= FRONT_REFERENCE_CONFIRM_SAMPLES;

      // The desired physical cell position is reached.  This can happen slightly
      // before OR after the nominal 192 mm encoder point, which removes accumulated
      // longitudinal error whenever a front wall is available.
      if (frontReferenceActive &&
          frontAvg <= FRONT_TURN_TARGET_MM + FRONT_REFERENCE_TOL_MM) {
        Serial.print("FRONT REFERENCE STOP: F=");
        Serial.print(frontAvg);
        Serial.print(" mm, encoder=");
        Serial.print(avgMm, 1);
        Serial.println(" mm");
        success = true;
        break;
      }

      // Hard emergency protection for a suspiciously early obstacle.
      if (collisionGuard && anyFrontValid && frontAvg < COLLISION_STOP_MM &&
          avgMm < target * 0.50f) {
        Serial.println("MOVE ABORT: emergency front collision guard triggered.");
        success = false;
        break;
      }

      // If the wall reference was acquired but we somehow pass far beyond the
      // normal cell target without reaching 80 mm, stop rather than run away.
      if (frontReferenceActive && avgMm > target + FRONT_MAX_EXTRA_TRAVEL_MM) {
        Serial.println("MOVE FAIL: front reference never reached target distance.");
        success = false;
        break;
      }
    }

    // Time-correct PID. The old implementation omitted dt, so I/D gains changed
    // whenever Wi-Fi/serial/sensor scheduling changed the loop period.
    uint32_t pidNowUs = micros();
    float pidDt = (pidNowUs - lastPidUs) * 1e-6f;
    lastPidUs = pidNowUs;
    pidDt = constrain(pidDt, 0.001f, 0.050f);
    pidIntegral += driveErrorMm * pidDt;
    pidIntegral = constrain(pidIntegral, -40.0f, 40.0f);
    float pidDerivative = pidHasPrev ? (driveErrorMm - pidPrevError) / pidDt : 0.0f;
    float pTerm = _cal.pidKp * driveErrorMm;
    float iTerm = _cal.pidKi * pidIntegral;
    float dTerm = _cal.pidKd * pidDerivative;
    float pidOutput = pTerm + iTerm + dTerm;
    correction += constrain((int)lroundf(pidOutput), -55, 55);
    correction = constrain(correction, -78, 78);
    pidPrevError = driveErrorMm;
    pidHasPrev = true;

    if (PID_TRACE_ENABLED && millis() - lastPidTraceMs >= PID_TRACE_PERIOD_MS) {
      lastPidTraceMs = millis();
      Serial.printf("[PID DRIVE] prog=%.1f err=%.2f enc=%.2f wall=%.2f P=%.2f I=%.2f D=%.2f out=%.2f corr=%d\n",
                    avgMm, driveErrorMm, syncErrMm, wallErrMm,
                    pTerm, iTerm, dTerm, pidOutput, correction);
    }

    const bool encoderDone =
        avgMm >= target && leftMm >= target * 0.94f && rightMm >= target * 0.94f;

    if (encoderDone) {
      // If a physical lattice target is locked and the front sensor is still
      // visible, do NOT stop just because 192 encoder millimetres elapsed.  Keep
      // moving to the inferred next cell centre.  If the wall reading disappears,
      // safely fall back to encoder odometry rather than hanging forever.
      bool latticeStillUsable = latticeLocked && (millis() - latticeLastValidMs <= 180u);
      if (!latticeStillUsable &&
          !(direction > 0 && useWallCentering && collisionGuard && frontCandidate)) {
        success = true;
        break;
      }
    }

    if (latticeLocked && avgMm > target + FRONT_LOCALIZE_MAX_EXTRA_MM) {
      Serial.println("MOVE FAIL: localized cell centre was not reached within safety travel limit.");
      success = false;
      break;
    }

    // FAST adaptive speed profile.  Open corridor = almost full requested PWM.
    // A real front wall causes progressive braking as it approaches 80 mm.
    float remaining = target - avgMm;
    float scale = 1.0f;

    // Short acceleration ramp only; do not spend 30% of every open cell slowing.
    if (avgMm < target * 0.08f) {
      scale = 0.68f + 0.32f * (avgMm / max(1.0f, target * 0.08f));
    }

    if (latticeLocked && frontAvg < 8190) {
      float physicalRemaining = (float)frontAvg - latticeTargetFrontMm;
      if (physicalRemaining < FRONT_LOCALIZE_BRAKE_MM) {
        float u = constrain(physicalRemaining / max(1.0f, (float)FRONT_LOCALIZE_BRAKE_MM), 0.0f, 1.0f);
        // Stay fast until the last ~90 mm, then brake progressively but retain
        // enough torque for the N20 motors.
        float localScale = 0.44f + 0.56f * u;
        scale = min(scale, localScale);
      }
    }

    if (frontCandidate) {
      // 170 mm -> full speed, 80 mm -> low controlled approach.
      float denom = max(1.0f, (float)FRONT_DOCK_TRIGGER_MM - (float)FRONT_TURN_TARGET_MM);
      float u = ((float)frontAvg - (float)FRONT_TURN_TARGET_MM) / denom;
      u = constrain(u, 0.0f, 1.0f);
      float frontScale = 0.42f + 0.58f * u;
      scale = min(scale, frontScale);

      // Once the encoder centre is passed, creep only toward the physical target.
      if (avgMm >= target) scale = min(scale, 0.50f);
    } else if (remaining > 0.0f && remaining < target * 0.10f) {
      // Open-cell braking only in the final 10% instead of the old final 30%.
      scale = min(scale, 0.60f + 0.40f * (remaining / max(1.0f, target * 0.10f)));
    }

    int cmd = max(MIN_MOVE_PWM,
                  (int)lroundf(basePwm * constrain(scale, 0.35f, 1.0f)));

    int leftCmd = constrain(cmd - correction, MIN_MOVE_PWM, PWM_MAX);
    int rightCmd = constrain(cmd + correction, MIN_MOVE_PWM, PWM_MAX);
    _motors.setWheels(direction * leftCmd, direction * rightCmd);
    delay(4);
  }

  _motors.stop(true);
  if (PID_TRACE_ENABLED) {
    Serial.printf("[PID DRIVE END] result=%s\n", (success && !killed()) ? "OK" : "FAIL");
  }
  return success && !killed();
}


bool MotionController::driveAnchoredMazeCell(int basePwm, bool resetAnchor) {
  if (_cal.ticksPerMmLeft <= 0.01f || _cal.ticksPerMmRight <= 0.01f) {
    Serial.println("MOVE BLOCKED: ticks/mm is not calibrated.");
    return false;
  }

  // A turn defines a new straight-line segment.  We intentionally do NOT reset
  // this anchor at every cell.  Targets become 192, 384, 576... mm from the same
  // segment origin so per-cell stop error cannot accumulate.
  if (resetAnchor || !_mazeAnchorValid) {
    _mazeAnchorLeftTicks = _motors.leftTicks();
    _mazeAnchorRightTicks = _motors.rightTicks();
    _mazeAnchorCells = 0;
    _mazeAnchorValid = true;
    Serial.println("SEGMENT ANCHOR: reset at current cell centre");
  }

  ++_mazeAnchorCells;
  const float absoluteTargetMm = (float)_mazeAnchorCells * CELL_MM;
  const float previousTargetMm = (float)(_mazeAnchorCells - 1u) * CELL_MM;
  basePwm = constrain(abs(basePwm), MIN_MOVE_PWM, PWM_MAX);

  Serial.print("SEGMENT CELL #");
  Serial.print(_mazeAnchorCells);
  Serial.print(" target=");
  Serial.print(absoluteTargetMm, 0);
  Serial.println(" mm from anchor");

  uint32_t startMs = millis();
  uint32_t timeoutMs = 9000u;

  // IMPORTANT: keep the long straight-segment anchor only for longitudinal
  // position (192, 384, 576...).  Wheel-sync steering must be LOCAL to this
  // cell.  Using the total left/right difference from the segment anchor makes
  // old steering corrections accumulate over several cells and fight the wall
  // controller, which causes the left-right wobble seen in dfs_test.
  const int32_t syncStartLeftTicks = _motors.leftTicks();
  const int32_t syncStartRightTicks = _motors.rightTicks();

  float lastAbsProgressMm = previousTargetMm;
  uint32_t lastProgressMs = millis();
  uint8_t recoveryCount = 0;
  uint32_t lastFrontStamp = 0;
  uint8_t frontCloseSamples = 0;
  bool requestFrontAlign = false;
  bool success = false;
  float pidIntegral = 0.0f;
  float pidPrevError = 0.0f;
  bool pidHasPrev = false;
  uint32_t lastPidUs = micros();
  uint32_t lastPidTraceMs = 0;
  if (PID_TRACE_ENABLED) {
    Serial.printf("[PID MAZE ACTIVE] cell=%u pwm=%d Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  (unsigned)_mazeAnchorCells, basePwm,
                  _cal.pidKp, _cal.pidKi, _cal.pidKd);
  }

  while (!killed() && millis() - startMs < timeoutMs) {
    int32_t dLticks = _motors.leftTicks() - _mazeAnchorLeftTicks;
    int32_t dRticks = _motors.rightTicks() - _mazeAnchorRightTicks;
    float leftAbsMm = fabsf((float)dLticks) / _cal.ticksPerMmLeft;
    float rightAbsMm = fabsf((float)dRticks) / _cal.ticksPerMmRight;
    float absProgressMm = 0.5f * (leftAbsMm + rightAbsMm);
    float thisCellProgressMm = max(0.0f, absProgressMm - previousTargetMm);

    // Absolute progress watchdog. Because the target is measured from the same
    // segment anchor, an overshoot in a previous cell automatically shortens the
    // current move instead of adding another full 192 mm.
    if (absProgressMm > lastAbsProgressMm + 1.5f) {
      lastAbsProgressMm = absProgressMm;
      lastProgressMs = millis();
    } else if (millis() - lastProgressMs > DRIVE_STALL_MS &&
               absProgressMm < absoluteTargetMm - 8.0f) {
      _motors.stop(true);
      if (recoveryCount >= DRIVE_MAX_RECOVERIES) {
        Serial.println("MOVE FAIL: encoder progress stalled after recovery attempts.");
        success = false;
        break;
      }
      ++recoveryCount;
      Serial.print("MOVE RECOVERY #");
      Serial.println(recoveryCount);
      _motors.setWheels(-DRIVE_RECOVERY_PWM, -DRIVE_RECOVERY_PWM);
      delay(DRIVE_RECOVERY_REVERSE_MS);
      _motors.stop(true);
      delay(70);
      lastAbsProgressMm = absProgressMm;
      lastProgressMs = millis();
      continue;
    }

    SensorSnapshot snap = _sensors.snapshot();

    // Encoder wheel-sync + side-wall centering run continuously.
    // Longitudinal progress is absolute from the segment anchor, but steering
    // error is measured only from the beginning of THIS cell.  This prevents a
    // correction made in cell #1 from still biasing cells #2/#3.
    int32_t syncDLticks = _motors.leftTicks() - syncStartLeftTicks;
    int32_t syncDRticks = _motors.rightTicks() - syncStartRightTicks;
    float syncLeftMm = fabsf((float)syncDLticks) / _cal.ticksPerMmLeft;
    float syncRightMm = fabsf((float)syncDRticks) / _cal.ticksPerMmRight;
    float syncErrMm = syncLeftMm - syncRightMm;
    float wallErrMm = wallSteeringErrorMm(snap);
    float driveErrorMm = syncErrMm + wallErrMm;
    uint32_t pidNowUs = micros();
    float pidDt = (pidNowUs - lastPidUs) * 1e-6f;
    lastPidUs = pidNowUs;
    pidDt = constrain(pidDt, 0.001f, 0.050f);
    pidIntegral += driveErrorMm * pidDt;
    pidIntegral = constrain(pidIntegral, -40.0f, 40.0f);
    float pidDerivative = pidHasPrev ? (driveErrorMm - pidPrevError) / pidDt : 0.0f;
    float pTerm = _cal.pidKp * driveErrorMm;
    float iTerm = _cal.pidKi * pidIntegral;
    float dTerm = _cal.pidKd * pidDerivative;
    float pidOutput = pTerm + iTerm + dTerm;
    int correction = constrain((int)lroundf(pidOutput), -55, 55);
    pidPrevError = driveErrorMm;
    pidHasPrev = true;

    const uint8_t sli = SensorMap::LEFT;
    const uint8_t sri = SensorMap::RIGHT;
    if (snap.valid[sli] && _cal.sideTargetLeftMm > 0 &&
        snap.mm[sli] + SIDE_HARD_MARGIN_MM < _cal.sideTargetLeftMm) {
      correction -= SIDE_ESCAPE_BOOST;
    }
    if (snap.valid[sri] && _cal.sideTargetRightMm > 0 &&
        snap.mm[sri] + SIDE_HARD_MARGIN_MM < _cal.sideTargetRightMm) {
      correction += SIDE_ESCAPE_BOOST;
    }
    correction = constrain(correction, -78, 78);

    if (PID_TRACE_ENABLED && millis() - lastPidTraceMs >= PID_TRACE_PERIOD_MS) {
      lastPidTraceMs = millis();
      Serial.printf("[PID MAZE] cell=%u prog=%.1f err=%.2f enc=%.2f wall=%.2f P=%.2f I=%.2f D=%.2f out=%.2f corr=%d\n",
                    (unsigned)_mazeAnchorCells, thisCellProgressMm, driveErrorMm,
                    syncErrMm, wallErrMm, pTerm, iTerm, dTerm, pidOutput, correction);
    }

    // Front wall is a SAFETY / physical-centre correction, not a lattice-phase
    // estimator.  v27 tried to infer the cell index from a late ToF reading and
    // could incorrectly snap a multi-cell corridor to one cell.  Here we simply
    // brake before the wall and let front_align finish gently at 80 mm.
    const uint8_t fl = SensorMap::FRONT_LEFT;
    const uint8_t fr = SensorMap::FRONT_RIGHT;
    const uint32_t nowMs = millis();
    const bool flFresh = tofFresh(snap, fl, nowMs);
    const bool frFresh = tofFresh(snap, fr, nowMs);
    uint16_t f1 = flFresh ? correctedFrontMm(snap, fl, FRONT_LEFT_OFFSET_MM) : 8190;
    uint16_t f2 = frFresh ? correctedFrontMm(snap, fr, FRONT_RIGHT_OFFSET_MM) : 8190;
    bool anyFront = (f1 < 8190 || f2 < 8190);
    bool bothFront = (f1 < 8190 && f2 < 8190);
    uint16_t frontDiff = bothFront ? (uint16_t)abs((int)f1 - (int)f2) : 999u;
    uint16_t frontAvg = bothFront ? (uint16_t)((f1 + f2) / 2u) : min(f1, f2);
    bool frontUsable = bothFront && frontDiff <= 45u;
    const uint32_t frontPairStamp = frontUsable ? pairedFrontStamp(snap) : 0;

    // A single front sensor is never enough for maze docking, but it is enough
    // to stop an imminent collision.
    if (anyFront && min(f1, f2) < COLLISION_STOP_MM) {
      _motors.stop(true);
      Serial.println("MOVE ABORT: emergency front ToF collision guard.");
      success = false;
      break;
    }

    if (frontPairStamp != 0 && frontPairStamp != lastFrontStamp) {
      lastFrontStamp = frontPairStamp;
      if (frontUsable && frontAvg <= FRONT_PREALIGN_MM) {
        if (frontCloseSamples < 10) ++frontCloseSamples;
      } else {
        frontCloseSamples = 0;
      }
    }

    if (frontCloseSamples >= FRONT_REFERENCE_CONFIRM_SAMPLES) {
      if (thisCellProgressMm < FRONT_PREALIGN_MIN_PROGRESS_MM) {
        _motors.stop(true);
        Serial.print("MOVE ABORT: front wall appeared too early at F=");
        Serial.print(frontAvg);
        Serial.print(" mm after only ");
        Serial.print(thisCellProgressMm, 1);
        Serial.println(" mm of this cell.");
        success = false;
        break;
      }

      Serial.print("FRONT PRE-ALIGN STOP: F=");
      Serial.print(frontAvg);
      Serial.print(" mm, segment=");
      Serial.print(absProgressMm, 1);
      Serial.print(" mm, this-cell=");
      Serial.print(thisCellProgressMm, 1);
      Serial.println(" mm");
      _motors.stop(true);
      requestFrontAlign = true;
      success = true;
      break;
    }

    // Absolute target from the turn anchor.  This is the core v28 correction.
    const bool encoderDone =
        absProgressMm >= absoluteTargetMm &&
        leftAbsMm >= absoluteTargetMm * 0.94f &&
        rightAbsMm >= absoluteTargetMm * 0.94f;
    if (encoderDone) {
      success = true;
      break;
    }

    float remainingMm = absoluteTargetMm - absProgressMm;
    float scale = 1.0f;

    // Short launch ramp.
    if (thisCellProgressMm < 18.0f) {
      scale = 0.70f + 0.30f * (thisCellProgressMm / 18.0f);
    }

    // Slow progressively when a real wall is approaching.  We stop at 105 mm,
    // then the low-speed front-align controller finishes to the calibrated 80 mm.
    if (frontUsable && frontAvg < FRONT_DOCK_TRIGGER_MM) {
      float denom = max(1.0f, (float)FRONT_DOCK_TRIGGER_MM - (float)FRONT_PREALIGN_MM);
      float u = ((float)frontAvg - (float)FRONT_PREALIGN_MM) / denom;
      u = constrain(u, 0.0f, 1.0f);
      scale = min(scale, 0.45f + 0.55f * u);
    } else if (remainingMm < 22.0f) {
      scale = min(scale, 0.60f + 0.40f * (remainingMm / 22.0f));
    }

    int cmd = max(MIN_MOVE_PWM,
                  (int)lroundf(basePwm * constrain(scale, 0.38f, 1.0f)));
    int leftCmd = constrain(cmd - correction, MIN_MOVE_PWM, PWM_MAX);
    int rightCmd = constrain(cmd + correction, MIN_MOVE_PWM, PWM_MAX);
    _motors.setWheels(leftCmd, rightCmd);
    delay(4);
  }

  _motors.stop(true);
  if (PID_TRACE_ENABLED) {
    Serial.printf("[PID MAZE END] cell=%u result=%s\n",
                  (unsigned)_mazeAnchorCells, (!success || killed()) ? "FAIL" : "OK");
  }
  if (!success || killed()) return false;

  if (requestFrontAlign) {
    if (!alignFrontToWall(FRONT_TURN_TARGET_MM, FRONT_ALIGN_MAX_PWM)) {
      Serial.println("MOVE FAIL: front pre-align was detected but final 80 mm alignment failed.");
      return false;
    }

    // We are now at a known physical cell centre.  Re-anchor the straight
    // segment here so any accumulated odometry error is eliminated.
    _mazeAnchorLeftTicks = _motors.leftTicks();
    _mazeAnchorRightTicks = _motors.rightTicks();
    _mazeAnchorCells = 0;
    _mazeAnchorValid = true;
    Serial.println("SEGMENT ANCHOR: corrected by front wall at 80 mm");
  }

  return true;
}

bool MotionController::turnDegrees(float degrees, int maxPwm) {
  if (!_imu.present()) {
    Serial.println("TURN BLOCKED: MPU6050 not available.");
    return false;
  }
  if (fabsf(degrees) < 0.5f) return true;
  const int dir = degrees > 0 ? +1 : -1; // + = left, - = right
  const float target = fabsf(degrees);
  maxPwm = constrain(abs(maxPwm), MIN_MOVE_PWM, PWM_MAX);

  // Flat-robot IMU test confirms MPU Z is the physical yaw axis. Use median-of-3
  // sampling so isolated I2C/gyro spikes do not make the angle jump.
  uint32_t prevUs = micros();
  uint32_t startMs = millis();
  float angle = 0.0f;
  bool success = false;

  while (!killed() && millis() - startMs < 3500u) {
    uint32_t now = micros();
    float dt = (now - prevUs) * 1e-6f;
    prevUs = now;
    float yawRate = 0.0f;
    if (_imu.readGyroYawFilteredDps(yawRate, _cal.gyroBiasZDps)) {
      float rate = fabsf(yawRate);
      if (rate > 0.8f && dt > 0.0f && dt < 0.12f) angle += rate * dt;
    }

    float remaining = target - angle;
    if (remaining <= 1.0f) {
      success = true;
      break;
    }

    int cmd = (int)lroundf(MIN_MOVE_PWM + 1.2f * remaining);
    cmd = constrain(cmd, MIN_MOVE_PWM, maxPwm);
    if (remaining < 18.0f) cmd = min(cmd, MIN_MOVE_PWM + 12);

    // Positive degree means left: left wheel reverse, right wheel forward.
    _motors.setWheels(-dir * cmd, dir * cmd);
    delay(1);
  }

  _motors.stop(true);
  return success && !killed();
}

bool MotionController::turnEncoderTicks(int32_t signedTicks, int pwm) {
  if (PID_TRACE_ENABLED) Serial.println("[PID OFF] encoder-only turn; no PID is used here");
  if (signedTicks == 0) return true;
  const int dir = signedTicks > 0 ? +1 : -1;
  const int32_t target = labs(signedTicks);
  pwm = constrain(abs(pwm), MIN_MOVE_PWM, 100);

  _motors.resetEncoders();
  uint32_t startMs = millis();
  // 3 s is deliberately generous for one 90-degree pivot at low PWM.
  while (!killed() && millis() - startMs < 3000u) {
    int32_t l = labs(_motors.leftTicks());
    int32_t r = labs(_motors.rightTicks());
    int32_t avg = (l + r) / 2;
    if (avg >= target) {
      _motors.stop(true);
      return !killed();
    }

    // Decelerate near the target to reduce battery/friction-dependent overshoot.
    int32_t remainingTicks = target - avg;
    int turnPwm = pwm;
    if (remainingTicks < 90) turnPwm = min(turnPwm, MIN_MOVE_PWM + 10);
    if (remainingTicks < 40) turnPwm = MIN_MOVE_PWM;

    // Keep both wheels contributing approximately the same pivot distance.
    int sync = (int)constrain((long)(l - r) / 10L, -14L, 14L);
    int lp = constrain(turnPwm - sync, MIN_MOVE_PWM, 100);
    int rp = constrain(turnPwm + sync, MIN_MOVE_PWM, 100);
    _motors.setWheels(-dir * lp, dir * rp);
    delay(2);
  }

  _motors.stop(true);
  return false;
}

bool MotionController::alignFrontToWall(uint16_t targetMm, int maxPwm) {
  maxPwm = constrain(abs(maxPwm), MIN_MOVE_PWM, 100);
  targetMm = constrain((int)targetMm, 20, 140);

  uint32_t startMs = millis();
  uint32_t stableSince = 0;

  // Gains use seconds. Keep this PID independent of straight-drive calibration.
  struct AlignPid {
    float integral = 0.0f;
    float previous = 0.0f;
    float derivative = 0.0f;
    bool initialized = false;

    float update(float error, float dt, float kp, float ki, float kd, float limit) {
      float slope = initialized ? (error - previous) / dt : 0.0f;
      derivative += dt / (FRONT_ALIGN_D_FILTER_S + dt) * (slope - derivative);
      // Discard accumulated drive when the error crosses the target.
      if (initialized && error * previous < 0.0f) integral = 0.0f;
      previous = error;
      initialized = true;
      float candidate = constrain(integral + ki * error * dt,
                                  -FRONT_ALIGN_I_LIMIT_PWM, FRONT_ALIGN_I_LIMIT_PWM);
      float output = kp * error + candidate + kd * derivative;
      if (fabsf(output) <= limit || output * error < 0.0f) integral = candidate;
      return constrain(kp * error + integral + kd * derivative, -limit, limit);
    }
  } distancePid, squarePid;
  uint32_t lastControlMs = startMs;
  float leftPulse = 0.0f;
  float rightPulse = 0.0f;
  int leftSign = 0;
  int rightSign = 0;
  uint8_t previousFrontMask = 0;

  // Buffer diagnostics during control; print only after stopping the motors.
  // This keeps serial output from stretching the small correction pulses.
  struct AlignSample {
    uint32_t elapsed;
    int left, right;
    float distanceError, squareError;
    int leftPwm, rightPwm;
    int32_t leftTicks, rightTicks;
  };
  constexpr uint32_t TRACE_PERIOD_MS = 250;
  AlignSample trace[FRONT_ALIGN_TIMEOUT_MS / TRACE_PERIOD_MS + 1];
  size_t traceCount = 0;
  uint32_t lastTraceMs = startMs;
  const int32_t startLeftTicks = _motors.leftTicks();
  const int32_t startRightTicks = _motors.rightTicks();
  auto recordSample = [&](uint32_t now, int left, int right, float distErr,
                          float squareErr, int lp, int rp) {
    if (traceCount && now - lastTraceMs < TRACE_PERIOD_MS) return;
    if (traceCount >= sizeof(trace) / sizeof(trace[0])) return;
    lastTraceMs = now;
    trace[traceCount++] = {now - startMs, left, right, distErr, squareErr,
                           lp, rp, _motors.leftTicks() - startLeftTicks,
                           _motors.rightTicks() - startRightTicks};
  };

  Serial.print("FRONT ALIGN target=");
  Serial.print(targetMm);
  Serial.println(" mm");
  Serial.printf("[PID FRONT ACTIVE] distance(Kp/Ki/Kd)=%.3f/%.3f/%.3f square(Kp/Ki/Kd)=%.3f/%.3f/%.3f\n",
                _cal.frontKp, _cal.frontKi, _cal.frontKd,
                _cal.squareKp, _cal.squareKi, _cal.squareKd);
  Serial.println("[PID FRONT] distance PID = forward/back, square PID = left/right squaring");

  while (!killed() && millis() - startMs < FRONT_ALIGN_TIMEOUT_MS) {
    uint32_t now = millis();
    if (now - lastControlMs < FRONT_ALIGN_CONTROL_MS) {
      delay(1);
      continue;
    }
    float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;
    SensorSnapshot s = _sensors.snapshot();
    const uint8_t li = SensorMap::FRONT_LEFT;
    const uint8_t ri = SensorMap::FRONT_RIGHT;

    bool lv = tofFresh(s, li, now);
    bool rv = tofFresh(s, ri, now);
    // Squaring needs BOTH front sensors. If either sensor is ERR/stale, do not
    // steer from a one-sided reading; wait briefly for a fresh pair instead.
    if (!lv || !rv) {
      _motors.stop(true);
      stableSince = 0;
      delay(2);
      continue;
    }

    uint8_t frontMask = 3;
    if (frontMask != previousFrontMask) {
      distancePid = AlignPid{};
      squarePid = AlignPid{};
      stableSince = 0;
      previousFrontMask = frontMask;
    }

    float left = (float)correctedFrontMm(s, li, FRONT_LEFT_OFFSET_MM);
    float right = (float)correctedFrontMm(s, ri, FRONT_RIGHT_OFFSET_MM);
    float front = 0.5f * (left + right);
    float distErr = front - (float)targetMm; // + = too far -> move forward
    float squareErr = (lv && rv) ? (left - right) : 0.0f;

    bool distOk = fabsf(distErr) <= FRONT_ALIGN_TOL_MM;
    bool squareOk = !(lv && rv) || fabsf(squareErr) <= FRONT_SQUARE_TOL_MM;

    if (distOk && squareOk) {
      _motors.stop(true);
      recordSample(now, lv ? (int)left : -1, rv ? (int)right : -1,
                   distErr, squareErr, 0, 0);
      distancePid = AlignPid{};
      squarePid = AlignPid{};
      leftPulse = rightPulse = 0.0f;
      if (stableSince == 0) stableSince = millis();
      if (millis() - stableSince >= 120u) {
        Serial.print("FRONT ALIGN OK: FL=");
        Serial.print(lv ? (int)left : -1);
        Serial.print(" FR=");
        Serial.print(rv ? (int)right : -1);
        Serial.print(" avg=");
        Serial.println(front, 1);
        Serial.println("[PID FRONT END] result=OK");
        return true;
      }
      continue;
    }
    stableSince = 0;

    // Translation: positive when too far from the wall, negative when too close.
    float drive = 0.0f;
    if (!distOk) {
      drive = distancePid.update(distErr, dt, _cal.frontKp, _cal.frontKi,
                                 _cal.frontKd, maxPwm);
    } else distancePid = AlignPid{};

    // Squaring: if FL > FR, the right-front corner is closer and the nose is
    // rotated left; command a small RIGHT correction (negative turn).
    float turn = 0.0f;
    if (!squareOk && lv && rv) {
      turn = -squarePid.update(squareErr, dt, _cal.squareKp, _cal.squareKi,
                               _cal.squareKd, maxPwm);
    } else squarePid = AlignPid{};

    // Scale both wheels together to preserve the distance/squaring balance.
    float leftDemand = drive - turn;
    float rightDemand = drive + turn;
    float peak = max(fabsf(leftDemand), fabsf(rightDemand));
    if (peak > maxPwm) {
      leftDemand *= maxPwm / peak;
      rightDemand *= maxPwm / peak;
    }

    // Below motor breakaway PWM, alternate short drive/brake intervals rather
    // than forcing every small error to continuous MIN_MOVE_PWM.
    auto pulseCommand = [](float demand, float &accumulator, int &previousSign) {
      int sign = demand > 0.0f ? 1 : (demand < 0.0f ? -1 : 0);
      if (sign != previousSign) accumulator = 0.0f;
      previousSign = sign;
      float magnitude = fabsf(demand);
      if (magnitude >= MIN_MOVE_PWM) {
        accumulator = 0.0f;
        return (int)lroundf(demand);
      }
      accumulator += magnitude;
      if (accumulator >= MIN_MOVE_PWM) {
        accumulator -= MIN_MOVE_PWM;
        return sign * MIN_MOVE_PWM;
      }
      return 0;
    };
    int leftCmd = pulseCommand(leftDemand, leftPulse, leftSign);
    int rightCmd = pulseCommand(rightDemand, rightPulse, rightSign);

    _motors.setWheels(leftCmd, rightCmd);
    recordSample(now, lv ? (int)left : -1, rv ? (int)right : -1,
                 distErr, squareErr, leftCmd, rightCmd);
  }

  _motors.stop(true);
  Serial.println(killed() ? "FRONT ALIGN stopped: Key2." : "FRONT ALIGN timeout: distance/squaring did not settle.");
  Serial.println("[PID FRONT END] result=FAIL");
  Serial.println("ALIGN TRACE: ms FL FR distErr squareErr pwmL pwmR ticksL ticksR");
  for (size_t i = 0; i < traceCount; ++i) {
    const AlignSample &s = trace[i];
    Serial.printf("%lu %d %d %.1f %.1f %d %d %ld %ld\n",
                  (unsigned long)s.elapsed, s.left, s.right,
                  s.distanceError, s.squareError, s.leftPwm, s.rightPwm,
                  (long)s.leftTicks, (long)s.rightTicks);
  }
  Serial.printf("Required: |distErr| <= %u mm and |squareErr| <= %u mm for 120 ms.\n",
                (unsigned)FRONT_ALIGN_TOL_MM, (unsigned)FRONT_SQUARE_TOL_MM);
  return false;
}

bool MotionController::turnToHeading(Heading &current, Heading target, int maxPwm) {
  (void)maxPwm; // navigation intentionally uses the calibrated encoder pivot.

  const int c = static_cast<int>(current);
  const int t = static_cast<int>(target);
  const int delta = (t - c + 4) % 4;
  bool ok = true;

  // Before any pivot, if there is a front wall, use it as a physical reference.
  // This prevents the robot from entering the turn while touching the wall or
  // while sitting several centimetres off the repeatable cell-centre position.
  if (delta != 0 && wallFront()) {
    if (!alignFrontToWall(FRONT_TURN_TARGET_MM, FRONT_ALIGN_MAX_PWM)) {
      Serial.println("TURN BLOCKED: front-wall alignment failed.");
      _motors.stop(true);
      return false;
    }
    _motors.stop(true);
    delay(180);
  }

  if (delta == 0) {
    ok = true;
  } else if (delta == 1) {
    // RIGHT 90 degrees.
    ok = turnEncoderTicks(-TURN_RIGHT_90_TICKS);
    if (ok) delay(250);
  } else if (delta == 2) {
    // 180 degrees: two fully stopped calibrated 90-degree pivots.
    ok = turnEncoderTicks(TURN_LEFT_90_TICKS);
    if (ok) {
      delay(350);
      ok = turnEncoderTicks(TURN_LEFT_90_TICKS);
    }
    if (ok) delay(300);
  } else if (delta == 3) {
    // LEFT 90 degrees.
    ok = turnEncoderTicks(TURN_LEFT_90_TICKS);
    if (ok) delay(250);
  }

  if (ok) current = target;
  return ok;
}

} // namespace MM3
