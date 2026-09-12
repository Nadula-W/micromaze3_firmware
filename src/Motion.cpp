#include "Motion.h"
#include <math.h>

namespace MM3 {

bool MotionController::calibrationReady() const {
  return isfinite(_cal.ticksPerMmLeft) && isfinite(_cal.ticksPerMmRight) &&
         _cal.ticksPerMmLeft > 0.01f && _cal.ticksPerMmRight > 0.01f && _imu.present();
}

bool MotionController::wallByIndex(uint8_t index) const {
  SensorSnapshot s = _sensors.snapshot();
  if (index >= 4 || !s.valid[index]) return false;
  return s.mm[index] < _cal.wallThresholdMm[index];
}

bool MotionController::wallLeft() const { return wallByIndex(SensorMap::LEFT); }
bool MotionController::wallRight() const { return wallByIndex(SensorMap::RIGHT); }

bool MotionController::wallFront() const {
  SensorSnapshot s = _sensors.snapshot();
  const uint8_t a = SensorMap::FRONT_LEFT;
  const uint8_t b = SensorMap::FRONT_RIGHT;
  bool wa = s.valid[a] && s.mm[a] < _cal.wallThresholdMm[a];
  bool wb = s.valid[b] && s.mm[b] < _cal.wallThresholdMm[b];
  // Either front sensor seeing a close wall is enough to stop a maze move.
  return wa || wb;
}

uint16_t MotionController::distanceLeft() const {
  SensorSnapshot s = _sensors.snapshot();
  return s.valid[SensorMap::LEFT] ? s.mm[SensorMap::LEFT] : 8190;
}
uint16_t MotionController::distanceRight() const {
  SensorSnapshot s = _sensors.snapshot();
  return s.valid[SensorMap::RIGHT] ? s.mm[SensorMap::RIGHT] : 8190;
}
uint16_t MotionController::distanceFront() const {
  SensorSnapshot s = _sensors.snapshot();
  uint16_t a = s.valid[SensorMap::FRONT_LEFT] ? s.mm[SensorMap::FRONT_LEFT] : 8190;
  uint16_t b = s.valid[SensorMap::FRONT_RIGHT] ? s.mm[SensorMap::FRONT_RIGHT] : 8190;
  if (a == 8190) return b;
  if (b == 8190) return a;
  return (uint16_t)((a + b) / 2u);
}

int MotionController::wallSteeringCorrection(const SensorSnapshot &s) const {
  const uint8_t li = SensorMap::LEFT;
  const uint8_t ri = SensorMap::RIGHT;
  bool lwall = s.valid[li] && s.mm[li] < _cal.wallThresholdMm[li];
  bool rwall = s.valid[ri] && s.mm[ri] < _cal.wallThresholdMm[ri];
  float steer = 0.0f;

  if (lwall && rwall && _cal.sideTargetLeftMm && _cal.sideTargetRightMm) {
    // Positive steer turns left: left wheel slower, right wheel faster.
    float lErr = (float)s.mm[li] - _cal.sideTargetLeftMm;
    float rErr = (float)s.mm[ri] - _cal.sideTargetRightMm;
    steer = _cal.wallKp * (lErr - rErr) * 0.5f;
  } else if (lwall && _cal.sideTargetLeftMm) {
    steer = _cal.wallKp * ((float)s.mm[li] - _cal.sideTargetLeftMm);
  } else if (rwall && _cal.sideTargetRightMm) {
    steer = -_cal.wallKp * ((float)s.mm[ri] - _cal.sideTargetRightMm);
  }
  return constrain((int)lroundf(steer), -65, 65);
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

  // v36: continuous side CLOSE/NOT-CLOSE tracking for the destination cell.
  _lastMoveSideObs = MoveSideObservation{};
  uint32_t lastSideObsStamp = 0;

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

    // Encoder synchronization: if left is farther ahead, slow left / speed right.
    float syncErrMm = leftMm - rightMm;
    int correction = constrain((int)lroundf(_cal.straightKp * syncErrMm), -55, 55);

    bool frontCandidate = false;
    bool frontReferenceActive = false;
    uint16_t frontAvg = 8190;

    if (direction > 0 && useWallCentering) {
      SensorSnapshot s = _sensors.snapshot();
      correction += wallSteeringCorrection(s);

      // v36 CONTINUOUS SIDE OPENING CONFIRMATION.
      // Only CLOSE is calibrated. During the latter half of the move, any fresh
      // sample that is not CLOSE increments a consecutive streak. One CLOSE
      // sample resets that streak. Three consecutive NOT-CLOSE samples latch
      // OPEN for that side of the destination cell.
      if (s.stampMs != lastSideObsStamp &&
          avgMm >= target * SIDE_OBS_WINDOW_START_FRAC &&
          avgMm <= target * SIDE_OBS_WINDOW_END_FRAC) {
        lastSideObsStamp = s.stampMs;
        auto observeSide = [&](uint8_t idx, uint8_t &streak, SideObservation &state) {
          const bool closeNow = s.valid[idx] && s.mm[idx] < _cal.wallThresholdMm[idx];
          if (closeNow) {
            streak = 0;
            if (state != SideObservation::Open) state = SideObservation::Wall;
          } else {
            if (streak < 250) ++streak;
            if (streak >= SIDE_OPEN_CONFIRM_SAMPLES) state = SideObservation::Open;
          }
        };
        observeSide(SensorMap::LEFT, _lastMoveSideObs.leftNoCloseStreak, _lastMoveSideObs.left);
        observeSide(SensorMap::RIGHT, _lastMoveSideObs.rightNoCloseStreak, _lastMoveSideObs.right);
      }

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
      correction = constrain(correction, -78, 78);

      // CONTINUOUS front-wall tracking.  Do not stop merely because the encoder
      // says 192 mm if a real front wall is already visible.  In that situation
      // the physical 80 mm wall distance is the stronger longitudinal reference.
      uint16_t f1 = s.valid[SensorMap::FRONT_LEFT] ? s.mm[SensorMap::FRONT_LEFT] : 8190;
      uint16_t f2 = s.valid[SensorMap::FRONT_RIGHT] ? s.mm[SensorMap::FRONT_RIGHT] : 8190;
      bool bothFrontValid = (f1 < 8190 && f2 < 8190);
      bool anyFrontValid = (f1 < 8190 || f2 < 8190);
      uint16_t frontDiff = bothFrontValid ? (uint16_t)abs((int)f1 - (int)f2) : 999u;
      frontAvg = bothFrontValid ? (uint16_t)((f1 + f2) / 2u) : min(f1, f2);
      bool frontLooksUsable = anyFrontValid && (!bothFrontValid || frontDiff <= 40u);

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
            if (s.stampMs != lastFrontStamp && latticeCandidateSamples < 10)
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
          if (physicalRemaining <= FRONT_ALIGN_TOL_MM) {
            if (s.stampMs != lastFrontStamp) {
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

      if (s.stampMs != lastFrontStamp) {
        lastFrontStamp = s.stampMs;
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
          frontAvg <= FRONT_TURN_TARGET_MM + FRONT_ALIGN_TOL_MM) {
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

  // If no 3-sample opening was latched, treat the side as a wall for safety.
  // Mapping code can still re-check at rest before committing the cell walls.
  if (direction > 0 && success && !killed()) {
    if (_lastMoveSideObs.left == SideObservation::Unknown)
      _lastMoveSideObs.left = SideObservation::Wall;
    if (_lastMoveSideObs.right == SideObservation::Unknown)
      _lastMoveSideObs.right = SideObservation::Wall;
    _lastMoveSideObs.valid = true;
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
    int correction = constrain((int)lroundf(_cal.straightKp * syncErrMm), -55, 55);
    correction += wallSteeringCorrection(snap);

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

    // Front wall is a SAFETY / physical-centre correction, not a lattice-phase
    // estimator.  v27 tried to infer the cell index from a late ToF reading and
    // could incorrectly snap a multi-cell corridor to one cell.  Here we simply
    // brake before the wall and let front_align finish gently at 80 mm.
    uint16_t f1 = snap.valid[SensorMap::FRONT_LEFT] ? snap.mm[SensorMap::FRONT_LEFT] : 8190;
    uint16_t f2 = snap.valid[SensorMap::FRONT_RIGHT] ? snap.mm[SensorMap::FRONT_RIGHT] : 8190;
    bool anyFront = (f1 < 8190 || f2 < 8190);
    bool bothFront = (f1 < 8190 && f2 < 8190);
    uint16_t frontDiff = bothFront ? (uint16_t)abs((int)f1 - (int)f2) : 999u;
    uint16_t frontAvg = bothFront ? (uint16_t)((f1 + f2) / 2u) : min(f1, f2);
    bool frontUsable = anyFront && (!bothFront || frontDiff <= 45u);

    if (snap.stampMs != lastFrontStamp) {
      lastFrontStamp = snap.stampMs;
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

    int cmd = (int)lroundf(MIN_MOVE_PWM + _cal.gyroTurnKp * remaining);
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

    // Keep both wheels contributing approximately the same pivot distance.
    int sync = (int)constrain((long)(l - r) / 10L, -16L, 16L);
    int lp = constrain(pwm - sync, MIN_MOVE_PWM, 100);
    int rp = constrain(pwm + sync, MIN_MOVE_PWM, 100);
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

  Serial.print("FRONT ALIGN target=");
  Serial.print(targetMm);
  Serial.println(" mm");

  while (!killed() && millis() - startMs < 2200u) {
    SensorSnapshot s = _sensors.snapshot();
    const uint8_t li = SensorMap::FRONT_LEFT;
    const uint8_t ri = SensorMap::FRONT_RIGHT;

    bool lv = s.valid[li] && s.mm[li] < 8190;
    bool rv = s.valid[ri] && s.mm[ri] < 8190;
    if (!lv && !rv) {
      _motors.stop(true);
      Serial.println("FRONT ALIGN skipped: no valid front-wall reading.");
      return false;
    }

    float left = lv ? (float)s.mm[li] : NAN;
    float right = rv ? (float)s.mm[ri] : NAN;
    float front = lv && rv ? 0.5f * (left + right) : (lv ? left : right);
    float distErr = front - (float)targetMm; // + = too far -> move forward
    float squareErr = (lv && rv) ? (left - right) : 0.0f;

    bool distOk = fabsf(distErr) <= FRONT_ALIGN_TOL_MM;
    bool squareOk = !(lv && rv) || fabsf(squareErr) <= FRONT_SQUARE_TOL_MM;

    if (distOk && squareOk) {
      _motors.stop(true);
      if (stableSince == 0) stableSince = millis();
      if (millis() - stableSince >= 120u) {
        Serial.print("FRONT ALIGN OK: FL=");
        Serial.print(lv ? (int)left : -1);
        Serial.print(" FR=");
        Serial.print(rv ? (int)right : -1);
        Serial.print(" avg=");
        Serial.println(front, 1);
        return true;
      }
      delay(8);
      continue;
    }
    stableSince = 0;

    // Translation: positive when too far from the wall, negative when too close.
    int drive = 0;
    if (!distOk) {
      int mag = (int)lroundf(FRONT_ALIGN_KP * fabsf(distErr));
      mag = constrain(mag, MIN_MOVE_PWM, maxPwm);
      drive = (distErr > 0.0f) ? mag : -mag;
    }

    // Squaring: if FL > FR, the right-front corner is closer and the nose is
    // rotated left; command a small RIGHT correction (negative turn).
    int turn = 0;
    if (!squareOk && lv && rv) {
      turn = (int)lroundf(-FRONT_SQUARE_KP * squareErr);
      turn = constrain(turn, -28, 28);
      if (drive == 0 && turn != 0 && abs(turn) < MIN_MOVE_PWM) {
        turn = (turn > 0) ? MIN_MOVE_PWM : -MIN_MOVE_PWM;
      }
    }

    int leftCmd = constrain(drive - turn, -maxPwm, maxPwm);
    int rightCmd = constrain(drive + turn, -maxPwm, maxPwm);

    // Ensure a commanded wheel is above the motor's useful dead-zone.
    auto liftDeadzone = [](int v) {
      if (v == 0) return 0;
      if (abs(v) < MIN_MOVE_PWM) return v > 0 ? MIN_MOVE_PWM : -MIN_MOVE_PWM;
      return v;
    };
    leftCmd = liftDeadzone(leftCmd);
    rightCmd = liftDeadzone(rightCmd);

    _motors.setWheels(leftCmd, rightCmd);
    delay(8);
  }

  _motors.stop(true);
  Serial.println("FRONT ALIGN timeout.");
  return false;
}

bool MotionController::turnToHeading(Heading &current, Heading target, int maxPwm) {
  (void)maxPwm; // navigation intentionally uses the calibrated encoder pivot.

  const int c = static_cast<int>(current);
  const int t = static_cast<int>(target);
  const int delta = (t - c + 4) % 4;
  bool ok = true;

  if (delta == 0) return true;

  // v32d TURN PREPARATION:
  // A repeatable 378-tick pivot is only a true 90-degree maze turn if the robot
  // enters the pivot stationary and straight.  Always remove forward momentum
  // before a turn.  If a front wall exists, use it as the strongest physical
  // reference to correct both longitudinal position and yaw before pivoting.
  _motors.stop(true);
  delay(120);

  if (wallFront()) {
    Serial.println("TURN PREP: front wall -> align/square before pivot");
    if (!alignFrontToWall(FRONT_TURN_TARGET_MM, FRONT_ALIGN_MAX_PWM)) {
      Serial.println("TURN BLOCKED: front-wall alignment failed.");
      _motors.stop(true);
      return false;
    }
    _motors.stop(true);
    delay(160);
  } else {
    // With no front wall there is no absolute yaw reference available from the
    // current sensor layout.  Do not invent a correction; simply let the robot
    // fully settle so the calibrated encoder pivot starts from zero momentum.
    Serial.println("TURN PREP: no front wall -> stationary settle");
  }

  if (delta == 1) {
    // RIGHT 90 degrees.
    ok = turnEncoderTicks(-TURN_90_TICKS);
  } else if (delta == 2) {
    // 180 degrees: two independently stopped calibrated 90-degree pivots.
    ok = turnEncoderTicks(TURN_90_TICKS);
    if (ok) {
      _motors.stop(true);
      delay(350);
      ok = turnEncoderTicks(TURN_90_TICKS);
    }
  } else if (delta == 3) {
    // LEFT 90 degrees.
    ok = turnEncoderTicks(TURN_90_TICKS);
  }

  // Let the chassis settle before the next 192 mm cell move starts.  This avoids
  // carrying rotational inertia into the following straight segment.
  _motors.stop(true);
  if (ok) delay(delta == 2 ? 300 : 200);

  if (ok) current = target;
  return ok;
}

} // namespace MM3
