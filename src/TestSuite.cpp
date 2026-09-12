#include "TestSuite.h"
#include <math.h>

namespace MM3 {

String TestSuite::readLine(uint32_t timeoutMs) {
  String s;
  uint32_t start = millis();
  Stream &console = _console ? *_console : Serial;
  while (true) {
    while (console.available()) {
      char c = (char)console.read();
      if (c == '\r') continue;
      if (c == '\n') {
        s.trim();
        return s;
      }
      s += c;
    }
    if (MotorSystem::emergencyLatched()) return String("__KILL__");
    if (timeoutMs && millis() - start >= timeoutMs) {
      s.trim();
      return s;
    }
    delay(2);
  }
}

bool TestSuite::waitEnter(const char *prompt, uint32_t timeoutMs) {
  Stream &console = _console ? *_console : Serial;
  console.println(prompt);
  if (timeoutMs) {
    console.print("(waiting up to ");
    console.print((unsigned long)timeoutMs);
    console.println(" ms)");
  }
  String s = readLine(timeoutMs);
  return s != "__KILL__";
}

float TestSuite::askFloat(const char *prompt, float defaultValue) {
  Stream &console = _console ? *_console : Serial;
  console.print(prompt);
  if (isfinite(defaultValue)) {
    console.print(" [default "); console.print(defaultValue, 3); console.print("]");
  }
  console.println();
  String s = readLine();
  if (s.length() == 0 && isfinite(defaultValue)) return defaultValue;
  return s.toFloat();
}

uint16_t TestSuite::sampleSensor(uint8_t index, uint16_t samples) {
  uint32_t sum = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    SensorSnapshot s = _sensorHub.snapshot();
    if (index < 4 && s.valid[index] && s.mm[index] < 4000) {
      sum += s.mm[index];
      ++n;
    }
    delay(25);
  }
  return n ? (uint16_t)(sum / n) : 8190;
}

void TestSuite::scanMainI2C(Print &out) {
  out.println("Main I2C scan on GPIO8/9:");
  uint8_t count = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    _wire.beginTransmission(a);
    if (_wire.endTransmission() == 0) {
      out.print("  found 0x"); if (a < 16) out.print('0'); out.println(a, HEX);
      ++count;
    }
  }
  out.print("Total main-bus devices: "); out.println(count);
  out.println("Expected from hardware document: 0x20 PCF8574, 0x50/0x51 EEPROM, 0x68 MPU6050.");
}

void TestSuite::testIO(Print &out) {
  if (!_io.present()) {
    out.println("PCF8574 I/O test: FAIL - device missing");
    return;
  }
  uint8_t raw = _io.readRaw();
  out.print("PCF raw=0b");
  for (int i = 7; i >= 0; --i) out.print((raw >> i) & 1);
  out.print(" mode="); out.println(_io.modeBits(), BIN);
  out.print("Key1="); out.print(_io.key1()); out.print(" Key2="); out.println(_io.key2());
  out.println("LED1 -> LED2 -> buzzer output test...");
  _io.setLed1(true); delay(250); _io.setLed1(false);
  _io.setLed2(true); delay(250); _io.setLed2(false);
  _io.beep(120);
  out.println("I/O output sequence complete. Visually confirm LED/buzzer behavior.");
}

void TestSuite::testToF(Print &out, uint16_t seconds) {
  out.println("\n=== VL53L0X LIVE TEST ===");
  out.print("VL53L0X present mask = 0x"); out.println(_tofArray.presentMask(), HEX);
  out.println("Confirmed mapping: 0=FRONT_LEFT, 1=LEFT, 2=RIGHT, 3=FRONT_RIGHT.");
  out.println("Move a hand/wall in front of ONE sensor at a time and confirm the matching index changes.");
  out.println("v38 uses the SAME Adafruit rangingTest() single-shot method as the proven standalone S1-S4 sketch.");
  out.println("ERR means that scan did not produce a usable range; present mask 0xF means all 4 devices initialized.");
  out.println("To verify one sensor, place a flat white card 50-100 mm directly in front of that sensor.");

  uint32_t until = millis() + seconds * 1000UL;
  while ((int32_t)(until - millis()) > 0) {
    SensorSnapshot s = _sensorHub.snapshot();
    out.print("ToF mm: ");
    for (uint8_t i = 0; i < 4; ++i) {
      out.print(i); out.print('=');
      if (s.valid[i]) out.print(s.mm[i]); else out.print("ERR");
      out.print(i == 3 ? ' ' : ' ');
    }
    out.print(" age=");
    if (s.stampMs == 0) out.println("NO-DATA");
    else { out.print(millis() - s.stampMs); out.println("ms"); }
    delay(200);
  }
  out.println("ToF live test complete.");
}

void TestSuite::testImu(Print &out) {
  if (!_imu.present()) {
    out.println("MPU6050 test: FAIL - missing");
    return;
  }
  for (uint8_t i = 0; i < 10; ++i) {
    _imu.printOne(out, _cal.gyroBiasZDps);
    delay(100);
  }
}

void TestSuite::testMotorAndEncoder(Print &out) {
  out.println("MOTOR TEST: lift the robot so both wheels are free. Key2 can kill immediately.");
  if (!waitEnter("Press ENTER when safe.")) return;
  _motors.resetEncoders();

  out.println("Motor A forward 500 ms");
  _motors.setMotorA(90); delay(500); _motors.stop(true); delay(200); if (MotorSystem::emergencyLatched()) return;
  out.print("Encoder A ticks: "); out.println(_motors.encoderA());

  out.println("Motor A reverse 500 ms");
  _motors.setMotorA(-90); delay(500); _motors.stop(true); delay(200); if (MotorSystem::emergencyLatched()) return;
  out.print("Encoder A ticks total: "); out.println(_motors.encoderA());

  out.println("Motor B forward 500 ms");
  _motors.setMotorB(90); delay(500); _motors.stop(true); delay(200); if (MotorSystem::emergencyLatched()) return;
  out.print("Encoder B ticks: "); out.println(_motors.encoderB());

  out.println("Motor B reverse 500 ms");
  _motors.setMotorB(-90); delay(500); _motors.stop(true); delay(200); if (MotorSystem::emergencyLatched()) return;
  out.print("Encoder B ticks total: "); out.println(_motors.encoderB());

  out.println("Motor truth-table test complete. Confirm forward/reverse directions match your wheel wiring.");
}

void TestSuite::testEncoderManual(Print &out) {
  _motors.stop(false);
  _motors.resetEncoders();
  out.println("Manually rotate/roll both wheels for 5 seconds...");
  uint32_t until = millis() + 5000;
  while ((int32_t)(until - millis()) > 0) {
    out.print("left ticks="); out.print(_motors.leftTicks());
    out.print(" right ticks="); out.println(_motors.rightTicks());
    delay(250);
  }
  out.println("PASS condition: each moving wheel produces changing counts without large random jumps.");
}

void TestSuite::testDistanceMove(float mm, int pwm, Print &out) {
  if (_cal.ticksPerMmLeft <= 0.01f || _cal.ticksPerMmRight <= 0.01f) {
    out.println("Distance test blocked: run 'cal_roll 540' (or 180) first.");
    return;
  }
  out.print("Distance move target = "); out.print(mm, 1); out.println(" mm");
  out.println("Place robot on a straight measured lane. Press ENTER to start.");
  readLine();
  _motors.resetEncoders();
  bool ok = _motion.driveDistanceMm(mm, pwm, true);
  int32_t lt = _motors.leftTicks();
  int32_t rt = _motors.rightTicks();
  float lmm = fabsf((float)lt) / _cal.ticksPerMmLeft;
  float rmm = fabsf((float)rt) / _cal.ticksPerMmRight;
  out.print("Motion result: "); out.println(ok ? "COMPLETED" : "FAILED/KILLED");
  out.print("Encoder-estimated left/right mm = "); out.print(lmm, 1); out.print(" / "); out.println(rmm, 1);
  out.println("Measure the REAL floor distance and type it in mm, or just ENTER to skip accuracy calculation:");
  String s = readLine();
  if (s.length()) {
    float actual = s.toFloat();
    if (actual > 1.0f) {
      float error = actual - fabsf(mm);
      float pct = fabsf(mm) > 1 ? 100.0f * error / fabsf(mm) : 0;
      out.print("Measured error = "); out.print(error, 2); out.print(" mm ("); out.print(pct, 3); out.println(" %)");
    } else {
      out.println("Floor measurement skipped/invalid (enter a positive distance in mm, e.g. 538).");
    }
  } else {
    out.println("Floor measurement skipped.");
  }
}

void TestSuite::testTurn(float deg, Print &out) {
  out.print("Gyro turn target = "); out.print(deg, 1); out.println(" deg. Press ENTER when robot can rotate safely.");
  readLine();
  bool ok = _motion.turnDegrees(deg);
  out.print("Turn result: "); out.println(ok ? "COMPLETED" : "FAILED/KILLED");
  out.println("Physically measure/mark the final angle. Tune gyro bias/turn Kp if needed.");
}

void TestSuite::testGyroManualAngle(float expectedDeg, Print &out) {
  if (!_imu.present()) {
    out.println("GYRO ANGLE TEST: MPU6050 missing.");
    return;
  }
  if (expectedDeg <= 0.0f) expectedDeg = 90.0f;
  out.println("GYRO MANUAL ANGLE TEST - MOTORS STAY OFF.");
  out.print("Mark the robot heading. Press ENTER, rotate the whole robot by exactly ");
  out.print(expectedDeg, 1);
  out.println(" degrees in ONE direction by hand, then press ENTER again.");
  if (readLine() == "__KILL__") return;

  uint32_t prevUs = micros();
  uint32_t startMs = millis();
  uint32_t lastPrint = startMs;
  float signedAngle = 0.0f;
  float absoluteAngle = 0.0f;
  Stream &console = _console ? *_console : Serial;
  bool done = false;

  while (!MotorSystem::emergencyLatched() && millis() - startMs < 12000u) {
    uint32_t nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;
    float rate = _imu.readGyroZDps(_cal.gyroBiasZDps);
    if (dt > 0.0f && dt < 0.1f) {
      signedAngle += rate * dt;
      absoluteAngle += fabsf(rate) * dt;
    }

    while (console.available()) {
      char c = (char)console.read();
      if (c == '\n') { done = true; break; }
    }
    if (done) break;

    if (millis() - lastPrint >= 250u) {
      lastPrint = millis();
      out.print("gyroZ="); out.print(rate, 2);
      out.print(" dps  integrated(abs)="); out.print(absoluteAngle, 2);
      out.println(" deg");
    }
    delay(2);
  }

  out.print("Manual angle expected = "); out.print(expectedDeg, 2); out.println(" deg");
  out.print("Gyro integrated signed = "); out.print(signedAngle, 2); out.println(" deg");
  out.print("Gyro integrated absolute = "); out.print(absoluteAngle, 2); out.println(" deg");
  if (!done) out.println("NOTE: test ended by timeout/Key2 rather than ENTER.");
}

void TestSuite::testGyroAxes(Print &out) {
  if (!_imu.present()) {
    out.println("GYRO AXES: MPU6050 missing.");
    return;
  }
  out.println("GYRO XYZ LIVE TEST - keep robot FLAT and rotate it left/right on the floor.");
  out.println("The yaw axis should show the largest sustained +/- dps. Invalid >550 dps samples are rejected.");
  uint32_t until = millis() + 5000u;
  uint32_t lastPrint = 0;
  uint32_t bad = 0;
  while ((int32_t)(until - millis()) > 0 && !MotorSystem::emergencyLatched()) {
    float x, y, z;
    bool ok = _imu.readGyroXYZDps(x, y, z);
    if (!ok) { ++bad; delay(3); continue; }
    if (millis() - lastPrint >= 80u) {
      lastPrint = millis();
      out.print("GX="); out.print(x, 2);
      out.print("  GY="); out.print(y, 2);
      out.print("  GZ="); out.println(z, 2);
    }
    delay(3);
  }
  out.print("Rejected invalid gyro samples: "); out.println(bad);
}

void TestSuite::testGyroAxis90(Print &out) {
  if (!_imu.present()) {
    out.println("GYRO AXIS 90: MPU6050 missing.");
    return;
  }
  _motors.stop(true);
  out.println("GYRO AXIS AUTO TEST - MOTORS OFF.");
  out.println("Keep robot FLAT and STILL for robust bias measurement.");
  float bias = _imu.calibrateGyroYaw(out, 201);
  out.println("In 3 seconds rotate exactly 90 degrees LEFT, smoothly, then hold still.");
  for (int i=3;i>=1;--i) { out.print(i); out.println("..."); delay(1000); }
  out.println("GO");

  float signedAngle=0.0f, absoluteAngle=0.0f;
  uint32_t rejected=0;
  uint32_t prevUs=micros();
  uint32_t startMs=millis();
  // IMPORTANT: do not print inside this integration loop. Browser-terminal
  // printing can block long enough to lose integration time.
  while (millis()-startMs < 3500u && !MotorSystem::emergencyLatched()) {
    uint32_t nowUs=micros();
    float dt=(nowUs-prevUs)*1e-6f;
    prevUs=nowUs;
    float z=0.0f;
    if (!_imu.readGyroYawFilteredDps(z, bias)) { ++rejected; delay(1); continue; }
    if (dt > 0.0f && dt < 0.12f) {
      signedAngle += z*dt;
      absoluteAngle += fabsf(z)*dt;
    }
    delay(1);
  }
  out.println("GYRO YAW 90 RESULT:");
  out.print("Physical yaw axis = MPU Z; robust bias = "); out.print(bias,3); out.println(" dps");
  out.print("Integrated signed Z = "); out.print(signedAngle,2); out.println(" deg");
  out.print("Integrated absolute Z = "); out.print(absoluteAngle,2); out.println(" deg");
  out.print("Rejected filtered reads = "); out.println(rejected);
  if (absoluteAngle < 70.0f || absoluteAngle > 115.0f)
    out.println("WARNING: result is not close enough to 90 deg; do not use normal turn yet.");
  else
    out.println("PASS candidate: yaw integration is close enough for a low-PWM powered turn diagnostic.");
}

void TestSuite::testTurnDiagnostic(float deg, Print &out) {
  if (!_imu.present()) { out.println("TURN DIAG: MPU6050 missing."); return; }
  if (fabsf(deg) < 1.0f) { out.println("Usage: turn_diag <degrees>"); return; }
  const int dir = deg > 0.0f ? +1 : -1;
  const float target = fabsf(deg);
  const int diagPwm = 68;
  uint32_t hardLimitMs = target <= 100.0f ? 1800u : (target <= 190.0f ? 3000u : 4200u);

  out.println("TURN DIAGNOSTIC - FILTERED MPU-Z YAW + LOW PWM + HARD TIMEOUT.");
  out.print("Target="); out.print(deg,1); out.print(" deg, PWM="); out.print(diagPwm);
  out.print(", timeout="); out.print(hardLimitMs); out.println(" ms");
  out.println("Put robot on floor with free space. Keep Key2 ready. Press ENTER to start.");
  if (readLine() == "__KILL__") return;

  _motors.resetEncoders();
  uint32_t prevUs=micros(), startMs=millis();
  float angle=0.0f;
  uint32_t rejected=0;
  bool reached=false;
  while (!MotorSystem::emergencyLatched() && millis()-startMs < hardLimitMs) {
    uint32_t nowUs=micros();
    float dt=(nowUs-prevUs)*1e-6f; prevUs=nowUs;
    float rate=0.0f;
    if (_imu.readGyroYawFilteredDps(rate, _cal.gyroBiasZDps)) {
      if (dt>0.0f && dt<0.12f) angle += fabsf(rate)*dt;
    } else ++rejected;
    if (angle >= target) { reached=true; break; }
    _motors.setWheels(-dir*diagPwm, dir*diagPwm);
    delay(1);
  }
  _motors.stop(true);
  out.print("TURN DIAG RESULT: integrated Z yaw="); out.print(angle,2); out.print(" deg; ");
  out.println(reached ? "target reached" : "SAFETY TIMEOUT before target");
  out.print("Final ticks L/R = "); out.print(_motors.leftTicks()); out.print(" / "); out.println(_motors.rightTicks());
  out.print("Rejected reads = "); out.println(rejected);
  out.println("Measure the PHYSICAL angle and send both physical angle + this output.");
}

void TestSuite::testEncoderTurnTicks(int32_t signedTicks, Print &out) {
  if (signedTicks == 0) { out.println("Usage: turn_ticks <ticks>; +left, -right"); return; }
  const int dir = signedTicks > 0 ? +1 : -1;
  const int32_t target = labs(signedTicks);
  const int pwm = 68;
  out.println("ENCODER-ONLY TURN DIAGNOSTIC - gyro ignored.");
  out.print("Target average wheel ticks = "); out.print(target);
  out.println(". + means left, - means right. Keep Key2 ready. Press ENTER.");
  if (readLine() == "__KILL__") return;
  _motors.resetEncoders();
  uint32_t startMs=millis();
  bool reached=false;
  while (!MotorSystem::emergencyLatched() && millis()-startMs < 3000u) {
    int32_t l=labs(_motors.leftTicks()), r=labs(_motors.rightTicks());
    int32_t avg=(l+r)/2;
    if (avg >= target) { reached=true; break; }
    int sync=(int)constrain((long)(l-r)/10L, -16L, 16L);
    int lp=constrain(pwm-sync, MIN_MOVE_PWM, 100);
    int rp=constrain(pwm+sync, MIN_MOVE_PWM, 100);
    _motors.setWheels(-dir*lp, dir*rp);
    delay(2);
  }
  _motors.stop(true);
  out.print("ENCODER TURN RESULT: "); out.println(reached ? "target ticks reached" : "timeout/killed");
  out.print("Final ticks L/R = "); out.print(_motors.leftTicks()); out.print(" / "); out.println(_motors.rightTicks());
  out.println("Measure the physical angle. This lets us calibrate turns even if the gyro remains unreliable.");
}

bool TestSuite::calibrateRollDistance(float mm, Print &out) {
  if (mm < 100.0f) mm = 800.0f;
  _motors.stop(false);
  out.println("ENCODER DISTANCE CALIBRATION - motors stay OFF.");
  out.print("1) Put robot on floor at a measured start line. 2) Press ENTER. 3) Roll it STRAIGHT by exactly ");
  out.print(mm, 1); out.println(" mm by hand. 4) Press ENTER again.");
  if (readLine() == "__KILL__") return false;
  _motors.resetEncoders();
  out.println("Now roll the robot; press ENTER when the measured distance is complete.");
  if (readLine() == "__KILL__") return false;
  int32_t lt = labs(_motors.leftTicks());
  int32_t rt = labs(_motors.rightTicks());
  out.print("Captured ticks L/R = "); out.print(lt); out.print(" / "); out.println(rt);
  if (lt < 10 || rt < 10) {
    out.println("Calibration FAIL: too few encoder ticks. Check encoder wiring/pins.");
    return false;
  }
  float tL = (float)lt / mm;
  float tR = (float)rt / mm;
  float meanTicks = 0.5f * ((float)lt + (float)rt);
  float mismatchPct = meanTicks > 1.0f ? 100.0f * fabsf((float)lt - (float)rt) / meanTicks : 100.0f;
  out.print("Candidate ticks/mm L/R = "); out.print(tL, 6); out.print(" / "); out.println(tR, 6);
  out.print("Left/right roll mismatch = "); out.print(mismatchPct, 2); out.println(" %");
  if (mismatchPct > 10.0f) {
    out.println("Calibration REJECTED: wheel counts differ by >10%. Roll straighter, avoid wheel slip/cable drag, and repeat.");
    return false;
  }
  _cal.ticksPerMmLeft = tL;
  _cal.ticksPerMmRight = tR;
  out.print("Accepted ticks/mm L/R = "); out.print(_cal.ticksPerMmLeft, 6); out.print(" / "); out.println(_cal.ticksPerMmRight, 6);
  return true;
}

bool TestSuite::calibrateWalls(Print &out) {
  out.println("VL53L0X CLOSE-ONLY WALL CALIBRATION");
  out.println("For each sensor, place a real maze wall at its normal PRESENT/CLOSE distance.");
  out.print("OPEN distance is not calibrated. Wall threshold = CLOSE + ");
  out.print(WALL_CLOSE_MARGIN_PCT);
  out.println("% margin.");

  for (uint8_t i = 0; i < 4; ++i) {
    out.print("Sensor index "); out.print(i);
    out.println(": place wall at normal PRESENT/CLOSE distance, then press ENTER.");
    if (readLine() == "__KILL__") return false;

    uint16_t present = sampleSensor(i, 35);
    out.print("close average = "); out.println(present);

    if (present >= 4000 || present < 20) {
      out.println("Sensor calibration FAIL: CLOSE wall did not produce a valid range.");
      return false;
    }

    uint32_t t = (uint32_t)present * (100u + WALL_CLOSE_MARGIN_PCT) / 100u;
    if (t > 500u) t = 500u;
    _cal.wallThresholdMm[i] = (uint16_t)t;

    out.print("threshold["); out.print(i); out.print("] = ");
    out.print(_cal.wallThresholdMm[i]);
    out.println(" mm (close + percentage margin)");

    // The side CLOSE value measured while physically centered is also the
    // centering reference for the wall-follow controller.
    if (i == SensorMap::LEFT) _cal.sideTargetLeftMm = present;
    if (i == SensorMap::RIGHT) _cal.sideTargetRightMm = present;
  }

  out.println("Close-only wall calibration complete. Run 'savecal' to persist.");
  return true;
}

void TestSuite::tunePidCommand(const String &line, Print &out) {
  float a = _cal.straightKp, b = _cal.wallKp, c = _cal.gyroTurnKp;
  int n = sscanf(line.c_str(), "pid %f %f %f", &a, &b, &c);
  if (n < 2) {
    out.println("Usage: pid <straightKp> <wallKp> [gyroTurnKp]");
    return;
  }
  _cal.straightKp = a;
  _cal.wallKp = b;
  if (n >= 3) _cal.gyroTurnKp = c;
  out.print("PID updated: straight="); out.print(_cal.straightKp, 3);
  out.print(" wall="); out.print(_cal.wallKp, 3);
  out.print(" gyroTurn="); out.println(_cal.gyroTurnKp, 3);
}

bool TestSuite::guidedCalibration() {
  _console = &Serial;
  Serial.println("\n=== MicroMaze 3 GUIDED CALIBRATION ===");
  Serial.println("Use USB Serial only. Do not use Bluetooth/Wi-Fi.");
  Serial.println("This mode calibrates gyro, encoders/distance, ToF wall thresholds, and verifies motors/encoders.");

  if (!_imu.present() || _tofArray.presentMask() != 0x0F) {
    Serial.println("Calibration cannot be complete: MPU6050 or one/more VL53L0X sensors are missing.");
  }

  if (!waitEnter("Keep robot still and press ENTER for gyro calibration.")) return false;
  _cal.gyroBiasZDps = _imu.calibrateGyroYaw(Serial);

  testToF(Serial, 4);
  Serial.println("If the physical sensor/index mapping above is wrong, fix SensorMap in include/Config.h BEFORE final inspection.");

  if (!calibrateRollDistance(800.0f, Serial)) return false;
  if (!calibrateWalls(Serial)) return false;

  testMotorAndEncoder(Serial);
  if (MotorSystem::emergencyLatched()) return false;

  bool saved = _calStore.save(_cal);
  Serial.print("Calibration save to ESP32 NVS: "); Serial.println(saved ? "PASS" : "FAIL");
  if (saved) {
    Serial.println("Recommended next verification: Debug mode -> 'distance 800', 'cell', 'turn 90', 'turn -90'.");
  }
  return saved;
}

void TestSuite::printStatus(Print &out) {
  out.println("\n=== MM3 STATUS ===");
  out.print("PCF8574 0x20: "); out.println(_io.present() ? "OK" : "MISSING");
  out.print("AT24C1024 0x50/0x51: "); out.println(_eeprom.present() ? "OK" : "MISSING");
  out.print("MPU6050 0x68: "); out.println(_imu.present() ? "OK" : "MISSING");
  out.print("VL53L0X mask: 0x"); out.println(_tofArray.presentMask(), HEX);
  out.print("ticks/mm L/R: "); out.print(_cal.ticksPerMmLeft, 6); out.print(" / "); out.println(_cal.ticksPerMmRight, 6);
  out.print("gyro physical-yaw bias Z dps: "); out.println(_cal.gyroBiasZDps, 5);
  out.print("wall thresholds: ");
  for (uint8_t i = 0; i < 4; ++i) { out.print(_cal.wallThresholdMm[i]); out.print(i == 3 ? '\n' : ' '); }
  out.print("side targets L/R: "); out.print(_cal.sideTargetLeftMm); out.print(" / "); out.println(_cal.sideTargetRightMm);
  out.print("PID straight/wall/turn: "); out.print(_cal.straightKp, 3); out.print(" / "); out.print(_cal.wallKp, 3); out.print(" / "); out.println(_cal.gyroTurnKp, 3);
}

void TestSuite::printHelp(Print &out) {
  out.println("\n=== DEBUG COMMANDS ===");
  out.println("help                 - this list");
  out.println("status               - hardware + calibration status");
  out.println("i2c                  - scan main I2C bus");
  out.println("io                   - DIP/buttons/LEDs/buzzer test");
  out.println("tof                  - 4 VL53L0X live readings");
  out.println("imu                  - MPU6050 readings");
  out.println("eeprom               - preserved external EEPROM R/W self-test");
  out.println("motor                - TB6612 A/B forward/reverse + encoder test (lift robot)");
  out.println("encoder              - manual encoder count test");
  out.println("distance <mm> [pwm]  - closed-loop distance test");
  out.println("cell                 - 192 mm one-cell (lattice pitch) movement test");
  out.println("turn <deg>           - gyro yaw turn using MPU Z axis + spike filtering");
  out.println("gyro_angle [deg]     - legacy manual gyro integration test");
  out.println("gyro_axes            - live X/Y/Z gyro values while rotating robot flat");
  out.println("gyro_axis90          - AUTO 90-degree yaw-axis diagnostic (recommended)");
  out.println("turn_diag <deg>      - LOW-PWM safe filtered-Z gyro turn diagnostic");
  out.println("turn_ticks <ticks>   - safe encoder-only pivot; +left, -right");
  out.println("front_align [mm]     - dock/square to front wall; default uses Config target");
  out.println("set_ticks <L> <R>    - directly set encoder ticks/mm calibration");
  out.println("walls                - print current L/F/R distance + wall decision for 5 s");
  out.println("cal_roll <mm>        - hand-roll encoder distance calibration (192 mm is one cell pitch)");
  out.println("cal_wall             - CLOSE-only wall calibration + percentage margin");
  out.println("cal_imu              - robust stationary physical-yaw (MPU Z) bias calibration");
  out.println("pid <s> <w> [g]      - set straight/wall/gyro-turn gains");
  out.println("savecal              - save calibration to ESP32 NVS");
  out.println("clearcal             - erase calibration NVS");
  out.println("deadend_test         - simple home route: first dead end then retrace to start");
  out.println("dfs_test             - conservative generic DFS test");
  out.println("dfs_fast             - adaptive FAST DFS diagnostic");
  out.println("explore              - competition exploration: start -> centre goal -> return -> save shortest path");
  out.println("fast_run             - run saved shortest path from START to centre at FAST_PWM");
  out.println("fast_return          - run saved shortest path to centre and autonomously return to START");
  out.println("maze                 - dump stored maze + optimal path");
  out.println("clearmaze            - clear external EEPROM maze/path header");
  out.println("all_safe             - I2C + IO + ToF + IMU + EEPROM tests (no wheel drive)");
  out.println("wireless             - print final wireless-compliance design statement");
  out.println("exit                 - leave Debug mode");
  out.println("Key2 interrupt can terminate the current mode/run.");
}

void TestSuite::debugLoop(Stream &console) {
  _console = &console;
  console.println("\n=== DEBUG MODE (DIP 100) ===");
  printHelp(console);

  while (true) {
    console.print("mm3> ");
    // Wait for a complete command. The selected console can be USB Serial,
    // the development WebTerminal, or a stream that multiplexes both.
    String line = readLine();
    if (line == "__KILL__" || MotorSystem::emergencyLatched()) break;
    line.trim();
    if (!line.length()) continue;

    if (line == "help") printHelp(console);
    else if (line == "status") printStatus(console);
    else if (line == "i2c") scanMainI2C(console);
    else if (line == "io") testIO(console);
    else if (line == "tof") testToF(console, 5);
    else if (line == "imu") testImu(console);
    else if (line == "eeprom") _eeprom.selfTest(console);
    else if (line == "motor") testMotorAndEncoder(console);
    else if (line == "encoder") testEncoderManual(console);
    else if (line == "cell") testDistanceMove(CELL_MM, SLOW_PWM, console);
    else if (line.startsWith("distance ")) {
      float mm = 0; int pwm = SLOW_PWM;
      int n = sscanf(line.c_str(), "distance %f %d", &mm, &pwm);
      if (n >= 1) testDistanceMove(mm, pwm, console);
      else console.println("Usage: distance <mm> [pwm]");
    }
    else if (line == "gyro_axes") testGyroAxes(console);
    else if (line == "gyro_axis90") testGyroAxis90(console);
    else if (line.startsWith("gyro_angle")) {
      float deg = 90.0f;
      sscanf(line.c_str(), "gyro_angle %f", &deg);
      testGyroManualAngle(deg, console);
    }
    else if (line.startsWith("turn_diag ")) {
      float deg = 0;
      if (sscanf(line.c_str(), "turn_diag %f", &deg) == 1) testTurnDiagnostic(deg, console);
      else console.println("Usage: turn_diag <degrees>");
    }
    else if (line.startsWith("turn_ticks ")) {
      long ticks = 0;
      if (sscanf(line.c_str(), "turn_ticks %ld", &ticks) == 1) testEncoderTurnTicks((int32_t)ticks, console);
      else console.println("Usage: turn_ticks <ticks>");
    }
    else if (line.startsWith("front_align")) {
      int mm = FRONT_TURN_TARGET_MM;
      sscanf(line.c_str(), "front_align %d", &mm);
      bool ok = _motion.alignFrontToWall((uint16_t)constrain(mm, 20, 140));
      console.println(ok ? "front_align: PASS" : "front_align: FAIL/timeout");
    }
    else if (line.startsWith("set_ticks ")) {
      float l=0.0f, r=0.0f;
      if (sscanf(line.c_str(), "set_ticks %f %f", &l, &r) == 2 && l > 0.1f && r > 0.1f) {
        _cal.ticksPerMmLeft=l; _cal.ticksPerMmRight=r;
        console.print("ticks/mm set L/R = "); console.print(l,6); console.print(" / "); console.println(r,6);
        console.println("Run savecal to persist.");
      } else console.println("Usage: set_ticks <left_ticks_per_mm> <right_ticks_per_mm>");
    }
    else if (line.startsWith("turn ")) {
      float deg = 0;
      if (sscanf(line.c_str(), "turn %f", &deg) == 1) testTurn(deg, console);
      else console.println("Usage: turn <degrees>");
    }
    else if (line == "walls") {
      uint32_t until = millis() + 5000;
      while ((int32_t)(until - millis()) > 0) {
        console.print("L="); console.print(_motion.distanceLeft()); console.print(_motion.wallLeft() ? "[W] " : "[ ] ");
        console.print("F="); console.print(_motion.distanceFront()); console.print(_motion.wallFront() ? "[W] " : "[ ] ");
        console.print("R="); console.print(_motion.distanceRight()); console.println(_motion.wallRight() ? "[W]" : "[ ]");
        delay(150);
      }
    }
    else if (line.startsWith("cal_roll")) {
      float mm = CELL_MM;
      sscanf(line.c_str(), "cal_roll %f", &mm);
      calibrateRollDistance(mm, console);
    }
    else if (line == "cal_wall") calibrateWalls(console);
    else if (line == "cal_imu") _cal.gyroBiasZDps = _imu.calibrateGyroYaw(console);
    else if (line.startsWith("pid ")) tunePidCommand(line, console);
    else if (line == "savecal") console.println(_calStore.save(_cal) ? "Calibration saved." : "Calibration save FAILED.");
    else if (line == "clearcal") { _calStore.clear(); console.println("Calibration NVS cleared; reboot or recalibrate."); }
    else if (line == "deadend_test") _maze.deadEndReturnTest(SLOW_PWM, console);
    else if (line == "dfs_test") _maze.homeDfsTest(SLOW_PWM, console);
    else if (line == "dfs_fast") _maze.homeDfsTest(FAST_PWM, console);
    else if (line == "explore") {
      console.println("COMPETITION GOAL: any centre cell (7,7), (7,8), (8,7), or (8,8). No endpoint entry is required.");
      console.println("Place robot in START cell (0,0), facing the only maze exit, then run.");
      _maze.explorationRun(SLOW_PWM, console);
    }
    else if (line == "fast_run") _maze.storedRun(FAST_PWM, false, FAST_PWM, console);
    else if (line == "fast_return") _maze.storedRun(FAST_PWM, true, FAST_PWM, console);
    else if (line == "maze") _maze.dumpStored(console);
    else if (line == "clearmaze") _maze.clearStored(console);
    else if (line == "all_safe") {
      scanMainI2C(console);
      testIO(console);
      testToF(console, 2);
      testImu(console);
      _eeprom.selfTest(console);
    }
    else if (line == "wireless") {
#if MM3_DEV_WIFI
      console.println("DEVELOPMENT BUILD: ESP32-S3 Wi-Fi AP/browser terminal is ENABLED.");
      console.println("This build is for bench/calibration testing only and is NOT competition compliant.");
      console.println("Flash environment 'esp32-s3-competition' before inspection/competition.");
#else
      console.println("COMPETITION BUILD: Wi-Fi/BLE are not initialized and are explicitly stopped/deinitialized at boot.");
      console.println("Autonomous modes require no PC or wireless link.");
#endif
    }
    else if (line == "exit") break;
    else console.println("Unknown command. Type 'help'.");
  }

  _motors.stop(true);
  _console = &Serial;
}

} // namespace MM3
