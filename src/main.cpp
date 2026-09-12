#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include "Config.h"
#if MM3_DEV_WIFI
#include "WebTerminal.h"
#endif
#include "RobotTypes.h"
#include "Hardware.h"
#include "CalibrationStore.h"
#include "Motion.h"
#include "Maze.h"
#include "TestSuite.h"

using namespace MM3;

TwoWire DistanceWire(1);

Pcf8574IO io;
ExternalEEPROM extEeprom;
DistanceArray tofArray;
SensorHub sensorHub;
Imu6050 imu;
MotorSystem motors;
CalibrationStore calStore;
CalibrationData calibration;
MotionController motion(motors, imu, sensorHub, calibration);
MazeMap mazeMap;
MazeStorage mazeStorage(extEeprom);
MazeNavigator mazeNav(mazeMap, mazeStorage, motion);
TestSuite tests(Wire, io, extEeprom, tofArray, sensorHub, imu, motors, motion,
                mazeNav, calStore, calibration);

#if MM3_DEV_WIFI
WebTerminal webTerminal;
#endif

volatile bool gIoInterrupt = false;
volatile bool gRunActive = false;
volatile bool gKillRequested = false;

static void disableBluetoothExplicitly() {
  esp_bt_controller_status_t st = esp_bt_controller_get_status();
  if (st == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    esp_bt_controller_disable();
    st = esp_bt_controller_get_status();
  }
  if (st == ESP_BT_CONTROLLER_STATUS_INITED) {
    esp_bt_controller_deinit();
  }
}

static void disableWirelessExplicitly() {
  // Competition build: force both Wi-Fi and Bluetooth controllers off.
  esp_wifi_stop();
  esp_wifi_deinit();
  disableBluetoothExplicitly();
}

void IRAM_ATTR ioInterruptISR() {
  gIoInterrupt = true;
}

static void serviceIoInterrupt() {
  if (!gIoInterrupt) return;
  gIoInterrupt = false;
  uint8_t raw = io.readRaw(); // also clears PCF8574 interrupt condition
  if ((raw & (1u << 4)) != 0) gKillRequested = true; // P5 = Key2, active HIGH
}

static bool killCheck() {
  serviceIoInterrupt();
  return gKillRequested || MotorSystem::emergencyLatched();
}

static const char *modeName(RunMode m) {
  switch (m) {
    case RunMode::Calibration: return "Calibration";
    case RunMode::Fast: return "Fast run";
    case RunMode::Slow: return "Slow run";
    case RunMode::FastReturnSameSpeed: return "Fast + return same speed";
    case RunMode::Debug: return "Debug";
    case RunMode::FastReturnSlow: return "Fast + return slow";
    case RunMode::SlowReturn: return "Slow + return slow";
    case RunMode::Exploration: return "Exploration";
  }
  return "Unknown";
}

static bool fiveSecondCountdown() {
  io.setLed2(false);
  io.setLed1(false);
  gKillRequested = false;
  MotorSystem::clearEmergencyLatch();

  uint32_t start = millis();
  bool state = false;
  uint32_t nextBlink = start;
  while (millis() - start < COUNTDOWN_MS) {
    serviceIoInterrupt();
    if (io.key2() || gKillRequested) {
      motors.stop(true);
      io.setLed1(false);
      io.setLed2(true);
      return false;
    }
    if ((int32_t)(millis() - nextBlink) >= 0) {
      state = !state;
      io.setLed1(state);
      nextBlink += 500;
    }
    delay(10);
  }
  io.setLed1(true); // continuously ON while running
  return true;
}

static bool executeMode(RunMode mode) {
  Serial.print("Starting mode: "); Serial.println(modeName(mode));
  switch (mode) {
    case RunMode::Calibration:
      return tests.guidedCalibration();

    case RunMode::Fast:
      return mazeNav.storedRun(FAST_PWM, false, FAST_PWM, Serial);

    case RunMode::Slow:
      return mazeNav.storedRun(SLOW_PWM, false, SLOW_PWM, Serial);

    case RunMode::FastReturnSameSpeed:
      return mazeNav.storedRun(FAST_PWM, true, FAST_PWM, Serial);

    case RunMode::Debug:
#if MM3_DEV_WIFI
      if (webTerminal.active()) tests.debugLoop(webTerminal);
      else tests.debugLoop(Serial);
#else
      tests.debugLoop(Serial);
#endif
      return !killCheck();

    case RunMode::FastReturnSlow:
      return mazeNav.storedRun(FAST_PWM, true, SLOW_PWM, Serial);

    case RunMode::SlowReturn:
      return mazeNav.storedRun(SLOW_PWM, true, SLOW_PWM, Serial);

    case RunMode::Exploration:
      return mazeNav.explorationRun(SLOW_PWM, Serial);
  }
  return false;
}

static void finishMode(bool ok) {
  motors.stop(true);
  gRunActive = false;
  io.setLed1(false);
  io.setLed2(true); // idle / calibration-complete indication
  if (ok) {
    io.beep(70); delay(80); io.beep(70);
  } else {
    io.beep(280);
  }
  gKillRequested = false;
  MotorSystem::clearEmergencyLatch();
  io.readRaw();
  gIoInterrupt = false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(250);
  Serial.println("\nMicroMaze 3 firmware boot");

#if MM3_DEV_WIFI
  // Development build: Wi-Fi is intentionally enabled for a cable-free browser
  // terminal. Bluetooth stays explicitly disabled.
  disableBluetoothExplicitly();
#else
  disableWirelessExplicitly();
#endif

  Wire.begin(Pins::MAIN_SDA, Pins::MAIN_SCL, MAIN_I2C_HZ);
  DistanceWire.begin(Pins::DIST_SDA, Pins::DIST_SCL, DIST_I2C_HZ);

  bool ioOk = io.begin(Wire);
  bool eeOk = extEeprom.begin(Wire);
  bool imuOk = imu.begin(Wire);
  bool tofOk = tofArray.begin(DistanceWire);
  motors.begin();
  sensorHub.begin(tofArray);

  if (!calStore.load(calibration)) {
    calibration = CalibrationData{};
    Serial.println("No valid calibration in ESP32 NVS. Run DIP 000 Calibration mode.");
  } else {
    Serial.println("Calibration loaded from ESP32 NVS.");
  }

#if MM3_DEV_WIFI
  if (!webTerminal.begin("MM3-Robot", "micromaze3")) {
    Serial.println("DEV Wi-Fi terminal FAILED to start; USB Serial still works.");
  } else {
    Serial.print("DEV Wi-Fi terminal: connect to MM3-Robot, then open http://");
    Serial.println(webTerminal.ip());
  }
#endif

  motion.setKillCheck(killCheck);

  pinMode(Pins::IO_INT, INPUT_PULLUP);
  io.readRaw(); // clear any stale PCF interrupt before attach
  attachInterrupt(digitalPinToInterrupt(Pins::IO_INT), ioInterruptISR, FALLING);

  io.setLed1(false);
  io.setLed2(true);
  io.setBuzzer(false);

  Serial.printf("PCF=%s EEPROM=%s MPU=%s ToF4=%s\n",
                ioOk ? "OK" : "FAIL", eeOk ? "OK" : "FAIL",
                imuOk ? "OK" : "FAIL", tofOk ? "OK" : "FAIL");
  Serial.println("Idle. Set DIP mode, then press Key1. Key1 starts after 5 s. Key2 is interrupt kill.");
  Serial.println("NOTE: PCB inputs are active-HIGH on this revision. Key1/Key2 are physical buttons.");
#if MM3_DEV_WIFI
  Serial.println("BUILD: DEVELOPMENT Wi-Fi terminal ENABLED. NOT for competition.");
#else
  Serial.println("BUILD: COMPETITION wireless-disabled.");
#endif
}

void loop() {
  serviceIoInterrupt();

  if (!io.present()) {
    motors.stop(false);
    delay(500);
    return;
  }

  static bool lastKey1 = false;
  static bool key1Armed = true;
  static uint32_t lastChange = 0;
  bool key1 = io.key1();
  if (key1 != lastKey1) {
    lastKey1 = key1;
    lastChange = millis();
  }

  // Re-arm only after a clean release. This prevents one physical press,
  // switch bounce, or a held button from starting the mode repeatedly.
  if (!key1 && millis() - lastChange >= BUTTON_DEBOUNCE_MS) {
    key1Armed = true;
  }

  if (key1 && key1Armed && millis() - lastChange >= BUTTON_DEBOUNCE_MS) {
    key1Armed = false;
    uint8_t bits = io.modeBits() & 0x07;
    RunMode mode = static_cast<RunMode>(bits);
    Serial.print("Key1 pressed. DIP="); Serial.print(bits, BIN);
    Serial.print(" -> "); Serial.println(modeName(mode));

    // Wait for Key1 release before countdown.
    while (io.key1()) delay(5);
    io.readRaw();
    gIoInterrupt = false;

    // If Key2 is physically held, do not start.
    if (io.key2()) {
      Serial.println("Start cancelled: Key2 is pressed.");
    } else if (fiveSecondCountdown()) {
      gRunActive = true;
      gKillRequested = false;
      MotorSystem::clearEmergencyLatch();
      bool ok = executeMode(mode);
      finishMode(ok && !killCheck());
    }
  }

  delay(10);
}
