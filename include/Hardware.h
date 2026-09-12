#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Config.h"
#include "RobotTypes.h"

namespace MM3 {

class Pcf8574IO {
public:
  bool begin(TwoWire &bus);
  uint8_t readRaw();
  uint8_t modeBits();
  bool key1();
  bool key2();
  void setLed1(bool on);
  void setLed2(bool on);
  void setBuzzer(bool on);
  void beep(uint16_t ms = 80);
  bool present() const { return _present; }

private:
  TwoWire *_bus = nullptr;
  uint8_t _shadow = 0x1F; // P1..P5 released HIGH as inputs; P6..P8 LOW/off
  bool _present = false;
  bool writeShadow();
  void setOutputBit(uint8_t bit, bool on);
};

class ExternalEEPROM {
public:
  bool begin(TwoWire &bus);
  bool present() const { return _present; }
  void protect(bool readOnly);
  bool readBytes(uint32_t address, uint8_t *data, size_t len);
  bool writeBytes(uint32_t address, const uint8_t *data, size_t len);
  bool selfTest(Print &out);

private:
  TwoWire *_bus = nullptr;
  bool _present = false;
  uint8_t deviceFor(uint32_t address) const;
  uint16_t wordAddress(uint32_t address) const;
  bool waitReady(uint8_t dev, uint16_t timeoutMs = 20);
};

class DistanceArray {
public:
  bool begin(TwoWire &bus);
  bool allPresent() const { return _allPresent; }
  void readAll(SensorSnapshot &out);
  uint8_t presentMask() const { return _presentMask; }

private:
  TwoWire *_bus = nullptr;
  Adafruit_VL53L0X _sensor[4];
  bool _ok[4] = {false, false, false, false};
  bool _allPresent = false;
  uint8_t _presentMask = 0;
};

class SensorHub {
public:
  bool begin(DistanceArray &array);
  SensorSnapshot snapshot();
  void stop();

private:
  DistanceArray *_array = nullptr;
  TaskHandle_t _task = nullptr;
  SensorSnapshot _snapshot;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  static void taskThunk(void *arg);
  void taskLoop();
};

class Imu6050 {
public:
  bool begin(TwoWire &bus);
  bool present() const { return _present; }
  float readGyroZDps(float bias = 0.0f);
  bool readGyroXYZDps(float &x, float &y, float &z);
  // Flat-robot tests show gravity on MPU Z, so physical yaw is MPU6050 Z axis.
  bool readGyroYawDps(float &yawDps, float bias = 0.0f);
  bool readGyroYawFilteredDps(float &yawDps, float bias = 0.0f);
  float calibrateGyroYaw(Print &out, uint16_t samples = 201);
  float calibrateGyroZ(Print &out, uint16_t samples = 800);
  void printOne(Print &out, float bias = 0.0f);

private:
  Adafruit_MPU6050 _mpu;
  bool _present = false;
};

class MotorSystem {
public:
  bool begin();
  void standby(bool enabled);
  void stop(bool brake = false);
  void setMotorA(int pwm);
  void setMotorB(int pwm);
  void setWheels(int leftPwm, int rightPwm);

  void resetEncoders();
  int32_t encoderA() const;
  int32_t encoderB() const;
  int32_t leftTicks() const;
  int32_t rightTicks() const;

  static void emergencyStandbyOffFromISR();
  static void clearEmergencyLatch();
  static bool emergencyLatched();

private:
  static volatile int32_t _encA;
  static volatile int32_t _encB;
  static volatile uint8_t _prevA;
  static volatile uint8_t _prevB;
  static portMUX_TYPE _encMux;
  static volatile bool _emergencyLatched;

  static void IRAM_ATTR isrEncA();
  static void IRAM_ATTR isrEncB();
  static void IRAM_ATTR updateEncoderA();
  static void IRAM_ATTR updateEncoderB();

  void writeMotor(bool motorA, int pwm);
  void writePwm(uint8_t pin, uint8_t channel, uint8_t duty);
};

} // namespace MM3
