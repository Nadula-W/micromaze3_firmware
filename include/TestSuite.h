#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Hardware.h"
#include "Motion.h"
#include "Maze.h"
#include "CalibrationStore.h"

namespace MM3 {

class TestSuite {
public:
  TestSuite(TwoWire &mainWire,
            Pcf8574IO &io,
            ExternalEEPROM &eeprom,
            DistanceArray &tofArray,
            SensorHub &sensorHub,
            Imu6050 &imu,
            MotorSystem &motors,
            MotionController &motion,
            MazeNavigator &maze,
            CalibrationStore &calStore,
            CalibrationData &cal)
      : _wire(mainWire), _io(io), _eeprom(eeprom), _tofArray(tofArray),
        _sensorHub(sensorHub), _imu(imu), _motors(motors), _motion(motion),
        _maze(maze), _calStore(calStore), _cal(cal) {}

  bool guidedCalibration();
  void debugLoop(Stream &console);
  void debugLoop() { debugLoop(Serial); }
  void printStatus(Print &out);
  void printHelp(Print &out);

private:
  TwoWire &_wire;
  Pcf8574IO &_io;
  ExternalEEPROM &_eeprom;
  DistanceArray &_tofArray;
  SensorHub &_sensorHub;
  Imu6050 &_imu;
  MotorSystem &_motors;
  MotionController &_motion;
  MazeNavigator &_maze;
  CalibrationStore &_calStore;
  CalibrationData &_cal;
  Stream *_console = &Serial;

  String readLine(uint32_t timeoutMs = 0);
  bool waitEnter(const char *prompt, uint32_t timeoutMs = 0);
  float askFloat(const char *prompt, float defaultValue = NAN);
  uint16_t sampleSensor(uint8_t index, uint16_t samples = 30);

  void scanMainI2C(Print &out);
  void testIO(Print &out);
  void testToF(Print &out, uint16_t seconds = 3);
  void testImu(Print &out);
  void testMotorAndEncoder(Print &out);
  void testEncoderManual(Print &out);
  void testDistanceMove(float mm, int pwm, Print &out);
  void testTurn(float deg, Print &out);
  void testGyroManualAngle(float expectedDeg, Print &out);
  void testGyroAxes(Print &out);
  void testGyroAxis90(Print &out);
  void testTurnDiagnostic(float deg, Print &out);
  void testEncoderTurnTicks(int32_t signedTicks, Print &out);
  bool calibrateRollDistance(float mm, Print &out);
  bool calibrateWalls(Print &out);
  void tunePidCommand(const String &line, Print &out);
  void tuneFrontPidCommand(const String &line, bool square, Print &out);
};

} // namespace MM3
