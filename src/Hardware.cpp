#include "Hardware.h"
#include <esp_arduino_version.h>
#include <driver/gpio.h>
#include <math.h>

namespace MM3 {

// ---------------- PCF8574 ----------------

bool Pcf8574IO::begin(TwoWire &bus) {
  _bus = &bus;
  // Real PCB: P1..P5 have external pull-down resistors and the
  // DIP switches / push buttons connect them to 3.3 V when active.
  // Therefore inputs are ACTIVE HIGH. PCF8574 input-role pins must still
  // be written HIGH (released) because the device is quasi-bidirectional.
  // P6..P8 drive LED1/LED2/buzzer and are ACTIVE HIGH on the schematic.
  // Start with inputs released HIGH and outputs OFF (LOW).
  _shadow = 0x1F;
  _bus->beginTransmission(Addr::PCF8574);
  _present = (_bus->endTransmission() == 0);
  if (_present) writeShadow();
  return _present;
}

bool Pcf8574IO::writeShadow() {
  if (!_bus || !_present) return false;
  _bus->beginTransmission(Addr::PCF8574);
  _bus->write(_shadow);
  return _bus->endTransmission() == 0;
}

uint8_t Pcf8574IO::readRaw() {
  if (!_bus || !_present) return 0;
  // Keep quasi-bidirectional input pins released HIGH before reading.
  _shadow |= 0x1F;
  writeShadow();
  if (_bus->requestFrom((int)Addr::PCF8574, 1) != 1) return 0;
  return _bus->read();
}

uint8_t Pcf8574IO::modeBits() {
  const uint8_t r = readRaw();
  // Real PCB inputs are ACTIVE HIGH: external 1 kΩ pull-downs hold them
  // LOW, and an ON switch connects the corresponding line to 3.3 V.
  const uint8_t sw1 = (r & (1u << 0)) ? 1u : 0u; // P1
  const uint8_t sw2 = (r & (1u << 1)) ? 1u : 0u; // P2
  const uint8_t sw3 = (r & (1u << 2)) ? 1u : 0u; // P3
  return (sw3 << 2) | (sw2 << 1) | sw1;
}

// Push buttons are ACTIVE HIGH on the real PCB.
bool Pcf8574IO::key1() { return (readRaw() & (1u << 3)) != 0; }
bool Pcf8574IO::key2() { return (readRaw() & (1u << 4)) != 0; }

void Pcf8574IO::setOutputBit(uint8_t bit, bool on) {
  // LED1, LED2 and buzzer are ACTIVE HIGH on this PCB.
  if (on) _shadow |= (1u << bit);
  else _shadow &= ~(1u << bit);
  // Always release input-role pins HIGH.
  _shadow |= 0x1F;
  writeShadow();
}

void Pcf8574IO::setLed1(bool on) { setOutputBit(5, on); }
void Pcf8574IO::setLed2(bool on) { setOutputBit(6, on); }
void Pcf8574IO::setBuzzer(bool on) { setOutputBit(7, on); }
void Pcf8574IO::beep(uint16_t ms) {
  setBuzzer(true);
  delay(ms);
  setBuzzer(false);
}

// ---------------- External AT24C1024 EEPROM ----------------

bool ExternalEEPROM::begin(TwoWire &bus) {
  _bus = &bus;
  pinMode(Pins::EEPROM_WP, OUTPUT);
  protect(true);

  _bus->beginTransmission(Addr::EEPROM_BLOCK0);
  bool p0 = (_bus->endTransmission() == 0);
  _bus->beginTransmission(Addr::EEPROM_BLOCK1);
  bool p1 = (_bus->endTransmission() == 0);
  _present = p0 && p1;
  return _present;
}

void ExternalEEPROM::protect(bool readOnly) {
  digitalWrite(Pins::EEPROM_WP, readOnly ? HIGH : LOW);
  delayMicroseconds(10);
}

uint8_t ExternalEEPROM::deviceFor(uint32_t address) const {
  return (address & 0x10000UL) ? Addr::EEPROM_BLOCK1 : Addr::EEPROM_BLOCK0;
}

uint16_t ExternalEEPROM::wordAddress(uint32_t address) const {
  return static_cast<uint16_t>(address & 0xFFFFUL);
}

bool ExternalEEPROM::waitReady(uint8_t dev, uint16_t timeoutMs) {
  uint32_t start = millis();
  do {
    _bus->beginTransmission(dev);
    if (_bus->endTransmission() == 0) return true;
    delay(1);
  } while (millis() - start < timeoutMs);
  return false;
}

bool ExternalEEPROM::readBytes(uint32_t address, uint8_t *data, size_t len) {
  if (!_present || !_bus || address + len > 131072UL) return false;
  size_t done = 0;
  while (done < len) {
    uint32_t a = address + done;
    uint8_t dev = deviceFor(a);
    uint16_t wa = wordAddress(a);
    size_t blockRemain = 65536UL - wa;
    size_t chunk = len - done;
    if (chunk > 28) chunk = 28; // conservative Wire buffer chunk
    if (chunk > blockRemain) chunk = blockRemain;

    _bus->beginTransmission(dev);
    _bus->write((uint8_t)(wa >> 8));
    _bus->write((uint8_t)(wa & 0xFF));
    if (_bus->endTransmission(false) != 0) return false;
    size_t got = _bus->requestFrom((int)dev, (int)chunk);
    if (got != chunk) return false;
    for (size_t i = 0; i < chunk; ++i) data[done + i] = _bus->read();
    done += chunk;
  }
  return true;
}

bool ExternalEEPROM::writeBytes(uint32_t address, const uint8_t *data, size_t len) {
  if (!_present || !_bus || address + len > 131072UL) return false;
  protect(false);
  size_t done = 0;
  bool ok = true;
  while (done < len) {
    uint32_t a = address + done;
    uint8_t dev = deviceFor(a);
    uint16_t wa = wordAddress(a);
    size_t blockRemain = 65536UL - wa;
    size_t pageRemain = 256UL - (wa & 0xFFu);
    size_t chunk = len - done;
    if (chunk > 28) chunk = 28;
    if (chunk > pageRemain) chunk = pageRemain;
    if (chunk > blockRemain) chunk = blockRemain;

    _bus->beginTransmission(dev);
    _bus->write((uint8_t)(wa >> 8));
    _bus->write((uint8_t)(wa & 0xFF));
    for (size_t i = 0; i < chunk; ++i) _bus->write(data[done + i]);
    if (_bus->endTransmission() != 0 || !waitReady(dev)) {
      ok = false;
      break;
    }
    done += chunk;
  }
  protect(true);
  return ok;
}

bool ExternalEEPROM::selfTest(Print &out) {
  if (!_present) {
    out.println("EEPROM: NOT PRESENT");
    return false;
  }
  constexpr uint32_t TEST_ADDR = 0x1FFE0UL;
  uint8_t original[16];
  uint8_t pattern[16];
  uint8_t verify[16];
  for (uint8_t i = 0; i < sizeof(pattern); ++i) pattern[i] = (uint8_t)(0xA5 ^ (i * 17));

  if (!readBytes(TEST_ADDR, original, sizeof(original))) return false;
  if (!writeBytes(TEST_ADDR, pattern, sizeof(pattern))) return false;
  if (!readBytes(TEST_ADDR, verify, sizeof(verify))) return false;
  bool ok = memcmp(pattern, verify, sizeof(pattern)) == 0;
  bool restored = writeBytes(TEST_ADDR, original, sizeof(original));
  out.print("EEPROM preserved self-test: ");
  out.println(ok && restored ? "PASS" : "FAIL");
  return ok && restored;
}

// ---------------- VL53L0X array ----------------

bool DistanceArray::begin(TwoWire &bus) {
  _bus = &bus;
  _presentMask = 0;

  // All VL53L0X devices power up at 0x29, so hold every sensor in reset first.
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(Pins::XSHUT[i], OUTPUT);
    digitalWrite(Pins::XSHUT[i], LOW);
    _ok[i] = false;
  }
  delay(100);

  // Bring the sensors up one at a time and assign the exact addresses proven by
  // the standalone Adafruit test: S1..S4 = 0x30..0x33.
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(Pins::XSHUT[i], HIGH);
    delay(100);

    // Match the known-good standalone test exactly: start at the factory 0x29
    // address on the dedicated ToF I2C bus, then move this device to 0x30+i.
    if (_sensor[i].begin(Addr::VL53_DEFAULT, false, _bus)) {
      _sensor[i].setAddress(Addr::VL53[i]);
      _ok[i] = true;
      _presentMask |= (1u << i);
    } else {
      _ok[i] = false;
      // Prevent a failed/default-address device from colliding with the next one.
      digitalWrite(Pins::XSHUT[i], LOW);
    }
  }

  // IMPORTANT (v38): do NOT start continuous ranging here.
  // The standalone sketch that proved S1..S4 all work uses Adafruit's
  // rangingTest() single-shot path.  v38 intentionally uses that exact
  // measurement method for the navigation sensor service too, because the
  // previous continuous isRangeComplete()/readRangeResult() path repeatedly
  // produced FRONT_RIGHT (S4/index 3) ERR on the real robot.

  _allPresent = (_presentMask == 0x0F);
  return _allPresent;
}

void DistanceArray::readAll(SensorSnapshot &out) {
  // v38 RELIABLE ToF PATH:
  // Match the user's known-good standalone test as closely as possible:
  //   sensor1.rangingTest(&m1, false);
  //   sensor2.rangingTest(&m2, false);
  //   sensor3.rangingTest(&m3, false);
  //   sensor4.rangingTest(&m4, false);
  // One complete S1->S4 scan is published as one coherent snapshot.
  bool anyPresent = false;

  for (uint8_t i = 0; i < 4; ++i) {
    if (!_ok[i]) {
      out.mm[i] = 8190;
      out.valid[i] = false;
      continue;
    }

    VL53L0X_RangingMeasurementData_t m{};
    _sensor[i].rangingTest(&m, false);

    // This is the same primary validity decision used by the proven sketch:
    // RangeStatus == 4 means out/no usable target.  Keep a few impossible
    // sentinel/zero checks so navigation can never mistake corrupt data for a
    // very close wall.
    const uint16_t mm = m.RangeMilliMeter;
    const bool good = (m.RangeStatus != 4) && (mm > 0) && (mm != 0xFFFFu) && (mm < 8190u);

    out.mm[i] = good ? mm : 8190;
    out.valid[i] = good;
    anyPresent = true;
  }

  if (anyPresent) out.stampMs = millis();
}

// ---------------- Core-0 distance sensor task ----------------

bool SensorHub::begin(DistanceArray &array) {
  _array = &array;
  BaseType_t r = xTaskCreatePinnedToCore(taskThunk, "mm3-tof", 4096, this, 2, &_task, 0);
  return r == pdPASS;
}

void SensorHub::stop() {
  if (_task) {
    vTaskDelete(_task);
    _task = nullptr;
  }
}

SensorSnapshot SensorHub::snapshot() {
  SensorSnapshot copy;
  portENTER_CRITICAL(&_mux);
  copy = _snapshot;
  portEXIT_CRITICAL(&_mux);
  return copy;
}

void SensorHub::taskThunk(void *arg) {
  static_cast<SensorHub *>(arg)->taskLoop();
}

void SensorHub::taskLoop() {
  SensorSnapshot s;
  for (;;) {
    // Preserve the previous readings when a sensor does not have a new sample
    // on this pass. DistanceArray::readAll() updates only ready sensors.
    portENTER_CRITICAL(&_mux);
    s = _snapshot;
    portEXIT_CRITICAL(&_mux);

    if (_array) _array->readAll(s);

    portENTER_CRITICAL(&_mux);
    _snapshot = s;
    portEXIT_CRITICAL(&_mux);

    // Always yield; the ToF service must never starve the debug/main task.
    vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// ---------------- MPU6050 ----------------

bool Imu6050::begin(TwoWire &bus) {
  _present = _mpu.begin(Addr::MPU6050, &bus);
  if (!_present) return false;
  _mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  _mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  return true;
}

float Imu6050::readGyroZDps(float bias) {
  if (!_present) return 0.0f;
  sensors_event_t a, g, t;
  _mpu.getEvent(&a, &g, &t);
  return (g.gyro.z * 180.0f / PI) - bias;
}

bool Imu6050::readGyroXYZDps(float &x, float &y, float &z) {
  x = y = z = 0.0f;
  if (!_present) return false;
  sensors_event_t a, g, t;
  _mpu.getEvent(&a, &g, &t);
  x = g.gyro.x * 180.0f / PI;
  y = g.gyro.y * 180.0f / PI;
  z = g.gyro.z * 180.0f / PI;
  // MPU6050 is configured for +/-500 dps. Values far beyond that are not
  // physically valid and indicate a bad/corrupted sample.
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return false;
  if (fabsf(x) > 550.0f || fabsf(y) > 550.0f || fabsf(z) > 550.0f) return false;
  return true;
}

bool Imu6050::readGyroYawDps(float &yawDps, float bias) {
  float x, y, z;
  if (!readGyroXYZDps(x, y, z)) { yawDps = 0.0f; return false; }
  yawDps = z - bias; // robot is flat: Z is the physical yaw axis
  return isfinite(yawDps);
}

bool Imu6050::readGyroYawFilteredDps(float &yawDps, float bias) {
  float a=0, b=0, c=0;
  if (!readGyroYawDps(a, bias)) return false;
  delayMicroseconds(700);
  if (!readGyroYawDps(b, bias)) return false;
  delayMicroseconds(700);
  if (!readGyroYawDps(c, bias)) return false;
  // Median-of-3 removes the isolated bad samples observed on the real robot.
  float m;
  if ((a <= b && b <= c) || (c <= b && b <= a)) m = b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) m = a;
  else m = c;
  yawDps = m;
  return true;
}

float Imu6050::calibrateGyroYaw(Print &out, uint16_t samples) {
  if (!_present) return 0.0f;
  if (samples < 51) samples = 51;
  if (samples > 301) samples = 301;
  float vals[301];
  uint16_t n = 0;
  out.println("Keep robot completely still: calibrating physical YAW gyro (MPU Z axis)...");
  for (uint16_t i=0; i<samples; ++i) {
    float x,y,z;
    if (readGyroXYZDps(x,y,z)) vals[n++] = z;
    delay(3);
  }
  if (n < 25) { out.println("Gyro yaw calibration FAIL: too few valid samples."); return 0.0f; }
  // Small insertion sort; n <= 301. Median is deliberately used instead of
  // mean because real-board diagnostics showed occasional large valid-range spikes.
  for (uint16_t i=1; i<n; ++i) {
    float v=vals[i]; int j=(int)i-1;
    while (j>=0 && vals[j] > v) { vals[j+1]=vals[j]; --j; }
    vals[j+1]=v;
  }
  float bias = vals[n/2];
  out.print("Gyro physical-yaw bias (Z) = "); out.print(bias,5); out.println(" dps");
  out.print("Robust sample middle range = ");
  out.print(vals[n/4],2); out.print(" .. "); out.print(vals[(3*n)/4],2); out.println(" dps");
  return bias;
}

float Imu6050::calibrateGyroZ(Print &out, uint16_t samples) {
  if (!_present) return 0.0f;
  out.println("Keep robot completely still: calibrating MPU6050 gyro Z...");
  double sum = 0.0;
  for (uint16_t i = 0; i < samples; ++i) {
    sensors_event_t a, g, t;
    _mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z * 180.0 / PI;
    delay(2);
  }
  float bias = (float)(sum / samples);
  out.print("Gyro Z bias = ");
  out.print(bias, 5);
  out.println(" dps");
  return bias;
}

void Imu6050::printOne(Print &out, float bias) {
  if (!_present) {
    out.println("MPU6050: NOT PRESENT");
    return;
  }
  sensors_event_t a, g, t;
  _mpu.getEvent(&a, &g, &t);
  out.print("ACC m/s^2 x="); out.print(a.acceleration.x, 3);
  out.print(" y="); out.print(a.acceleration.y, 3);
  out.print(" z="); out.print(a.acceleration.z, 3);
  out.print(" | GYRO dps x="); out.print(g.gyro.x * 180.0f / PI, 3);
  out.print(" y="); out.print(g.gyro.y * 180.0f / PI, 3);
  out.print(" z="); out.println(g.gyro.z * 180.0f / PI - bias, 3);
}

// ---------------- Motors + quadrature encoders ----------------

volatile int32_t MotorSystem::_encA = 0;
volatile int32_t MotorSystem::_encB = 0;
volatile uint8_t MotorSystem::_prevA = 0;
volatile uint8_t MotorSystem::_prevB = 0;
portMUX_TYPE MotorSystem::_encMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool MotorSystem::_emergencyLatched = false;

bool MotorSystem::begin() {
  pinMode(Pins::MOTOR_STBY, OUTPUT);
  pinMode(Pins::MOTOR_A_IN1, OUTPUT);
  pinMode(Pins::MOTOR_A_IN2, OUTPUT);
  pinMode(Pins::MOTOR_B_IN1, OUTPUT);
  pinMode(Pins::MOTOR_B_IN2, OUTPUT);
  pinMode(Pins::MOTOR_A_PWM, OUTPUT);
  pinMode(Pins::MOTOR_B_PWM, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(Pins::MOTOR_A_PWM, PWM_HZ, PWM_BITS);
  ledcAttach(Pins::MOTOR_B_PWM, PWM_HZ, PWM_BITS);
#else
  ledcSetup(0, PWM_HZ, PWM_BITS);
  ledcSetup(1, PWM_HZ, PWM_BITS);
  ledcAttachPin(Pins::MOTOR_A_PWM, 0);
  ledcAttachPin(Pins::MOTOR_B_PWM, 1);
#endif

  pinMode(Pins::ENC_A_C1, INPUT);
  pinMode(Pins::ENC_A_C2, INPUT);
  pinMode(Pins::ENC_B_C1, INPUT);
  pinMode(Pins::ENC_B_C2, INPUT);

  _prevA = (digitalRead(Pins::ENC_A_C1) << 1) | digitalRead(Pins::ENC_A_C2);
  _prevB = (digitalRead(Pins::ENC_B_C1) << 1) | digitalRead(Pins::ENC_B_C2);
  attachInterrupt(digitalPinToInterrupt(Pins::ENC_A_C1), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pins::ENC_A_C2), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pins::ENC_B_C1), isrEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pins::ENC_B_C2), isrEncB, CHANGE);

  standby(false);
  stop(false);
  return true;
}

void MotorSystem::writePwm(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void MotorSystem::standby(bool enabled) {
  digitalWrite(Pins::MOTOR_STBY, enabled ? HIGH : LOW);
}

void MotorSystem::writeMotor(bool motorA, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  uint8_t in1 = motorA ? Pins::MOTOR_A_IN1 : Pins::MOTOR_B_IN1;
  uint8_t in2 = motorA ? Pins::MOTOR_A_IN2 : Pins::MOTOR_B_IN2;
  uint8_t pp = motorA ? Pins::MOTOR_A_PWM : Pins::MOTOR_B_PWM;
  uint8_t ch = motorA ? 0 : 1;

  if (pwm > 0) {
    // TB6612 truth table from the uploaded hardware document: IN1=H, IN2=L => forward.
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  writePwm(pp, ch, (uint8_t)abs(pwm));
}

void MotorSystem::setMotorA(int pwm) {
  if (_emergencyLatched) return;
  standby(true);
  writeMotor(true, pwm);
}
void MotorSystem::setMotorB(int pwm) {
  if (_emergencyLatched) return;
  standby(true);
  writeMotor(false, pwm);
}

void MotorSystem::setWheels(int leftPwm, int rightPwm) {
  if (_emergencyLatched) return;
  if (LEFT_MOTOR_INVERT) leftPwm = -leftPwm;
  if (RIGHT_MOTOR_INVERT) rightPwm = -rightPwm;
  standby(true);
  if (LEFT_MOTOR_IS_A) {
    writeMotor(true, leftPwm);
    writeMotor(false, rightPwm);
  } else {
    writeMotor(false, leftPwm);
    writeMotor(true, rightPwm);
  }
}

void MotorSystem::stop(bool brake) {
  if (brake) {
    standby(true);
    digitalWrite(Pins::MOTOR_A_IN1, HIGH);
    digitalWrite(Pins::MOTOR_A_IN2, HIGH);
    digitalWrite(Pins::MOTOR_B_IN1, HIGH);
    digitalWrite(Pins::MOTOR_B_IN2, HIGH);
    writePwm(Pins::MOTOR_A_PWM, 0, PWM_MAX);
    writePwm(Pins::MOTOR_B_PWM, 1, PWM_MAX);
    delay(20);
  }
  writeMotor(true, 0);
  writeMotor(false, 0);
  standby(false);
}

void MotorSystem::resetEncoders() {
  portENTER_CRITICAL(&_encMux);
  _encA = 0;
  _encB = 0;
  _prevA = (digitalRead(Pins::ENC_A_C1) << 1) | digitalRead(Pins::ENC_A_C2);
  _prevB = (digitalRead(Pins::ENC_B_C1) << 1) | digitalRead(Pins::ENC_B_C2);
  portEXIT_CRITICAL(&_encMux);
}

int32_t MotorSystem::encoderA() const {
  portENTER_CRITICAL(&_encMux);
  int32_t v = _encA;
  portEXIT_CRITICAL(&_encMux);
  return v;
}
int32_t MotorSystem::encoderB() const {
  portENTER_CRITICAL(&_encMux);
  int32_t v = _encB;
  portEXIT_CRITICAL(&_encMux);
  return v;
}
int32_t MotorSystem::leftTicks() const { return LEFT_MOTOR_IS_A ? encoderA() : encoderB(); }
int32_t MotorSystem::rightTicks() const { return LEFT_MOTOR_IS_A ? encoderB() : encoderA(); }

void IRAM_ATTR MotorSystem::isrEncA() { updateEncoderA(); }
void IRAM_ATTR MotorSystem::isrEncB() { updateEncoderB(); }

void IRAM_ATTR MotorSystem::updateEncoderA() {
  uint8_t curr = (gpio_get_level((gpio_num_t)Pins::ENC_A_C1) << 1) |
                 gpio_get_level((gpio_num_t)Pins::ENC_A_C2);
  uint8_t s = (_prevA << 2) | curr;
  int8_t d = 0;
  switch (s) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: d = +1; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: d = -1; break;
    default: break;
  }
  portENTER_CRITICAL_ISR(&_encMux);
  _encA += d;
  _prevA = curr;
  portEXIT_CRITICAL_ISR(&_encMux);
}

void IRAM_ATTR MotorSystem::updateEncoderB() {
  uint8_t curr = (gpio_get_level((gpio_num_t)Pins::ENC_B_C1) << 1) |
                 gpio_get_level((gpio_num_t)Pins::ENC_B_C2);
  uint8_t s = (_prevB << 2) | curr;
  int8_t d = 0;
  switch (s) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: d = +1; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: d = -1; break;
    default: break;
  }
  portENTER_CRITICAL_ISR(&_encMux);
  _encB += d;
  _prevB = curr;
  portEXIT_CRITICAL_ISR(&_encMux);
}

void MotorSystem::emergencyStandbyOffFromISR() {
  _emergencyLatched = true;
  gpio_set_level((gpio_num_t)Pins::MOTOR_STBY, 0);
}

void MotorSystem::clearEmergencyLatch() { _emergencyLatched = false; }
bool MotorSystem::emergencyLatched() { return _emergencyLatched; }

} // namespace MM3
