# Pin, Address and Mode Reference

## ESP32-S3 pins

```text
Main I2C: SDA=8 SCL=9
ToF I2C:  SDA=7 SCL=15
XSHUT:    4,2,16,18
STBY:     42
PWMA:     11
PWMB:     48
AIN1/2:   12,13
BIN1/2:   21,47
Encoder A: 41,40
Encoder B: 39,38
EEPROM WP: 10 (HIGH read-only, LOW read/write)
PCF INT:    14
RGB:        46 (not used)
```

## Main I2C addresses

```text
0x20 PCF8574A
0x50 AT24C1024 block with address bit16=0
0x51 AT24C1024 block with address bit16=1
0x68 MPU6050
```

## VL53L0X second bus

All modules power up at 0x29. Firmware holds all XSHUT low, then powers them one at a time and assigns:

```text
XSHUT1 GPIO4  -> 0x30
XSHUT2 GPIO2  -> 0x31
XSHUT3 GPIO16 -> 0x32
XSHUT4 GPIO18 -> 0x33
```

## TB6612 truth table used

```text
STBY LOW                    -> STOP
STBY HIGH, IN1 L, IN2 L    -> STOP
STBY HIGH, IN1 L, IN2 H    -> REVERSE
STBY HIGH, IN1 H, IN2 L    -> FORWARD
STBY HIGH, IN1 H, IN2 H    -> BRAKE
```

## PCF8574 mapping

```text
P1 = DIP SW1
P2 = DIP SW2
P3 = DIP SW3
P4 = Key1 start
P5 = Key2 kill
P6 = LED1 countdown/running
P7 = LED2 idle/calibration complete
P8 = buzzer
```

The P1/P2/P3-to-SW1/SW2/SW3 correspondence is a firmware assumption because the PDF lists P1-P3 as DIP inputs but does not explicitly label each one with SW1/SW2/SW3. Verify with Debug `io` before inspection.
