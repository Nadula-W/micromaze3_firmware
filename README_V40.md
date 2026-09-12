# v40 Home 10x10 - ToF zero-range fix

Based on v39 home 10x10, goal (6,6).

Only ToF acquisition was changed:
- four distinct Adafruit_VL53L0X objects, matching the proven standalone sketch
- S1 -> S2 -> S3 -> S4 rangingTest() order
- 100 kHz distance I2C
- a 0 mm reading is treated as invalid, never as a wall
- one retry on an invalid/zero single-shot

This prevents S4=0 from being averaged with S1 (for example 80 and 0 becoming a false front distance of 40 mm).
PID, calibration storage, motor code, turns, maze logic and home goal (6,6) are unchanged.
