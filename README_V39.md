# MicroMaze III v39 — exact standalone ToF acquisition

Base: v37 explore-reliability build.

Only the VL53L0X acquisition path was changed to match the proven standalone 4-sensor sketch:

- ToF I2C bus: 100 kHz (`Wire.begin(SDA=7, SCL=15, 100000)`).
- XSHUT sequence: all LOW, then S1/S2/S3/S4 HIGH one at a time with 100 ms delay.
- Addresses: S1=0x30, S2=0x31, S3=0x32, S4=0x33.
- No continuous-ranging mode.
- Every sensor scan calls `rangingTest(..., false)` in the exact S1 -> S2 -> S3 -> S4 order.
- `RangeStatus == 4` is treated as `Out`, exactly like the standalone test.
- The sensor task waits 100 ms after every complete four-sensor scan, matching the standalone loop pacing.
- A snapshot is published only after all four sensors in that scan have been read.

Maze, exploration, motors, PID/calibration, encoder turns, close-only wall logic, EEPROM and 16x16 center goal are inherited from v37 and were not intentionally changed.

First test after upload:

```
tof
```

You should see standalone-style output such as:

```
S1: 163 mm   S2: 450 mm   S3: 215 mm   S4: 152 mm
```

Then run `walls`, and only after all four sensors behave correctly run `clearmaze` and `explore`.
