# MicroMaze III v38 — reliable ToF single-shot fix

Base: **v37 explore-reliability**.

## What changed

Only the VL53L0X acquisition path was changed.

- Removed `startRangeContinuous()` / `isRangeComplete()` / `readRangeResult()` from the runtime sensor path.
- All four sensors are now read in order with `Adafruit_VL53L0X::rangingTest(..., false)`, matching the standalone sketch that proved S1, S2, S3 and S4 all work on the real robot.
- One full S1→S4 scan is published as a coherent `SensorSnapshot`.
- Sensor mapping remains: 0=FRONT_LEFT, 1=LEFT, 2=RIGHT, 3=FRONT_RIGHT.

## Intentionally NOT changed

- PID/calibration storage or NVS keys
- ticks/mm
- 192 mm cell pitch
- 378-tick encoder turns
- close-only wall logic from v36/v37
- v37 fresh-map exploration and decision reliability changes
- motor mappings / PWM code
- maze goal (16x16 centre cells)
- development/competition Wi-Fi build behavior

## First test

In Debug mode run:

```text
tof
```

With targets in view, all four indices should be capable of returning independent real values. In particular, index 3 must no longer be permanently `ERR` when a white card is 50–100 mm in front of FRONT_RIGHT.

Then run:

```text
walls
```

and confirm real walls/openings classify sensibly before `explore`.

## Fast straight speed test

The existing debug command already lets you test any PWM up to 255:

```text
distance 768 155
distance 768 180
distance 768 200
distance 768 220
distance 768 240
distance 768 255
```

Use a long safe straight lane and Key2 as emergency stop. Do not choose fast-run PWM only from straight speed; it must still stop and turn reliably.
