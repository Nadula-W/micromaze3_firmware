# MicroMaze III firmware v36 — close-only wall logic (16x16)

This build is based on the v32d 16x16 exploration/fast-run firmware.

## What changed

- `cal_wall` now records **only PRESENT/CLOSE** for each ToF sensor.
- Wall threshold is `close + 50%` (`WALL_CLOSE_MARGIN_PCT` in `Config.h`).
- OPEN calibration is no longer used for threshold calculation.
- While traversing the latter half of every cell, LEFT and RIGHT are sampled continuously.
- A side is latched as OPEN only after **3 consecutive fresh samples are NOT CLOSE**.
- At a stopped cell, a path is accepted as OPEN only when CLOSE is absent for 3 consecutive fresh checks.
- One CLOSE reading resets the no-close streak, biasing decisions toward safety.
- Existing v32d turn preparation, 192 mm cell motion, 378-tick turns, PID/NVS calibration, explore/fast-run logic remain.
- Maze is **16x16**. Competition goal is any center cell: `(7,7)`, `(7,8)`, `(8,7)`, `(8,8)`.

## Trial workflow

1. `status`
2. `cal_wall` — for each sensor, show only a real wall at normal close distance.
3. `savecal`
4. `walls` — verify wall/open decisions.
5. `cell` and `turn_ticks 378` / `turn_ticks -378`
6. `clearmaze`
7. `explore`
8. `maze`
9. Put robot back at START, same heading, then `fast_run`.

If the close margin needs adjustment later, change only `WALL_CLOSE_MARGIN_PCT` in `include/Config.h`.



## v37 exploration reliability patch
- `explore` always resets to a fresh 16x16 map; it no longer loads stale walls/path from a previous maze.
- Cell wall decisions remain CLOSE-only. No OPEN-distance averaging is used.
- A wall now needs 2 CLOSE votes out of 3 fresh checks; one bad low ToF sample cannot create a fake dead end.
- Side openings latched continuously during movement are preserved.
- Front-reference stopping now requires 3 confirming samples.
- Exploration prints logical `(x,y)`, heading, and chosen next direction every step so unexpected 180s can be diagnosed immediately.
- Saved PID/ticks/mm/wall calibration in ESP32 NVS are not cleared by this firmware update.
