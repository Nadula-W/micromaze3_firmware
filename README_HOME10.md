# MM3 v39 Home 10x10 Test Build

Based on v39 exact standalone ToF acquisition.

Home test configuration:
- Maze size: 10 x 10
- Start: (0,0)
- Start heading: North
- Goal: (6,6)
- North = y+1, East = x+1, South = y-1, West = x-1
- Maze storage version bumped so an old 16x16 EEPROM map is not reused.

For `dfs_test`, place the robot at (0,0), facing North, with the maze
extending forward and to its right. DFS uses the same 10x10 bounds as movement,
explores reachable cells, and returns to START. Its action limit allows all
100 cells to be explored and backtracked (198 moves plus the final START check).
It does not stop at the goal (6,6).

Front alignment now uses separate distance and squaring PID controllers, with
filtered derivatives, bounded integrals, and short PWM pulses for small errors.
The target remains 80 mm; alignment must settle within 15 mm distance (65-95 mm)
and 6 mm
front-sensor difference for 120 ms. The timeout is 5 seconds.
After uploading, test `front_align 80` facing a nearby flat wall before `dfs_test`.
The initial gains need verification on the robot. In the terminal:

```text
front_pid                         # view distance gains
front_pid 1.25 0.20 0.08           # set distance Kp Ki Kd
front_square_pid                   # view squaring gains
front_square_pid 0.90 0.15 0.05    # set squaring Kp Ki Kd
front_align 80                     # test
savecal                            # keep gains after reboot
```

Send commands without the explanatory `#` comments. All three gains are required
when setting values; each must be finite and between 0 and 100. Changes apply to
the next alignment, including DFS alignment. Existing saved calibration loads
with the front defaults from `include/Config.h`. The terminal `pid` command still
controls straight driving only. Integral and derivative gains use seconds.

Before a fresh home exploration:
1. `status`
2. `tof`
3. `walls`
4. `clearmaze`
5. `explore`
6. `maze`
7. reposition at START facing North, then select Fast mode and press Key1.

`explore` (or Exploration DIP mode `111`) starts at (0,0), facing North,
targets only (6,6), returns to (0,0), and saves the confirmed shortest route.
Goal coordinates are `MAZE_GOAL_X/Y` in `include/Config.h`. Storage version 3
rejects maps/routes saved for the previous goal; run exploration before Fast mode.

For the competition, restore `MAZE_N = 16` and the normal four center goal cells.
