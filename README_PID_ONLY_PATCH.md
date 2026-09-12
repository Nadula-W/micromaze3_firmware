# v32 + PID-only patch

Baseline: exact `micromaze3_firmware_fixed_v32_dfs_start_stop.zip`.

Only the steering PID semantics were changed.

- Terminal command remains `pid <a> <b> <c>`.
- The three values now directly mean `Kp`, `Ki`, `Kd`.
- `cell`, `distance`, and v32 DFS drive use one discrete PID steering controller.
- The controller error combines encoder left/right progress mismatch and the raw side-wall centering error when side walls are available.
- No separate straight/wall gains remain.
- 378-tick DFS turns are unchanged.
- The gyro diagnostic turn keeps its old fixed proportional gain internally and is not controlled by this drive PID.
- No ToF driver, maze/DFS, wall thresholds, ticks/mm, motor pins, front alignment, or EEPROM logic was changed.

IMPORTANT after flashing: the three old v32 NVS gain slots still contain whatever straight/wall/turn values you last saved. Before moving the robot, run:

    pid 1.8 0 0.03

Then test `cell`. Only after the motion is good, run `savecal`.
