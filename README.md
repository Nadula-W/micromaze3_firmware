# MicroMaze III firmware v32 — DFS start-boundary stop fix

This build keeps the v31 movement controller unchanged (same `driveCell()` and PWM as the smooth `cell` test).

## Fix
`dfs_test` now treats the direction physically behind the robot at launch as a permanent **start boundary**. The test starts facing North, so the original rear direction is South.

Previously, after DFS backtracked to START with a different heading, that South edge could appear as front/left/right and be mistaken for a new unexplored branch. The robot could then drive past its starting point and continue into open space.

v32 explicitly refuses to explore that original rear edge while at START. It can still return to START, inspect/explore any remaining legitimate start branches, and then terminate with `DFS TEST PASS` once those are exhausted.

No PID, distance, turn, ToF threshold, or PWM values were changed.
