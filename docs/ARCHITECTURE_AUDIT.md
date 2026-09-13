# Firmware architecture audit

Date: 2026-09-13. Scope: the current working tree after the requested rollback, based on Git commit `c782528`. This is a source audit, not a certification that this firmware is physically reliable.

No firmware, GPIO, calibration, PID, motion, maze, or storage changes were made for this audit. No ML implementation was started. Existing uncommitted changes were retained. The working tree differs from HEAD in `README_HOME10.md`, `include/Config.h`, `include/Maze.h`, and `src/Maze.cpp`; therefore HEAD alone does not reproduce this baseline.

**Build verification is blocked.** Automatic approval review rejected the verification build because the session usage limit was reached. No successful build of this exact rolled-back working tree is claimed. Neither environment has been freshly verified for this audit. Do not treat old `.pio` binaries as verification. No commit/tag was created because the requested compile-before-commit condition has not been met.

Reviewed: all seven `src/*.cpp` files, all eight `include/*.h` files, `platformio.ini`, all seven root README files, both existing `docs` files, `.gitignore`, `.vscode/extensions.json`, and relevant installed Adafruit VL53L0X implementation/error definitions. No repository AGENTS.md, automated test suite, CI workflow, telemetry dataset, optimizer, or ML implementation was found. `TestSuite` contains interactive hardware diagnostics, not automated regression tests.

Evidence notation: **confirmed** means visible in this source; **observed** means supplied robot logs; **unverified** means a physical explanation or measurement still needed. Historical README claims are not proof of current code or successful hardware validation. Competition requirements below refer to the user's supplied specification, not independently verified event rules.

## A. Current firmware architecture

| File | Responsibility |
|---|---|
| `src/main.cpp` | Global object wiring, boot, two I2C buses, DIP dispatch, Key1 countdown, deferred Key2 handling, wireless build selection |
| `src/Hardware.cpp`, `include/Hardware.h` | PCF8574, AT24C1024, four VL53L0X objects, snapshot task, MPU6050, TB6612, encoder ISRs |
| `src/Motion.cpp`, `include/Motion.h` | Blocking distance/cell controller, front alignment, encoder pivots, gyro diagnostic turns, dormant anchored controller |
| `src/Maze.cpp`, `include/Maze.h` | Wall map, flood distances, DFS diagnostic, outbound-history retracing, shortest path, external storage |
| `src/TestSuite.cpp`, `include/TestSuite.h` | USB/browser commands, manual tests and calibration |
| `src/CalibrationStore.cpp`, corresponding header | ESP32 Preferences/NVS calibration blob |
| `src/WebTerminal.cpp`, corresponding header | Development AP, HTTP terminal, queued input and log mirror |
| `include/Config.h`, `include/RobotTypes.h` | Pins, addresses, constants, calibration defaults, snapshot/pose/storage types |

`main` creates one shared `CalibrationData`, one `MotionController`, one `SensorHub`, one navigator/map/storage set. Autonomous execution is synchronous in the Arduino task. The sensor task runs on core 0, priority 2, stack argument 4096. Development HTTP task runs on core 0, priority 1, stack argument 6144. Snapshot copies use a short critical section; I2C measurements run outside it.

PlatformIO defaults to `esp32-s3` with `MM3_DEV_WIFI=1`. `esp32-s3-competition` sets it to 0. Both use Arduino, `esp32-s3-devkitc-1`, 16 MB flash, QIO/OPI settings, 115200 monitor baud, 57600 upload baud, and the same maze/motion constants. The platform is unpinned; library dependencies use compatible-version ranges (`VL53L0X ^1.2.5`, `MPU6050 ^2.2.6`, `Unified Sensor ^1.1.15`). The standalone screenshot selects a different board label; actual flash/PSRAM module compatibility is not established by this audit.

| DIP | Action |
|---|---|
| 000 | Guided calibration |
| 001 | Stored path at PWM 155 |
| 010 | Stored path at PWM 90 |
| 011 | Stored fast outbound and fast return |
| 100 | Debug console |
| 101 | Stored fast outbound and slow return |
| 110 | Stored slow outbound and slow return |
| 111 | Exploration at PWM 90 |

Key1 is active HIGH, debounced 35 ms, requires release, and starts a 5-second countdown. PCF bits 0..2 are DIP, bit 3 Key1, bit 4 Key2, bits 5..7 LED1/LED2/buzzer. Documentation labels these P1..P8; bit numbering is zero-based in code.

Development starts an AP and HTTP server with `/`, `/log`, `/cmd`; a 1024-byte queue feeds the same debug parser. Logging uses a 24 KB String with 6 KB trimming and mutex waits up to 100 ms. USB writes are synchronous. Direct `Serial` messages from Motion do not all reach the browser log. Queue-send failures are ignored, so commands can truncate under load. USB and browser input can interleave. Competition excludes the web implementation and explicitly stops/deinitializes Wi-Fi/Bluetooth at boot; this alone does not configure the requested competition maze.

## B. Exact sensor-reading architecture

| Bus/device | GPIO/address | Current behavior |
|---|---|---|
| Main `Wire` | SDA 8, SCL 9, 400 kHz | PCF 0x20, MPU 0x68, EEPROM 0x50/0x51 |
| `DistanceWire(1)` | SDA 7, SCL 15, 400 kHz | Dedicated ToF bus |
| Index 0 / front-left | XSHUT 4, address 0x30 | Separate Adafruit object |
| Index 1 / left | XSHUT 2, address 0x31 | Separate Adafruit object |
| Index 2 / right | XSHUT 16, address 0x32 | Separate Adafruit object |
| Index 3 / front-right | XSHUT 18, address 0x33 | Separate Adafruit object |

Pins/addresses agree with the supplied standalone sketch. No duplicate assigned ToF addresses or overlapping configured GPIO functions were found. Physical sensor orientation and motor polarity remain hardware facts, not proven by constants.

`DistanceArray::begin` holds all XSHUT LOW for 100 ms. For each sensor it raises XSHUT, waits 100 ms, calls `begin(0x29, false, bus)`, then `setAddress(0x30+i)`. Failed initialization resets that sensor. It then starts every initialized sensor using `startRangeContinuous(50)`, with an 8 ms gap between starts. **Address-change and continuous-start return values are ignored.** The presence mask records successful `begin`, not ongoing ranging health.

The canonical runtime chain is:

```text
SensorHub::taskLoop (read pass, then 20 ms delay)
  -> DistanceArray::readAll
     -> each sensor.isRangeComplete()
     -> if ready: readRangeResult(), readRangeStatus()
  -> publish SensorSnapshot
motion / walls / tof / wall calibration -> SensorHub::snapshot()
```

There is no runtime `rangingTest` call in project source. All four devices range continuously; readiness/results are polled sequentially. This differs from the working standalone sketch's four blocking single-shot measurements followed by 100 ms delay. The standalone sketch does not explicitly set the bus to 400 kHz. Matching initialization does not establish equivalent runtime timing.

The installed library's `isRangeComplete` also reports true when its API status is an error. `readRangeResult` fetches measurement data, clears the interrupt on a successful fetch, and returns `65535` on API/clear failure or range status 4. Its local measurement struct is not initialized before the API call, so range status is not reliable after failure. Project diagnostics correctly display 255 when read API status is nonzero.

Project validity is `rangeStatus != 4 && mm > 0 && mm < 4000 && mm != 65535`. It excludes zero/sentinels but accepts other nonzero range statuses; it does not explicitly require API success plus range status 0. Current library error sentinels generally prevent API failures being accepted, but this is an implicit dependency.

When not ready, an old value and validity remain unchanged indefinitely. `readStampMs[i]` is a read-attempt timestamp, including failed attempts. Shared `stampMs` updates when any sensor is read, including invalid results. Neither is a per-sensor last-success timestamp. Front confirmation counts shared timestamp changes; a side sensor can therefore make an unchanged front sample count again. Calibration sampling can also count the same sample repeatedly.

The log `readyAPI=-6 readAPI=-12 range=255 raw=65535` maps locally to `RANGE_ERROR`, `INTERRUPT_NOT_CLEARED`, unavailable range status, and error sentinel. It demonstrates a ranging/API failure, not a valid measured distance or proof of a broken cable. `0xF` does not contradict it. No automatic recovery/reinitialization exists after boot. Sensor 3 worked in an earlier log, but sustained four-sensor reliability has not been established. Bus integrity, power, sensor configuration, optical geometry, and continuous-mode behavior remain diagnostic possibilities, not identified physical root causes.

## C. Motion call graph

```text
debug cell -> testDistanceMove(192, 90) -> driveDistanceMm
debug distance mm pwm -> testDistanceMove -> driveDistanceMm
dfs_test / dfs_fast -> homeDfsTest -> stepPose
explore / DIP 111 -> explorationRun -> stepPose
stored run DIP modes -> storedRun -> executeAbsolutePath -> stepPose
stepPose:
  turnToHeading
    if heading changes and wallFront: alignFrontToWall
    turnEncoderTicks (one or two 378-tick pivots)
  invalidateMazeSegmentAnchor
  driveCell -> driveDistanceMm(192, pwm, true, true, false)
  advance logical coordinates if movement returns true
deadend_test -> turnToHeading + driveCell, outbound and reversed route
front_align -> alignFrontToWall
turn -> turnDegrees (gyro diagnostic)
turn_ticks -> TestSuite's separate encoder pivot loop
turn_diag -> TestSuite's separate fixed-PWM gyro loop
motor -> direct MotorSystem A/B writes
```

`driveAnchoredMazeCell` has a complete duplicate drive loop but no callers in the repository. `driveLocalizedCell` and `driveKnownOpenCell` also have no callers and both disable lattice locking. No current caller enables the optional lattice-lock branch. Header comments claiming active anchored maze navigation are incorrect.

## D. Exact straight-driving PID

Travel is absolute encoder displacement since entry divided by separately calibrated ticks/mm; average travel is the mean of the two wheels. Encoder signs are discarded for progress. Local PID state resets on each distance/cell call.

```text
encoderError = leftMm - rightMm
both side walls: wallError = 0.5 * ((L - targetL) - (R - targetR))
left wall only: wallError = L - targetL
right wall only: wallError = -(R - targetR)
no calibrated side reference: wallError = 0
error = encoderError + wallError  [forward with centering enabled]
integral = clamp(integral + error, -250, 250)
derivative = error - previousError  [first iteration: 0]
pid = Kp*error + Ki*integral + Kd*derivative
correction = clamp(round(pid), -55, 55) + sideEscape
correction = clamp(correction, -78, 78)
left = direction * clamp(baseCommand - correction, 58, 255)
right = direction * clamp(baseCommand + correction, 58, 255)
```

This is a discrete per-iteration PID, **not a seconds-based PID**. There is no multiplication/division by measured dt. The loop ends with `delay(4)` but also does sensor copies, floating-point work, kill I2C handling and occasional prints. Effective I/D behavior changes with loop timing. The integral has a clamp but no actuator-saturation anti-windup; D has no filter. Switching between one/two/no walls can step the error. Individual wheel minimum-PWM clamps can erase small corrections or distort the requested pair. The dormant anchored controller repeats these equations.

## E. Complete controller/gain inventory

Defaults below are source defaults. Saved NVS values override them; the robot's present NVS contents were not read by this audit.

| Parameter / coefficient | Value | Location and use |
|---|---|---|
| `pidKp/Ki/Kd` | 1.8 / 0 / 0.03 | `RobotTypes.h`; active drive and dormant anchored steering |
| `frontKp/Ki/Kd`, `FRONT_ALIGN_K*` | 1.25 / 0.20 / 0.08 | Config defaults, calibration fields; front distance PID, seconds-based |
| `squareKp/Ki/Kd`, `FRONT_SQUARE_K*` | 0.90 / 0.15 / 0.05 | Front angular squaring PID, seconds-based |
| Side error blend | encoder 1, side 1; two-wall average 0.5 | `wallSteeringErrorMm`, drive loops |
| Side escape | +/-28 PWM, 12 mm margin | Both drive implementations; additional threshold correction |
| Drive I/output limits | I +/-250; PID +/-55; combined +/-78 | Both drive loops |
| Front I/filter | +/-12 PWM contribution; tau 0.08 s | Local `AlignPid` |
| Front actuator limits | normally 72 max, 58 pulse magnitude | Distance/square mixing, proportional pair scaling |
| Encoder pivot synchronization | integer `(leftTicks-rightTicks)/10`, clamp +/-16 | `turnEncoderTicks` and duplicate `testEncoderTurnTicks` |
| Gyro turn proportional coefficient | 1.2 PWM/degree | `turnDegrees`: `58 + 1.2*remaining`, max 105 by default; below 18 degrees cap at 70 |
| Gyro diagnostic drive | constant PWM 68 | `testTurnDiagnostic`; no tunable PID |
| Encoder diagnostic drive | constant PWM 68 plus sync | `testEncoderTurnTicks` |
| Ordinary launch scaling | `0.68 + 0.32*u` over first 8% | `driveDistanceMm` |
| Ordinary front approach scaling | `0.42 + 0.58*u`, then max 0.50 past encoder target | 170 to 80 mm front reference |
| Ordinary final braking scaling | `0.60 + 0.40*u` over final 10% | No front candidate |
| Dormant lattice braking | `0.44 + 0.56*u` over final 90 mm | Optional lattice branch, no active caller |
| Dormant anchored profiles | launch `0.70+0.30*u` over 18 mm; front `0.45+0.55*u`; end `0.60+0.40*u` over 22 mm | `driveAnchoredMazeCell` |
| Profile clamps | ordinary 0.35..1, anchored 0.38..1; wheel min 58 | Feed-forward PWM scheduling, not speed feedback |
| Recovery | PWM 78, reverse 120 ms, at most 2 | 550 ms progress watchdog, 1.5 mm progress increment |
| Requested speeds | slow 90, fast 155, encoder pivot 68 | Config; no wheel-speed PID |

Other calibration factors are ticks/mm left/right, side targets, four wall thresholds, and gyro Z bias. MPU filtering is configured to 44 Hz; yaw diagnostics additionally use median-of-three samples 700 microseconds apart. Gyro turn integration uses absolute rate, a 0.8 dps deadband, and rejects dt >=0.12 s. No current `straightKp`, `wallKp`, `gyroTurnKp` runtime fields, hidden ML gains, or speed PID were found; those names in older docs are obsolete.

## F. What terminal `pid` means

`pid <Kp> <Ki> [Kd]` sets the three shared drive fields above. Two numbers retain the old Kd. It does not change front alignment, squaring, encoder turns or gyro turns. Changes apply to the next synchronous motion call; `savecal` persists the whole calibration structure. `status` shows drive gains. Bare `pid` is not a view command.

`sscanf` accepts two/three floats without finite, nonnegative, upper-bound or trailing-token validation. Front commands instead require exactly three finite values in 0..100 or allow a bare view command. That numerical bound is not a physically validated safe gain envelope.

The PID-only patch retained the old v32 three-float storage positions and version. Old straight/wall/turn numbers can silently become Kp/Ki/Kd. README tells the operator to reset them manually, but there is no schema-level semantic migration. A future dt-based controller must explicitly address this again; copying these Ki/Kd numbers into seconds-based equations would change behavior.

## G. Are multiple steering controllers active?

One combined encoder/side-wall PID controls ordinary straight movement, plus the non-PID side escape. There are not two independently tuned normal straight/wall PIDs running concurrently. There are, however, separate motion implementations and actuator policies to maintain: dormant anchored steering, gyro diagnostics and duplicated encoder pivots.

Front distance and square PIDs run together only during alignment, then stop before the pivot/drive. They control different axes. They can compete for the limited wheel output through mixing, especially with asymmetric or stale readings. No independent background task writes motors; the sensor and web tasks publish data/input only.

## H. Cell and distance behavior

`cell`, DFS, exploration and stored-path cells use the same `driveDistanceMm` implementation with a 192 mm target. `distance N` uses one call with target N; repeated cells reset PID/progress references and brake between calls, so a long distance and repeated cells are not equivalent trajectories.

Normal completion requires average travel >= target and each wheel >=94% target, **unless a front candidate suppresses completion**. A candidate requires usable front data <=170 mm after 55% target travel. Two shared snapshot changes activate the reference; once front <=83 mm, the function returns success even before the nominal cell distance. With a candidate, it can keep driving past 192 mm toward 80 mm; active reference beyond target+90 mm fails. An unconfirmed candidate also suppresses encoder completion. Timeout is target*35+3000 ms, clamped 3..20 seconds.

This confirms why a 192 mm command can log 232/250/270 encoder mm: the source deliberately substitutes a front-wall reference. Conversely it can return success from about 105.6 mm onward if the front reference reaches its threshold. `stepPose` still advances exactly one cell. The code does not prove that the observed wall belongs to the next cell boundary or that 80 mm means a cell center for this geometry.

The previously reported `MOVE BLOCKED: front wall before encoder target` branch is absent after the rollback; it must not be confused with this current source. The current `MOVE FAIL: front reference never reached target distance` branch is present. Historical logs may come from different binaries.

## I. Front alignment

Called explicitly by `front_align` and before a heading change if `wallFront()` is true. Normal straight travel uses front-reference stopping, not the front PID. The dormant anchored path can also invoke alignment after its 105 mm pre-stop.

Target defaults to 80 mm, accepted command range 20..140. A 20 ms minimum control interval uses measured seconds. Both valid front sensors provide average distance and `FL-FR` square error; one valid sensor provides distance only and squaring is considered satisfied. Neither valid, or shared timestamp older than 200 ms, aborts. Mask changes reset PID states. Stale individual fronts are not excluded if other sensors keep the shared timestamp fresh.

Each local PID filters `(error-previous)/dt` with `dt/(0.08+dt)`, clamps integral contribution to +/-12 PWM, conditionally integrates under saturation, and clears integral on error sign change. Translation uses front gains; turn is negative square-PID output. Wheels receive `drive-turn`, `drive+turn`, scaled together to max PWM. Below 58 demand, independent pulse accumulators emit occasional +/-58 commands. Zero intervals use the motor zero-output path, not `stop(true)`'s explicit 20 ms braking sequence despite the pulse comment.

Passing requires distance within +/-15 mm and, when both fronts exist, square error within +/-6 mm for 120 ms. At target 80, 65..95 mm can pass. Repeated snapshots can satisfy this dwell. Timeout is 5 seconds. Traces are buffered during control and printed after timeout; start messages are printed before the loop. The supplied 86 mm timeout trace belonged to the former +/-3 mm condition. It does not establish a good gain set for this version.

No angular-displacement cap, wall-pair geometry qualification, or retreat obstacle sensor exists in alignment. A persistent front offset or seeing different surfaces can produce sustained rotation. The user's reported near-180-degree event is not enough to prove its exact cause; sensor pairing/offset and physical squaring direction need measurements.

## J. Encoders and turns

TB6612: STBY42; A=right, PWM11, direction12/13; B=left, PWM48, direction21/47. Invert flags are false. PWM is 20 kHz, 8 bit. Encoder A uses41/40, B39/38, CHANGE on both quadrature channels. Transition tables update signed int32 counts; invalid transitions are ignored without counters. Reads/resets use critical sections, but left/right reads are separate. Actual signs and wheel-side mapping require physical confirmation.

`turnEncoderTicks` resets counters and uses average absolute wheel ticks. Positive is left, negative right. A 90-degree target is 378 ticks at PWM68, constrained58..100, with synchronization correction /10 limited16. Timeout3 s. There is no deceleration by remaining ticks, per-wheel completion minimum, or gyro angle validation. One wheel could compensate for an under-moving wheel in the average. Braking lasts20 ms and coast is not included in the success condition.

`turnToHeading` performs right=-378, left=+378, and 180 as two +378 pivots separated by350 ms. Successful90 turns wait250 ms; 180 waits300 ms after the second. Front alignment adds a180 ms pause. The maxPwm argument is ignored. Heading changes only on full turn success; a failed second half of a180 leaves physical heading changed but the old logical heading. `turn` is a different gyro diagnostic, so passing it does not validate navigation pivots.

## K. Wall decisions and map integrity

Left/right: a valid reading strictly below its saved threshold is a wall. Front: either valid front below its individual threshold is a wall. Distance output averages two fronts, but reference driving additionally requires their difference <=40 mm. These are different acceptance rules.

Invalid readings yield false (open), rather than unknown. There is no per-sensor freshness requirement, hysteresis, temporal opening confirmation, or cell-position gate. `senseCurrentCell` obtains three separate snapshots through helpers, marks all three directions known, and mirrors each edge into its neighbor. Arrival-back is marked open; there is no persistent travelled-edge protection. A later noisy reading can close a previously travelled edge. Boundary walls can be overwritten in storage, although `canMove` still rejects out-of-bounds neighbors.

`cal_wall` averages35 samples of PRESENT and OPEN. Threshold is their midpoint; if OPEN is invalid it uses PRESENT+96 mm capped500. This can learn distant-wall geometry rather than a close-wall criterion, and a broken sensor in OPEN placement is indistinguishable from legitimate no-target. Targets update incrementally; an interrupted/failed calibration can leave some RAM fields changed. No fresh-sample count is enforced.

## L. Exploration versus DFS

Coordinates: start(0,0), North=+y, East=+x; arrays indexed y*MAZE_N+x; pose uses int8. Current bounds0..9. BFS queue/distance indices are uint16 and sized N*N. Normal neighbor selection checks bounds. `stepPose` checks bounds only after turning/driving, which is too late as a last line of defense for a malformed stored path.

`explore` is flood-distance navigation, not DFS. It loads a valid old map when available, otherwise resets it. Unknown edges are optimistically open for planning. Neighbor score is `100*floodDistance + 20*visited + preference`, with preference straight,left,right,back. A physically possible180 is already considered, but if the map gives every neighbor infinite goal distance, the planner stops. It does not backtrack to repair a disconnected map.

At the configured goal, it reverses the exact successful outbound heading sequence, including detours. Up to700 outbound moves and700 reverse moves are allowed. It saves only after history empties and logical pose is back(0,0). Return movement still runs alignment and ordinary cell guards. It cannot guarantee the physical reverse path if earlier distance/turn results were inaccurate. It continues sensing on return and can alter the map used for final path calculation. History is RAM-only; failed/partial moves are not appended.

`dfs_test` uses separate visited/parent arrays, straight/left/right unvisited preference, and parent backtracking. It starts0,0,N and permanently excludes original rear South at start. Local N is hardcoded10, action limit300; current global N matches, but changing only Config to16 will not expand DFS. With correct sensing/pose,198 moves suffice to traverse/backtrack a100-cell tree. It has no three-cell limit and does not target6,6. It does not maintain remembered branch observations; a transient false wall can omit a reachable branch. The35 ms settle delay does not guarantee a new front measurement on a50 ms ranging period. Backtracking comments claim slower/no-guard behavior, but the actual call uses the same PWM and guarded `driveCell`.

## M. Competition goal

Both environments currently detect **exactly (6,6) in a10x10 maze**. `isGoal`, BFS seed, and logs share MAZE_GOAL_X/Y. The supplied final requirement of16x16 with four central cells is not implemented by selecting the competition environment. This requires a deliberate configuration/goal-set change and storage compatibility decision later. Current goal bounds have a static assertion; a multi-goal BFS would need all goal cells seeded consistently with completion/path validation.

## N. Shortest path and fast run

`buildShortestPath` uses BFS with unknown edges blocked, then chooses minimum-distance neighbors with straight/left/right/back tie preference until the goal, max512 moves. On a consistent undirected map this gives shortest cell count among confirmed-open edges. It is not guaranteed globally shortest through unexplored space and does not optimize total turn time.

Stored paths are absolute headings, one per cell. `storedRun` starts its logical pose0,0,N, replays through `stepPose`, and optionally reverses/opposes the same path. Fast mode increases requested straight PWM to155 but still brakes each cell and uses the same68-PWM stopped turns. There are no merged straight segments, trajectories, slalom turns, wheel-speed control or measured maximum reliable speed. There is no `fast_run` terminal command; DIP modes select stored runs, while `dfs_fast` is fast DFS exploration.

Loaded heading bytes are masked with3 rather than validated. Replay does not preflight all edges/bounds or check its final position against the stored/configured goal before declaring outbound completion. Return checks logical0,0 only. Final heading need not beNorth, so another run requires physically correct start placement/orientation.

## O. Calibration persistence

ESP32 Preferences namespace `mm3cal`, key `data`, stores native-layout `CalibrationData`, magic0x4D4D3343, version3. Layout on the expected32-bit ABI is68 bytes; old records ending before `frontKp` are44 bytes. Code accepts either `sizeof(CalibrationData)` or `offsetof(frontKp)` length. A default-initialized temporary supplies appended front gains when loading44 bytes. It validates length/magic/version, not numeric finiteness, plausible values or PID semantics.

`savecal` writes all fields. `clearcal` clears NVS but leaves current RAM calibration until reset/recalibration. Missing calibration defaults ticks/mm to0, preventing ordinary moves, but the stricter finite-value `calibrationReady()` helper has no callers. Direct setter accepts positive infinity; other numeric parsers are similarly permissive. Preserve existing wheel calibration and front gains; a future migration must distinguish format compatibility from changed controller semantics.

## P. External maze/path storage

AT24C1024 handling uses131072 bytes, address bit16 selects0x50/0x51, 16-bit word address, <=28-byte Wire chunks,256-byte page boundaries and ACK polling. GPIO10 controls write protection, re-enabled after write-loop failures. Packed `MazeBlob` at address0 contains magic, version, path length, goalX/Y, two reserved bytes, N*N walls, N*N known masks,512 path bytes, CRC32. Size is728 bytes for10x10 and would be1040 for16x16. CRC uses reflected polynomial0xEDB88320 over bytes before crc.

Load validates header/version/pathLen<=512 and CRC. It does not validate map symmetry, boundary bits, goal bounds, heading bytes, or full route legality. Dimension/goal-set identity is not explicit in the record. A future layout/config change needs compatibility handling. Writes replace one blob in place without dual slots, commit marker or post-save readback. Power loss can produce a CRC mismatch; the user's CRC message does not identify which cause occurred. Clear overwrites only magic.

EEPROM self-test backs up16 bytes at0x1FFE0, writes/verifies/restores. Some early failure returns after writing skip restoration; it is not unconditionally preserved. `all_safe` includes this write test despite having no wheel motion.

## Q. Confirmed bugs, risks and evidence limits

| Priority | Finding and source | Consequence / evidence boundary |
|---|---|---|
| First | Key2 ISR only sets `gIoInterrupt`; `emergencyStandbyOffFromISR` has no callers (`main`, `Hardware`, `TestSuite`) | Normal Motion loops service kill; direct motor/turn diagnostic loops and `readLine` check only an unset motor latch. Claimed immediate independent kill is not implemented across all paths. |
| First | Blocking500 ms motor-test drives and120 ms recovery reverse without kill polling | Key2 response depends on execution path, I2C and delays. Measure after repairing logic; do not rely on the console's immediate-kill claim. |
| First | PCF read failures return0, and interrupt service clears flag before read | Read failure can be treated as Key2 released and lose an event. Direct stop(true) also re-enables STBY for braking even if a latch were set. |
| High | Shared freshness and retained valid readings (`Hardware::readAll`) | One healthy sensor can conceal another's stale reading; repeated data can pass confirmation/dwell. |
| High | Continuous-start/address results unchecked; no recovery | Presence0xF can coexist with repeated front-right errors. Exact cause of S4 failure remains unverified. |
| High | Range validity accepts status values other than0/4 | Unqualified measurements may enter steering/map. |
| High | Cell completion switches to front reference (`Motion::driveDistanceMm`) | Early/late physical stops still count as one cell; logs directly demonstrate non192 encoder stops. |
| High | Front collision handling is inside forward+centering branch and early guard only covers <50% target | Disabling centering disables that guard; later obstacles are not covered uniformly. Averaging fronts can conceal a nearer corner. |
| High | Invalid=known open; no temporal wall confirmation (`Motion`, `Maze`) | Unknown sensing becomes route evidence. False walls can close travelled edges; false opens can command blocked routes. |
| High | Alignment mixes potentially mismatched front surfaces, can pass with one front, uses shared freshness | Alignment success does not prove squareness; prolonged rotation possible. Physical sign/offset needs validation. |
| High | Per-loop drive I/D and no semantic NVS migration | Gain meaning/timing differs from requested seconds-based PID; old slots can load inappropriate numbers. |
| High | Coordinates update only from successful move counts; no pose confidence/result details | Partial drive/turn or false success causes physical/logical divergence. Log(1,4) versus reported(3,5) proves disagreement, not its exact origin. |
| High | Stored route preflight absent; bounds checked after motion | A structurally valid but semantically bad blob can cause an inappropriate physical move. |
| Medium | Threshold calibration uses distant OPEN, repeated samples, incremental mutation | Thresholds/side targets can be unsuitable even after an apparent calibration pass. |
| Medium | Normal startup ignores sensor-task creation result and permits incomplete sensor health | Hardware presence print is not a motion-readiness gate. |
| Medium | Encoder turn average only, absolute gyro integration, no physical turn verification | Wheel imbalance or wrong rotation can appear complete. Correct tick totals do not prove angle. |
| Medium | New move origins ignore previous braking overshoot; minimum PWM limits fine motion | Drift and intermittent pulse adjustments can accumulate. Actual distance/angle data needed. |
| Medium | Planner stops on map disconnection and outbound limit without guaranteed home recovery | Available physical turn alone does not ensure finite mapped route. |
| Medium | Both builds home10/goal6,6; DFS hardcoded10; EEPROM format implicit dimensions | Competition configuration and migration remain unfinished. |
| Medium | No finite/range validation for drive/calibration inputs/load; unbounded terminal String | Malformed values/input can corrupt control or exhaust memory. |
| Medium | Serial prints before moving motors are stopped; web logging blocks and drops queued characters | Timing disturbances/missing output are possible, but printing alone is not established as the navigation failure cause. |
| Medium | EEPROM writes non-atomic; self-test restoration not guaranteed on all exits | Interrupted write can invalidate saved data. |
| Maintenance | Dormant anchored/lattice code and contradictory docs | Future changes can target inactive code and appear ineffective. |
| Validation | No current successful build, automated replay tests, repeatability dataset or hardware measurements | No reliable-speed, PID, sensor or competition acceptance claim is justified. |

Specific documentation conflicts: README_V38/V39/V40 describe single-shot acquisition and100 kHz, while source is continuous400 kHz. TEST_PLAN still gives old PID semantics, names nonexistent `gyroTurnKp`, labels360/720/1440 mm as2/4/8 cells despite192 pitch, expects four-center goals, describes a different return policy, and claims any DIP change immediately kills. Guided calibration actually rolls800 mm. Motion.h claims active anchored movement. These should be corrected after baseline behavior is agreed, not used to infer that those features currently work.

## R. Smallest recommended sequence

1. **Preserve and verify this source baseline.** Complete unchanged development and competition builds when execution is available: `pio run -e esp32-s3` and `pio run -e esp32-s3-competition`. Record actual compiler/library versions, warnings and binary identity. Preserve the dirty working tree, calibration output and last physically trusted firmware separately; do not call this rollback physically proven. No flash is implied by compilation.
2. **Repair common stop handling before powered validation.** Make Key2 reach every actuator path with bounded response and preserve latch semantics across braking/recovery. Avoid I2C inside the ISR. Specify treatment of other PCF inputs explicitly; do not silently change DIP behavior. This changes physical behavior and needs confirmation under the supplied instructions.
3. **Qualify the existing continuous ToF path.** Keep GPIO/address map and all four continuous sensors. Check initialization/start results, expose independent health/last-success/sample sequence, distinguish API errors from no-target, and establish bounded recovery. Verify each sensor at50/80/100 mm with a flat target, plus no target and repeat power cycles. Establish actual per-sensor timing before changing acquisition cadence. Capture diagnostics for all four, not onlyS4.
4. **Make wall evidence explicit.** Use one decision snapshot with qualified fresh readings; invalid stays unknown. Add measured close-wall thresholds, opening confirmation and conflict reporting for travelled edges. Replay artificial valid/stale/error sequences and synthetic maze junctions before physical tests. Keep exploration preference and outbound retracing intact.
5. **Define and test cell completion separately from optional docking.** Decide the exact physical-center reference from measurement. An ordinary192 mm target must not silently become arbitrary80 mm wall seeking. Return typed completion/blocked/partial outcomes and preserve uncertain pose on interruption. Do not simply convert early wall stops into success or blindly restore the132 mm abort policy that already failed the user's route. Test wall approaches, open lanes, late candidates, missing fronts and turn exits.
6. **Introduce one time-based drive PID only after timing is measured.** Reuse one implementation for ordinary movement; keep distinct front distance/square axes. Preserve old gain semantics via explicit version/migration or an opt-in controller version, never automatic gain reinterpretation. Add measured dt, output-aware anti-windup, derivative handling and timing telemetry. Verify equation/saturation behavior offline and retune from physical data.
7. **Validate alignment and pivots independently.** Retain80 mm and+/-15 mm pending geometry measurement. Validate each front's offset, pair consistency, squaring sign, too-close retreat and too-far approach; establish angular/travel bounds. Repeat left/right90 and180 with actual angles and wheel travel. Do not assume378 ticks stays correct after mechanical or calibration changes.
8. **Re-run the same home route repeatedly.** Record physical cell coordinates at each move, stop reason, heading and both wheel distances; compare to logged pose. Verify DFS coverage,6,6 goal and exact outbound retrace before changing planner policy. Demonstrate stored slow replay before increasing speed.
9. **Validate persistence and competition configuration.** Add nonmoving map/path/CRC/goal validation tests. Agree EEPROM compatibility before dimension/schema changes. Separate home10/single goal from requested competition16/four-center goals; seed BFS and validate replay consistently. Test interrupted storage separately from navigation.
10. **Measure fast behavior, then telemetry/optimization/ML.** Establish repeatable fixed-PID baseline across battery/load/lighting, conservative stored fast runs, and maximum reliable speed from measurements. Only then implement bounded telemetry, reproducible optimization, real-data labels, training/validation and optional TinyML gain scheduling with fixed-PID fallback. No gains, labels, model performance or speed limits can be supplied accurately from these logs alone.

Each later firmware patch should address one demonstrated issue, compile both relevant environments, review its diff, record validation/rollback information, and only then be committed. Physical tests require the user's measurements and confirmation; EEPROM compatibility changes and replacement of working exploration require explicit agreement. This audit does not implement those future stages.

### Baseline source fingerprints

SHA256 of selected files as read for this audit (raw working-tree bytes):

```text
src/Hardware.cpp       663CEBCF0BF959B319208909FB1D7D3C2867C5BF61566A2126E80BCDFC3DE86B
src/Motion.cpp         C28CF22A1D7108E46A651CADB32329F9DBB041EB7B51FE7837BF63A80B8D4C11
src/Maze.cpp           C2EB8C3D62C288DE9DAFCAB5C41EB7ED9542A5CF22F30DD0779E93FC7210DEB2
include/Config.h       02F7015D92D20431237CCADA2F251BBC94B0B4248B3A18127852D2DCC4B03851
include/RobotTypes.h   A78C6C638C454DD046B9DB7E781BA62C04FCDCBBB654F3E4315E986314092805
platformio.ini         4140A44513838355D9F529DB2284CFFDF560D6DD131B1685884F6A500AC415D5
```
