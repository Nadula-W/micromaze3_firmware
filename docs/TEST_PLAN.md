# MicroMaze 3 Complete Test and Validation Plan

This plan separates **booklet requirements**, **hardware functional tests**, and **engineering tuning tests**. A rule that cannot be verified by software is listed as a manual inspection item instead of being silently omitted.

## A. Competition-rule manual checks

### A1. Autonomy
- Robot must be self-contained and fully autonomous during a run.
- Final run modes must work without USB, PC, phone, Wi-Fi, Bluetooth or outside control.

### A2. Electrical power
- Measure with a multimeter: voltage between any two circuit points must never exceed 24 V.
- Verify regulators and motor rail under load.

### A3. Size
- Measure the assembled robot, not only the PCB.
- Maximum length: 14.5 cm.
- Maximum width: 14.5 cm.
- Height is unrestricted by the booklet.

### A4. Weight
- Record initial inspection weight.
- Battery replacement must keep overall weight within +/-5 g of that initial weight.
- Reweigh after replacement of competition components when required.

### A5. Mechanical behavior
- No jumping, climbing, scratching, cutting, burning, marking or damaging maze walls.
- No loose/dislodged parts may be left in the maze.
- Cable ties, sensor boards, screws, tyres and battery must be secure.

### A6. Wireless
- Final source must contain no wireless debugging/control path.
- Demonstrate that Wi-Fi/Bluetooth are disabled.
- Keep the `wireless` debug statement and source available for judges if requested.

### A7. Surface tolerance
- Build a test seam/step around +/-2 mm and repeatedly drive across it at slow and fast speed.
- Verify no chassis bottoming, wheel lift, encoder loss or path deviation.

### A8. Lighting
- Test sensors under several ambient conditions.
- Distance sensors should still be verified under bright room light because the booklet says not to assume sunlight/fluorescent conditions.

## B. Static electronics tests - no floor motion

Select DIP `100` Debug.

### B1. Main I2C
Command:
```text
i2c
```
Expected document addresses:
- 0x20 PCF8574A
- 0x50 and 0x51 AT24C1024BN memory blocks
- 0x68 MPU6050

PASS: expected devices appear consistently with no intermittent disappearance.

### B2. PCF8574, DIP switches, buttons, LEDs, buzzer
Command:
```text
io
```
Verify:
- P1/P2/P3 change with DIP switches.
- Key1 changes P4.
- Key2 changes P5.
- LED1 operates.
- LED2 operates.
- buzzer operates.

Also verify every DIP combination maps to the intended mode.

### B3. Four VL53L0X sensors
Command:
```text
tof
```
Verify all four report valid distances and the present mask is `0x0F`.
Move a wall in front of one physical sensor at a time and verify the software index.
If mapping differs, update `SensorMap` in `include/Config.h` before final inspection.

### B4. MPU6050
Command:
```text
imu
```
Verify plausible acceleration and Z gyro data. Rotate robot left/right and confirm Z rate responds.

### B5. External EEPROM
Command:
```text
eeprom
```
The test preserves the original bytes, writes a pattern near the end of memory, verifies it, and restores the original data.
PASS: `EEPROM preserved self-test: PASS`.

## C. Encoder and motor tests

### C1. Encoder manual test
Command:
```text
encoder
```
Rotate/roll the wheels manually.
PASS: both counts change smoothly; stationary wheels do not generate large random counts.

### C2. TB6612 motor truth-table test
Lift robot first.
Command:
```text
motor
```
The firmware tests Motor A forward/reverse and Motor B forward/reverse, then brakes/stops.
PASS:
- each motor rotates in both directions;
- corresponding encoder changes;
- STBY kill stops motion;
- physical wheel direction matches the software convention.

If left/right or polarity is wrong, edit only these constants before final inspection:
- `LEFT_MOTOR_IS_A`
- `LEFT_MOTOR_INVERT`
- `RIGHT_MOTOR_INVERT`

## D. Calibration mode (DIP 000)

Procedure from hardware document:
1. Kill existing mode with Key2.
2. Set DIP `000`.
3. Press Key1.
4. Wait five seconds while LED1 blinks.
5. Run guided calibration over USB Serial during development.
6. Calibration is saved to ESP32 NVS.
7. LED2 indicates idle/calibration complete.

Guided calibration performs:
- stationary MPU6050 Z-bias calibration;
- VL53L0X mapping/read verification;
- exact hand-roll encoder distance calibration (192 mm default);
- wall-present/open threshold calibration for all four sensors;
- side centering target capture;
- motor and encoder verification;
- calibration save to ESP32 NVS.

## E. Distance and cell-motion accuracy

### E1. 192 mm / 19.2 cm cell-pitch test
Command:
```text
cell
```
Mark a precise 192 mm lane. After the robot stops, measure the real travel and enter the measured value.
The program reports error in mm and percent.

Repeat at least 10 times and record:
- actual distance;
- left/right encoder estimate;
- lateral drift;
- battery voltage.

The booklet does not specify an 18 cm tolerance. Define an internal target; a tighter target is better because the lattice-point/cell-center pitch is 192 mm (the clear corridor width is 180 mm).

### E2. One cell
Command:
```text
cell
```
Target = 192 mm.
Repeat through multiple cells and verify accumulated position error.

### E3. Multiple cells
Run:
```text
distance 360
distance 720
distance 1440
```
Check 2, 4 and 8 cell-equivalent straight distances.

## F. Turn tests

Commands:
```text
turn 90
turn -90
turn 180
```
Use a floor angle jig/printed cross.
Repeat each direction at least 10 times.
Check:
- mean angular error;
- left/right symmetry;
- final position translation after in-place turns.

If errors are systematic, recalibrate gyro bias and tune `gyroTurnKp`.

## G. Wall sensing and wall-centering

Command:
```text
walls
```
At cell center, test these layouts:
1. no nearby side walls;
2. left wall only;
3. right wall only;
4. both side walls;
5. front wall;
6. corner/dead end.

Verify L/F/R wall decisions are correct.
Then run straight-cell tests in a corridor and inspect lateral drift.
Tune with:
```text
pid <straightKp> <wallKp> <gyroTurnKp>
savecal
```

## H. Emergency kill test

In every motion test:
1. start motion;
2. press Key2;
3. verify TB6612 STBY drops immediately and motion stops;
4. verify the run/function does not continue driving after the kill latch.

Also change a DIP during a run: the firmware treats any PCF input change during an active run as an emergency stop, which is conservative and also prevents changing modes during a run.

## I. Maze memory and algorithm tests

### I1. Start with a new maze
Before a genuinely new maze layout:
```text
clearmaze
```
Do this before competition/inspection as appropriate. Do not use an external device to modify software during a competition run.

### I2. Exploration mode
Select DIP `111`, press Key1, wait five seconds.
Expected behavior:
- start logical pose `(0,0)` facing the only exit;
- sense left/front/right walls at each cell;
- update N/E/S/W wall map;
- use flood-fill-like navigation through unknown cells;
- reach one of the four center goal cells;
- return autonomously to start while continuing to map;
- calculate shortest path using confirmed open edges;
- write map/path to AT24C1024BN.

### I3. Inspect stored data
Debug command:
```text
maze
```
Verify goal coordinate, path length and absolute N/E/S/W path.

### I4. Slow stored path
DIP `010`.
Run the stored path slowly and visually verify every turn and cell transition.

### I5. Return modes
- `011`: fast out + fast return.
- `101`: fast out + slow return.
- `110`: slow out + slow return.

Returning autonomously is important because the booklet applies a 20-second penalty if the contestant manually lifts the robot from the destination and resets it to the start after a completed run.

### I6. Fast mode
DIP `001` after slow mode is reliable.
Fast mode reaches the destination and stops. If you manually reset to start, competition reset-penalty rules apply.

## J. Collision testing

The booklet applies a 3-second penalty for each wall contact/collision, with successive contacts within three seconds treated as one penalty.

Test:
- long corridor;
- 90-degree corner;
- narrow approach to front wall;
- repeated left/right turns;
- low battery and full battery;
- +/-2 mm floor seams.

Engineering goal: zero contacts.

## K. Competition timing rehearsal

### Qualifier rehearsal
- 8-minute Trial Time.
- Maximum 5 runs.
- Rehearse search/exploration and fast runs without code upload.
- Include all between-run switch/battery/sensor adjustment time in the 8-minute rehearsal.

### Final rehearsal
- 8-minute Competition Time.
- Maximum 10 runs.
- No code upload after maze layout is revealed.
- Robot remains autonomous.
- Practice choosing modes only with onboard DIP switches between runs.

## L. Final pre-inspection checklist

- [ ] Overall size <= 145 mm x 145 mm.
- [ ] No circuit point > 24 V.
- [ ] Weight recorded; spare battery weight matched within +/-5 g.
- [ ] Wireless disabled and demonstrable in source.
- [ ] Key2 emergency interrupt works.
- [ ] Key1 five-second start countdown works.
- [ ] LED1 countdown/running indication works.
- [ ] LED2 idle/calibration-done indication works.
- [ ] All 4 VL53L0X sensors stable.
- [ ] MPU6050 stable.
- [ ] Both encoders stable.
- [ ] Both motors forward/reverse/brake/standby correct.
- [ ] EEPROM read/write and write-protect correct.
- [ ] 192 mm cell-pitch repeatability validated.
- [ ] 192 mm straight repeatability validated.
- [ ] 90/180 degree turns validated.
- [ ] Wall thresholds validated in multiple lighting conditions.
- [ ] Robot crosses +/-2 mm seam.
- [ ] Maze map/path saved and reloaded correctly.
- [ ] Slow path run succeeds without wall contact.
- [ ] Fast path run succeeds without wall contact.
- [ ] Return-to-home modes succeed.
- [ ] No loose/dislodged parts.
- [ ] No wall-damaging mechanical feature.
- [ ] Stored maze cleared before a new maze layout.
- [ ] Final source frozen before inspection; no later flashing/reprogramming.
