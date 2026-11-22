# Session notes

Running notes so a session on either computer can pick up where the last one stopped.
Update the "Current state", "Open issues" and "Next up" sections at the end of each session, and add a dated
line to the log at the bottom.

## Current state (2026-11-02)
- Competition teleops: `RedTeleop` / `BlueTeleop` (`CompetitionTeleop`), now built on the subsystems: Drive (Pedro
  follower for driving + localization), Intake (direct-control methods), Shooter (`runLutFlywheel`), BatteryVoltage.
  - gamepad1: drive (field-centric), right trigger intake / shoot, left trigger outtake, right bumper holds the gate
    open (rumbles), x / b pick the LEFT / RIGHT goal, dpad down = heading reset, dpad up = relocalize to the goal point.
  - gamepad2: start = relocalize pose, bumpers = rpm offset +/-10, triggers = hood offset +/-0.01, dpad left/right = turret offset 3 deg,
    dpad up = heading 90 deg (back against a wall), a = reset offsets, y = flower wedge toggle.
  - Flywheel always targets the LUT speed. SHOOTING uses the per-distance kP table if `useShootingKpLut` is on.
- Shooter settings live in `teleop/systems/Shooter/config/`: `CRShooter` (2 in wheel, CR 1:1, 2900 rpm output,
  two motors) and `UnderShooter` (2 in wheel, 5800 rpm output). Pick with `Shooter.activeConfig` in Panels.
- Turret: full 500-2500 us PWM, -345 deg/position, output smoothing 60 ms, optional single-servo mode.
- Flower wedge enabled everywhere; per-servo up/down positions are in `Intake`.

## Open issues
- Competition teleop on the Pedro follower: driving checked on the robot and fine (as is the LUT test).
  Still to check: the relocalize buttons, and that the pose and aiming match the old teleop.
- Pinpoint pod offsets differ: Pedro `Constants` (used by Drive and autos) has 2.845 / -6.321 in, the LUT test and
  ManualTest have 2.362 / -4.508 in. Decide which is right and use it everywhere.
- `Drive.update(gamepad)` (Robot-based teleops: testCode, LutTuner, ActuallyEverythingTest) now negates the stick
  strafe/turn for Pedro's convention; before, strafe and turn were likely reversed there. Verify on the robot.
- `RedTeleop` sometimes ends on its own with no error. Not a code stop (nothing calls requestOpModeStop).
  Suspects: brief Wi-Fi drop (RC stops the opmode on lost heartbeat), Driver Station timer, Panels/Dashboard
  sending stop. Check `robotControllerLog.txt` around the stop.
- `UnderShooter` speeds (2988-3222, idle 2880 ticks/s) are above a bare goBILDA motor's ~2800 ticks/s free speed,
  and 5800 rpm output is about 2710 ticks/s at the encoder. Needs real numbers (and probably kV, ticksPerRev).
- Turret misbehaves in one turning direction only (ActuallyEverythingLutTest). The aim/wrap math simulates as an
  exact mirror both ways, so suspect hardware/calibration: `TurretPollen.minPosition/maxPosition` are still 0 / 1
  (full +/-172.5 deg, TurretServoTuner step 1 never copied in), so one side may drive into a hard stop or cable.
  Trim fight fixed in code (2026-11-03, untested on robot): the travel limits now shrink so position + trim stays
  in 0..1 for both servos, e.g. rightTrim +0.03 caps the turret at 0.97 (about 10 deg less on that side).
- Placeholders still to tune: flower wedge up/down positions (0.5 / 0), turret offset from robot center (0, 0).
- Check blue's LEFT/RIGHT goal labels on the robot (mirroring may swap them from the blue driver's view).
- Driver-2 rpm offset converts with motor rpm, so with the 2:1 reduction each 10 rpm step is ~5 rpm at the wheel
  (user chose to leave it).

## Shoot on the move (branch `shoot-on-the-move`, not merged, untested on the robot)
For small motion while shooting (getting pushed), not driving shots. `Shooter/ShootOnTheMove.java`, run by
`Shooter.updateAim()`, so every opmode using the Shooter gets it. Settings in Panels under ShootOnTheMove.
- Velocity: Pinpoint (Pedro `follower.velocity()`, field frame; set in `FollowerPose` by `Drive.update()` and
  `Robot.update(Pose)`; the LUT test reads its raw Pinpoint), or the pose change per loop if that's missing or
  `velocitySource = POSE_DIFFERENCE`. Low-pass 40 ms, deadband 1.5 in/s / 4 deg/s (ramps in to 2x), caps 20 in/s /
  180 deg/s. Pose-difference jumps over 150 in/s / 720 deg/s (relocalize) are skipped.
- Lead: aims from pose + velocity x lead and heading + turn rate x lead, lead = TurretPollen.smoothingTimeMs + 40 ms.
- Ball flight: aims at goal - turret velocity x time of flight (3 iterations); flywheel/hood use that distance.
- Still robot = exactly the old aim. Desktop sim: 10 in/s push 5.9 in miss -> 0.35 in; 60 deg/s spin 4.1 -> 0.9 in.
- To do on the robot: push the robot in the LUT test and check pinpoint vs pose-diff telemetry agree (confirms the
  Pinpoint velocity is field frame: push along field +x with the robot turned 90 deg, both should read +x).
  Then measure CRShooter `lutTimesOfFlight` (placeholders 0.40-0.60 s; always used, even with UnderShooter active) with slow-mo video, and check `extraLeadMs`.
- While driving fast the turret pre-swings (capped, about 20 deg at full speed) and swings back on stopping.
  Turn `enabledInAuto` off if that bothers the autos.
- The older full design (zones, brake-and-shoot, fire gate, Foresight stop prediction) is not built.

## Limelight aim override (branch `shoot-on-the-move`, untested on the robot)
`Shooter/LimelightAim.java`, run by `Shooter.updateAim()`. Limelight is on the turret. Inside the zone for the current
side (odometry robot center in a rectangle, red tuned, blue mirrored) with that side's tag in a fresh frame, the turret
angle and LUT distance come from the Limelight; anywhere else, odometry as before. Shoot-on-the-move shifts are added on top.
- Turret angle = commanded turret angle when the frame was taken (history, minus `servoLagMs`) + bearing to the goal
  point - robot turn since the frame. Bearing from tx, range from the 3D tag pose (camera space, floor part).
- Goal point = tag + (goal point - tag) offset rotated by heading + turret angle; tag positions default to the
  GoalTargets points (offset 0 = aim straight at the tag).
- Falls back to odometry if the frame is older than 120 ms or the Limelight disagrees with odometry by > 25 deg.
- Camera is tilted: tag position comes from the Limelight robot-space pose (`useRobotSpacePose`, default), with the
  camera pose entered in the Limelight web UI relative to the turret's rotation center ("robot forward" = turret
  forward). Docs say robot space +y = right; check "tag forward / left" telemetry with a tag on the left, else
  `flipRobotSpaceSide`. Level-camera path (camera space + tx + cameraForward/Left/Yaw) kept for `useRobotSpacePose = false`.
- With the turret encoder (below) the turret angle at the frame is the measured one, no `servoLagMs` guess
  (`useTurretEncoder`, telemetry `turretAngleSource`).
- Pipeline per alliance/side, switched automatically (only on change; frames from the old pipeline are ignored):
  red right 0, red left 1, blue right 2, blue left 3 (`redRightPipeline` etc.). All 4 need setting up in the web UI.
- Default mode is point of interest (`usePointOfInterest`, 2026-11-14): each pipeline = AprilTag + ID filter + POI
  offset to the goal point. Bearing from tx; distance computed in code from ty:
  (poiHeightIn - cameraHeightIn) / tan(cameraPitchDeg + ty). Heights / pitch are placeholders (12 in, 20 deg, 40 in).
  The two 3D-pose modes are still there with `usePointOfInterest = false`.
- Placeholders to set: hardware name `limelight`, the 4 pipelines (need the right tag size), tag IDs (-1 = any),
  tag field positions, camera offset/yaw on the turret, both zones. Calibrate in the LUT test: right after a
  relocalize, Limelight vs odometry turret angle / distance telemetry should agree.

## Turret encoder (branch `shoot-on-the-move`, untested on the robot)
Analog absolute encoder geared 4:1 to the turret (4 encoder turns = 1 turret turn), read by `TurretPollen` (servo
mode stays; the encoder only measures). Analog Input `turretEncoder`, optional (`tryGet`).
- The encoder is only absolute within 90 turret deg. The turn is counted from wraps between loops and re-picked as
  the one closest to the servo command once the command has been still `settleMs` (300 ms), so miscounts fix
  themselves when the turret stops. Needs the servo within 45 deg of its command when still, and the turret
  turning < 45 deg per loop (600 deg/s at a 75 ms loop).
- `getMeasuredAngle()` (NaN until the turn is known), `Shooter.turretMeasuredAnglePollen`, LUT test telemetry.
- Set up in TurretServoTuner: check direction (`encoderReversed`), point forward, back = zero (`encoderOffsetDegrees`),
  copy both into the code. Voltage range defaults 0-3.3 V (`encoderMin/MaxVoltage`).
- Not used yet for anything but the Limelight. Could later correct the servo command (closed loop on top of the
  servo) or confirm the turret is on target before firing.

## Next up
- Limelight: mount/configure, set tag IDs, tag positions and zones, compare with odometry in the LUT test.
- Shoot on the move: robot checks above, measure time of flight, then decide whether to merge the branch.
- Tune: flower wedge positions (FlowerWedgeTest), shooting kP table, UnderShooter values.

## Log
- 2026-11-02: Shooter settings moved to `config/` (CRShooter / UnderShooter, selectable in Panels); flower wedge
  per-servo positions; competition teleop FSM, kP table, gamepad2 controls; turret smoothing; CLAUDE.md and these notes added.
- 2026-11-02: Competition teleop moved onto the Pedro follower and subsystems (Drive rewrite, Intake direct control,
  Shooter.runLutFlywheel, BatteryVoltage util shared with Robot). Driving checked on the robot.
- 2026-11-03: Checked the turret wrap in both turning directions (simulated, symmetric). TurretPollen travel limits
  now leave room for leftTrim / rightTrim so the two servos can't fight at one end.
- 2026-11-07: ShootPreload fired before the flywheel spun up: `Robot.update(Pose)` checked `inRange()` before
  `shooter.update()`, so the first loop compared 0 speed to a 0 target and jumped to SHOOT. Check moved after
  `shooter.update()`, and `inRange()` now returns false for a 0 target. Untested on the robot.
- 2026-11-07: Added `ResetStaticValues` opmode + `StaticDefaults`: records every TeamCode static field (and the
  contents of arrays / TeamCode objects like `CRShooter.values`) when the app starts, and puts them back on INIT.
  Logic tested on the desktop with stubs; the `@OnCreate` hook and DexFile class listing are untested on the robot.
- 2026-11-08: Shoot on the move (small motion / getting pushed) on branch `shoot-on-the-move`: ShootOnTheMove
  (lead + virtual goal), Pinpoint velocity in FollowerPose, time-of-flight table in the shooter configs, telemetry
  in CompetitionTeleop and the LUT test. Simulated on the desktop, untested on the robot.
- 2026-11-09: Limelight aim override (LimelightAim) on `shoot-on-the-move`: turret angle + LUT distance from the
  turret-mounted Limelight inside per-side zones. Builds; untested on the robot (all settings are placeholders).
- 2026-11-13: Turret encoder (4:1 analog absolute) read by TurretPollen; LimelightAim uses the measured turret angle
  when there is one; zero/direction setup in TurretServoTuner. LimelightAim switched to the robot-space tag pose
  (tilted camera, camera pose set in the Limelight UI). Builds; untested on the robot.
- 2026-11-13: Shooter motor directions moved into the configs (`shooter1Inverted` / `shooter2Inverted`, re-applied
  on a config switch). UnderShooter reverses shooter2 (not inverted); CRShooter unchanged (both inverted). Untested on the robot.
- 2026-11-13: Autos: `waitBeforeShoot` (1000 ms) before every shot, `waitAfterShoot` 2000 ms, no shooting during paths;
  intake + transfer off while the flywheel spins up for the preload (`Robot.preloadSpinUp`). Untested on the robot.
- 2026-11-14: Blue autos (BlueDuoLeft/Right, BlueSolo2Flower, ShootPreloadBlue(_Reset), ShootPreload_Left_Blue) extend
  the red ones with `isRedAlliance() = false`; every pose goes through `pose()`, which mirrors for blue. Edit only the
  red file. TestAuto has no blue copy. LimelightAim pipelines 0/1 red, 2/3 blue. Builds; untested on the robot.
- 2026-11-14: Autos: flower intakes hold `flowerIntakeWait` (3 s) at the flower, run at `flowerIntakeSpeed` (0.5, Foresight
  `maxPathSpeed` modifier per path) and finish turning at `flowerHeadingEnd` (75%). Solo / ShootPreload / TestAuto drive
  to (58, 20, 135) for the preload while the flywheel spins up (in auto it's always at LUT speed). RedSolo2Flower aims
  LEFT after the preload and parks after the 3rd shot (garden paths unused). Comp teleop starts at the auto end pose,
  else 70.75, 70.75, 0 (was hard-coded 15, 90, 90); shows the odometry pose in telemetry. LUT test: Limelight lines
  replaced by the pose. Untested on the robot.
- 2026-11-14 open: Pinpoint heading runs away after ~2 min of teleop (hardware: check `FAULT_IMU_RUNAWAY`, static,
  heat, cable; firmware). Turret has a ~20 deg dead spot on direction reversal even with SOTM / Limelight / smoothing
  off (suspect the two servos fighting: test one servo / TurretServoTuner jog). Pedro pod offsets in Constants were
  changed to (2.36, -5.5) from the tuned (2.845, -6.321): confirm with PinpointTuner.
- 2026-11-14: HoodPanelsTest opmode (hood position from Panels). Comp teleop: gamepad2 triggers = hood offset
  +/-0.01 (`Shooter.hoodOffset`, added to the LUT hood, reset by gamepad2 a). Untested on the robot.
- 2026-11-14: `Shooter.hoodConstantAddition` = 0.015 added to every hood command (Panels, under Shooter). Untested on the robot.
- 2026-11-14: Autos: every path into a flower is `race(follow(...), waitMs(flowerPathTimeout))` (GlobalVars, 3000 ms),
  then flowerIntakeWait as before, so a robot stuck on a flower still moves on. Auto INTAKE / POWER_SHOOTER has a
  transfer stall check (`Intake.autoTransferStallMilliamps`); the after-every-path reset is not wired up yet.
  CompetitionTeleop init sets all four red goal values (60,58 / 58,85) since autos change them. Untested on the robot.
- 2026-11-19: Tried Sloth 0.3.2 hot reload (needs Gradle 9.1.0 for the Load plugin; FTC SDK kept at 11.2.1). Did not work
  on the robot (likely Sloth itself), so the `sloth-integration` branch was deleted. No Sloth on any branch.
- 2026-11-22: Reviewed RedSolo3TipPartnerPark (untested) against RedSolo2Flower (works). Fixed: poses go through
  `pose()` (blue mirror), both flower intakes at `flowerIntakeSpeed` like Solo2Flower, `doubleParkSLOW` actually slow
  (`GlobalVars.doubleParkSpeed` 0.4), garden path starts at the alliance-preload shot pose (was 1 deg off). Builds;
  untested on the robot. Open: the fixed waits alone are ~17 s (plus 13 paths), so it likely runs past 30 s and misses
  the garden shot / park; preload gets ~1 s of feed (Solo2Flower waits 2 s extra); `flowerPathTimeout` was lowered
  3000 -> 1000 for every auto, Solo2Flower included.
