# Session notes

Running notes so a session on either computer can pick up where the last one stopped.
Update the "Current state", "Open issues" and "Next up" sections at the end of each session, and add a dated
line to the log at the bottom.

## Current state (2026-11-02)
- Competition teleops: `RedTeleop` / `BlueTeleop` (`CompetitionTeleop`), now built on the subsystems: Drive (Pedro
  follower for driving + localization), Intake (direct-control methods), Shooter (`runLutFlywheel`), BatteryVoltage.
  - gamepad1: drive (field-centric), right trigger intake / shoot, left trigger outtake, right bumper holds the gate
    open (rumbles), x / b pick the LEFT / RIGHT goal, dpad down = heading reset, dpad up = relocalize to the goal point.
  - gamepad2: start = relocalize pose, bumpers = rpm offset +/-10, dpad left/right = turret offset 3 deg,
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
  Then measure `lutTimesOfFlight` (placeholders 0.40-0.60 s) with slow-mo video, and check `extraLeadMs`.
- While driving fast the turret pre-swings (capped, about 20 deg at full speed) and swings back on stopping.
  Turn `enabledInAuto` off if that bothers the autos.
- The older full design (zones, brake-and-shoot, fire gate, Foresight stop prediction) is not built.

## Next up
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
