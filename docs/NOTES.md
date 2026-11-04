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

## In progress: shooting on the move (design only, no code yet)
Plan discussed, waiting on the user's answers:
- Aim at a "virtual goal" = goal - robot velocity x flight time (iterate 2-3x); turret, speed and hood all use it.
  Needs a flight-time vs distance table per shooter config.
- Aim from the predicted release pose (pose + velocity x total delay: loop, turret, feed).
- Zones with policies: close zone = moving shots under a speed cap; far zone = brake-and-shoot (predict the stop
  pose with Pedro Foresight, pre-aim, fire when slow); optional constant-distance "orbit" lane.
- Fire gate: predicted release pose in zone, speed / turn rate under limits, flywheel in tolerance, turret settled.
- Braking model already tuned in `pedro/Constants.foresightConfig` (forward stop distance = 0.155 v + 0.0008 v^2,
  strafe = 0.101 v + 0.0014 v^2, heading = 0.056 w + 0.005 w^2). Validate it against teleop braking with logs.
- Build order: Pedro follower in teleop + flight-time table, then brake-and-shoot, then low-speed moving shots,
  then moving shots in auto.
- Open questions asked: which zones, aim tolerance, motion types, max speed, auto-fire vs manual, stream vs single,
  whether the robot may hold a point, feed delay, turret feedback, flywheel recovery, which shooter at comp,
  turret center offset, switching teleop to Pedro's follower, whether Foresight was tuned on the current robot,
  slow-mo camera and practice time, zones editable in Panels.

## Next up
- Get answers to the shoot-on-the-move questions, then start with the groundwork.
- Tune: flower wedge positions (FlowerWedgeTest), shooting kP table, UnderShooter values.

## Log
- 2026-11-02: Shooter settings moved to `config/` (CRShooter / UnderShooter, selectable in Panels); flower wedge
  per-servo positions; competition teleop FSM, kP table, gamepad2 controls; turret smoothing; CLAUDE.md and these notes added.
- 2026-11-02: Competition teleop moved onto the Pedro follower and subsystems (Drive rewrite, Intake direct control,
  Shooter.runLutFlywheel, BatteryVoltage util shared with Robot). Driving checked on the robot.
- 2026-11-03: Checked the turret wrap in both turning directions (simulated, symmetric). TurretPollen travel limits
  now leave room for leftTrim / rightTrim so the two servos can't fight at one end.
