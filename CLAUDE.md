# CLAUDE.md

FTC robot code. Team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
Session notes (current state, open issues, plans) are in [docs/NOTES.md](docs/NOTES.md): read it at the
start of a session and update it at the end.

## Stack
- FTC SDK 11.2.1, Pedro Pathing 3.0 (with Foresight braking model), FTCLib, Panels (`@Configurable`), FTC Dashboard.
- Remote: https://github.com/marc-j-morillo/quickstart. Working branch: `robot-fixes-and-tuners`, PR #1 into `master`.
- Teammate Marc also pushes to this branch: fetch and rebase before pushing.

## Build
From the repo root, with `JAVA_HOME` pointing at a JDK 21 (Android Studio's bundled `jbr` works):

```bash
./gradlew :TeamCode:compileDebugJavaWithJavac -q
```

- Gradle wrapper is 8.14.6. No machine-specific path is committed in `gradle.properties`, so set `JAVA_HOME`
  (or Android Studio's Gradle JDK) per machine. On the Windows desktop: `C:/Users/joshu/.jdks/jbr-21.0.11`.
- Compile after every change. Nothing can be run on the robot from here, so always say what is untested on the robot.

## Working rules (user preferences)
- Commit or push only when asked. Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`;
  PR descriptions end with `🤖 Generated with [Claude Code](https://claude.com/claude-code)`.
- "Don't change anything" / "just talk" means no edits that turn: answer only.
- Don't commit machine-specific build settings (e.g. `org.gradle.java.home`).
- The user is a student: explain plainly, give numbers, flag anything that will behave differently on the robot.
- Match the surrounding code style and comment density. Tunables are `public static` in a `@Configurable` class
  so they can be changed live in Panels.

## Code map
- `teleop/systems/Shooter/Shooter.java`: turret aim, hood, LUT lookups (`updateAim`), flywheel PIDF
  (`update` for the Robot/transport-state flow, `runFlywheel` for the competition teleop). All tunables come from
  `Shooter.config()`, which returns the set picked by `Shooter.activeConfig` (Panels: `CR_SHOOTER` / `UNDER_SHOOTER`).
- `teleop/systems/Shooter/config/`: `ShooterConfig` lists every shooter setting (instance fields);
  `CRShooter` and `UnderShooter` each hold a full set in `values`. Panels only shows fields a class declares itself,
  so these are separate holders of one `ShooterConfig` type, not subclasses.
- `teleop/systems/Shooter/TurretPollen.java`: positional Axon turret (two servos). `angle = (pos - centerPosition) * degreesPerPosition`
  (center 0.5, -345 deg/position, PWM 500-2500 us). `aimAt` picks the reachable equivalent angle and holds the
  nearest limit in the dead zone. Output smoothing (`smoothingTimeMs`), `rightServoEnabled`, output on/off.
- `teleop/systems/Shooter/ShootOnTheMove.java` (branch `shoot-on-the-move`): run by `updateAim`, corrects the aim for robot
  motion (lead by turret lag + virtual goal = goal - velocity x time of flight). Velocity from `FollowerPose.velX/velY/turnRateDeg`
  (Pinpoint, field frame) or pose differences. Time of flight table: `ShooterConfig.lutTimesOfFlight`.
- `teleop/systems/utils/GoalTargets.java`: red goal points (RIGHT / LEFT side), relocalize pose. Blue mirrors red:
  x -> 141.5 - x, heading -> 180 - heading.
- `teleop/systems/Drive.java`: the one Pedro follower (drivetrain + Pinpoint localizer from `pedro/Constants`).
  `drive(gamepad)` = field-centric teleop drive + follower update + `FollowerPose`; relocalize helpers
  (`setPose`, `setPosition`, `setHeading`, `resetHeading`, `relocalize`). Autos get it via `Robot.getFollower()`.
  Pedro `DrivePowers` are (forward, strafe + = left, turn + = counterclockwise), so stick x axes are negated.
- `teleop/systems/Intake.java`: intake/transfer, gate, flower wedge (per-servo `left/rightWedgeUp/Down`).
  `update(...)` runs the Robot transport FSM (autos); `runIntake` / `runFeed` / `runOuttake` / `stopRollers` /
  `setGateOpen` / `setFlowerWedgeDown` are direct control for the competition teleop (with the transfer stall check).
- `teleop/CompetitionTeleop.java` (+ `RedTeleop`, `BlueTeleop`): competition teleop built on Drive / Intake /
  Shooter / BatteryVoltage. FSM INTAKING / IDLING / SHOOTING, controls listed in the class comment.
- Test opmodes: `ActuallyEverythingLutTest` (real Shooter + LUTs), `ActuallyEverythingManualTest` (manual flywheel/hood),
  `LutTuner`, `TurretHeadingHold`, `TurretServoTuner`, `teleop/hardwaretests/*` (no subsystems: motor directions,
  turret, flower wedge, shooter motors).
- `teleop/systems/utils/`: `FollowerPose` (static pose; `takeStartPose()` uses the auto's end pose once),
  `BulkReads` (manual bulk caching), `BatteryVoltage` (throttled voltage + feedforward multiplier), `LoopTimer`,
  `StaticDefaults` (records every TeamCode static at app start via `@OnCreate`; `resetAll()` puts them back).
- `teleop/ResetStaticValues`: "Reset Static Values" opmode, resets all TeamCode statics on INIT (uses `StaticDefaults`).
- The LUT test and ManualTest still use their own raw Pinpoint + mecanum code (with different pod offsets than Pedro's).
- `pedro/Constants.java`: Pedro config, including tuned Foresight braking coefficients. Autos are in `pedro/auto/`.

## Conventions
- Field frame: inches, origin at a field corner, heading in degrees, 0 = +x, counterclockwise positive. Field is 141.5 in.
- Turret angles: degrees, 0 = robot forward, counterclockwise positive.
- Flywheel speeds are motor encoder ticks/s (28 ticks per motor rev). CRShooter has a 2:1 reduction to the wheel.
- Motor directions checked on the robot: drive left FORWARD / right REVERSE, intake REVERSE, transfer FORWARD,
  both shooter motors inverted (same mounting).

## Hardware config names
- Motors: `frontLeft`, `frontRight`, `backLeft`, `backRight`, `intakeMotor`, `transferMotor`, `shooter1`, `shooter2`
  (flywheel velocity is read from `shooter1`'s encoder).
- Servos: `turretPollenLeft`, `turretPollenRight`, `hoodPollen`, `transferPollenBlocker` (gate), `flowerWedgeLeft`, `flowerWedgeRight`.
- I2C: `pinpoint`. Unused/legacy names still in code: `turretEncoder`, `turretNectarLeft/Right`, `transferNectar`, `led`.
