package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Intake;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.LimelightAim;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.ShootOnTheMove;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.BatteryVoltage;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.BulkReads;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;

import java.util.List;

/**
 * Competition teleop. RedTeleop and BlueTeleop only pick the alliance.
 *
 * Subsystems:
 *   Drive ..... Pedro follower: field-centric driving + Pinpoint localization (pedro/Constants). Starts at the
 *               pose the auto ended at, or the middle of the field at 0 deg if no auto ran.
 *   Intake .... intake / transfer rollers, gate, flower wedge
 *   Shooter ... turret + hood aimed from the LUTs at the GoalTargets point for the current side; flywheel always at
 *               the LUT speed for the current distance (plus the rpm offset), so it's ready to shoot.
 *               Inside a LimelightAim zone with the tag in view, the turret angle and LUT distance come from the
 *               Limelight instead of odometry ("Aim" line in telemetry)
 *
 * States (picked from gamepad1 every loop):
 *   SHOOTING .. gate open (right bumper held) AND right trigger: Intake.runFeed (no transfer stall check),
 *               flywheel P from the shooting kP table (Shooter.config().lutShootingKps) if useShootingKpLut
 *   INTAKING .. right trigger (gate closed): Intake.runIntake (stops the transfer when it stalls), flywheel P = shooterP
 *   IDLING .... anything else (just driving): left trigger outtakes, flywheel P = shooterP
 *
 * gamepad1 (driver):
 *   left stick / right stick .... drive (field-centric) / turn
 *   right trigger ............... intake, or shoot while the gate is open
 *   left trigger ................ outtake
 *   right bumper (hold) ......... open the gate (rumbles while it's open)
 *   x / b ....................... aim at the LEFT / RIGHT goal point (also works during init)
 *   dpad down ................... reset heading only (keeps x/y)
 *   dpad up ..................... relocalize x/y to the current goal point (keeps heading)
 *
 * gamepad2 (operator), also works during init:
 *   start ....................... relocalize to the GoalTargets relocalize pose (configurable, blue mirrored)
 *   right bumper / left bumper .. flywheel offset +10 / -10 rpm
 *   dpad right / dpad left ...... turret offset 3 degrees right / left
 *   dpad up ..................... heading only: back against a wall, facing +y (heading = 90 deg)
 *   a ........................... reset all offsets to 0
 *   y ........................... toggle the flower wedge up / down (positions: Intake left/right WedgeUp / WedgeDown)
 *
 * Loop time: manual bulk reads (one read per hub per loop), battery voltage read every
 * Robot.voltageReadIntervalMs, transfer current read every few loops and only while intaking, rumble only
 * sent when it changes, and telemetry built only when it is sent (no FTC Dashboard telemetry).
 */
public abstract class CompetitionTeleop extends OpMode {
    /** true for RedTeleop, false for BlueTeleop. */
    protected abstract boolean isRedAlliance();

    public enum State { INTAKING, IDLING, SHOOTING }
    private State state = State.IDLING;

    // ------------- DRIVER ADJUSTMENTS -------------
    public static final double turretOffsetStepDeg = 3;
    public static final double rpmOffsetStep = 10;
    public static final double wallHeadingDeg = 90; // gamepad2 dpad up: back against a wall, facing +y
    public static final double triggerThreshold = 0.1;

    // ------------- SUBSYSTEMS -------------
    private List<LynxModule> hubs;
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    private BatteryVoltage battery;

    private boolean rumbling = false;
    private boolean flowerWedgeDown = false;

    @Override
    public void init() {
        hubs = BulkReads.enableManual(hardwareMap); // one bulk read per hub per loop
        GlobalVars.isRed = isRedAlliance();
        GlobalVars.inAuto = false;
        Shooter.useTargetOverride = false;

        drive = new Drive(hardwareMap, true, true);
        intake = new Intake(hardwareMap);   // gate closed, wedge up
        shooter = new Shooter(hardwareMap); // also resets the offsets and the side to RIGHT
        battery = new BatteryVoltage(hardwareMap);

        // Items are added once and their values are only computed when telemetry is actually sent
        telemetry.setAutoClear(false);
        telemetry.addData("Side", () -> GoalTargets.leftSide ? "LEFT" : "RIGHT");
        telemetry.addData("Turret offset (deg)", () -> String.format("%+.0f", Shooter.turretAngleOffset));
        telemetry.addData("RPM offset", () -> String.format("%+.0f", Shooter.velocityOffsetRpm));
        telemetry.addData("SOTM vel x/y (in/s) turn (deg/s)", () -> String.format("%.1f / %.1f / %.0f %s",
                ShootOnTheMove.velX, ShootOnTheMove.velY, ShootOnTheMove.turnRate,
                ShootOnTheMove.usingPinpoint ? "pinpoint" : "pose diff"));
        telemetry.addData("SOTM aim shift (deg) / dist shift (in)", () -> String.format("%+.1f / %+.1f",
                ShootOnTheMove.aimShiftDeg, ShootOnTheMove.distanceShift));
        telemetry.addData("Aim", () -> LimelightAim.active ? "LIMELIGHT" : "odometry (" + LimelightAim.status + ")");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateControls();
        telemetry.update();
    }

    @Override
    public void loop() {
        BulkReads.clear(hubs); // before any hardware read this loop

        // ================= DRIVER BUTTONS + OPERATOR =================
        if (gamepad1.dpadDownWasPressed()) drive.resetHeading();
        // Robot is sitting on the goal point: snap x/y there, leave the heading alone
        if (gamepad1.dpadUpWasPressed()) drive.setPosition(GoalTargets.goalX(), GoalTargets.goalY());
        updateControls();

        // ================= DRIVE (also updates the pose the Shooter aims from) =================
        drive.drive(gamepad1);

        // ================= GATE =================
        boolean gateOpen = gamepad1.right_bumper; // open only while held
        intake.setGateOpen(gateOpen);
        rumbleWhile(gateOpen);

        // ================= SHOOTING SIDE =================
        // Automatically change the target based on which side the robot is on
        if (FollowerPose.botY < 58) {
            GoalTargets.leftSide = false;
        } else if (FollowerPose.botY > 83) {
            GoalTargets.leftSide = true;
        }

        // ================= STATE =================
        boolean rightTrigger = gamepad1.right_trigger > triggerThreshold;
        if (gateOpen && rightTrigger) state = State.SHOOTING;
        else if (rightTrigger) state = State.INTAKING;
        else state = State.IDLING;

        switch (state) {
            case SHOOTING:
                intake.runFeed();
                break;
            case INTAKING:
                intake.runIntake();
                break;
            case IDLING:
            default:
                if (gamepad1.left_trigger > triggerThreshold) intake.runOuttake();
                else intake.stopRollers();
                break;
        }
        intake.setFlowerWedgeDown(flowerWedgeDown);

        // ================= SHOOTER =================
        shooter.updateAim(); // turret, hood and the LUT lookups, in every state
        shooter.runLutFlywheel(state == State.SHOOTING, battery.multiplier());

        telemetry.update(); // only actually sends every transmission interval
    }

    // ------------------------------------------------------------------
    // Controls that also work during init
    // ------------------------------------------------------------------

    private void updateControls() {
        // ----- gamepad1: goal side -----
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) GoalTargets.leftSide = true;
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) GoalTargets.leftSide = false;

        // ----- gamepad2: relocalization -----
        if (gamepad2.startWasPressed()) drive.relocalize();
        if (gamepad2.dpadUpWasPressed()) drive.setHeading(wallHeadingDeg); // field heading, same for both alliances

        // ----- gamepad2: offsets -----
        if (gamepad2.rightBumperWasPressed()) Shooter.velocityOffsetRpm += rpmOffsetStep;
        if (gamepad2.leftBumperWasPressed()) Shooter.velocityOffsetRpm -= rpmOffsetStep;
        // Turret angles are counterclockwise positive, so right is negative
        if (gamepad2.dpadRightWasPressed()) Shooter.turretAngleOffset -= turretOffsetStepDeg;
        if (gamepad2.dpadLeftWasPressed()) Shooter.turretAngleOffset += turretOffsetStepDeg;
        if (gamepad2.aWasPressed()) {
            Shooter.turretAngleOffset = 0;
            Shooter.velocityOffsetRpm = 0;
        }

        // ----- gamepad2: flower wedge (the servos are set in loop()) -----
        if (gamepad2.yWasPressed() || gamepad1.yWasPressed()) flowerWedgeDown = !flowerWedgeDown;
    }

    /** Rumbles gamepad1 the whole time `on` is true, only sending a command when it changes. */
    private void rumbleWhile(boolean on) {
        if (on == rumbling) return;
        if (on) gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        else gamepad1.stopRumble();
        rumbling = on;
    }
}
