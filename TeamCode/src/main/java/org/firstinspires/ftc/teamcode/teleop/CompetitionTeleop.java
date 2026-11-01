package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.BulkReads;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;

import java.util.List;

/**
 * Competition teleop, based on ActuallyEverythingLutTest. RedTeleop and BlueTeleop only pick the alliance.
 * The Shooter aims the turret and sets the hood and flywheel from the LUTs, at the GoalTargets point
 * for the current side.
 *
 * gamepad1 (driver):
 *   left stick / right stick .... drive (field-centric) / turn
 *   right trigger ............... intake (with transfer stall detection)
 *   left trigger ................ outtake
 *   a (hold) .................... open the blocker, closed otherwise (rumbles while it's open)
 *   right bumper (hold) ......... spin the flywheel up to the LUT speed (idles at Shooter.idleRpm otherwise)
 *   share ....................... relocalize to the GoalTargets relocalize pose (configurable, blue mirrored)
 *   dpad down ................... reset heading only (keeps x/y)
 *   dpad up ..................... relocalize x/y to the current goal point (keeps heading)
 *
 * gamepad2 (operator), also works during init:
 *   left bumper / right bumper .. aim at the LEFT / RIGHT goal point
 *   dpad left / dpad right ...... turret offset +3 / -3 degrees (turns the aim left / right)
 *   dpad up / dpad down ......... flywheel offset +15 / -15 rpm
 *   y ........................... reset both offsets to 0
 *
 * Loop time: manual bulk reads (one read per hub per loop), voltage read every
 * Robot.voltageReadIntervalMs, transfer current read every 5th loop and only while intaking, rumble only
 * sent when it changes, and telemetry built only when it is sent (no FTC Dashboard telemetry).
 */
public abstract class CompetitionTeleop extends OpMode {
    /** true for RedTeleop, false for BlueTeleop. */
    protected abstract boolean isRedAlliance();

    // ------------- DRIVER ADJUSTMENTS -------------
    public static final double turretOffsetStepDeg = 3;
    public static final double rpmOffsetStep = 15;

    // ------------- BLOCKER -------------
    public static final double blockerOpen = 0.6;
    public static final double blockerClosed = 0.85;
    private Servo blocker;
    private boolean rumbling = false;

    // ------------- INTAKE -------------
    // Directions checked on the robot: intake reversed, transfer forward.
    public static final boolean transferReversed = false;
    public static final boolean intakeReversed = true;
    public static final double intaking = 1;
    public static final double transferMult = 0.7;
    public static final double intakeStallDraw = 5000; // mA on the transfer = it's full
    public static final int stallCheckEveryLoops = 5;  // current isn't in the bulk read, so don't read it every loop

    private DcMotorEx transferMotor, intakeMotor;
    private boolean transferFull = false;
    private int loopCount = 0;

    // ------------- DRIVETRAIN -------------
    public static final double stickThreshold = 0.05; // for isDriving
    public static final double redDriverHeading = 0;
    public static final double blueDriverHeading = 180;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private boolean isDriving = false;

    // ------------- PINPOINT -------------
    public static final double pinpointXOffsetIn = 2.362;
    public static final double pinpointYOffsetIn = -4.508;

    private GoBildaPinpointDriver pinpoint;
    private Pose startPose;
    private boolean pinpointSeeded = false;
    private final ElapsedTime pinpointTimer = new ElapsedTime();

    // ------------- SHOOTER -------------
    private Shooter shooter;
    private VoltageSensor voltageSensor;
    private double voltage = 0;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private List<LynxModule> hubs;

    @Override
    public void init() {
        hubs = BulkReads.enableManual(hardwareMap); // one bulk read per hub per loop
        GlobalVars.isRed = isRedAlliance();
        GlobalVars.inAuto = false;

        // ----- Blocker -----
        blocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        blocker.setDirection(Servo.Direction.FORWARD);

        // ----- Intake -----
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(transferReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(intakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{transferMotor, intakeMotor}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // ----- Drivetrain -----
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ----- Pinpoint -----
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(pinpointXOffsetIn, pinpointYOffsetIn, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpointTimer.reset(); // pose is seeded 0.5s later, after the IMU recalibrates
        startPose = FollowerPose.takeStartPose(); // auto's end pose, used once; middle of the field, 0 deg if no auto ran

        // ----- Shooter -----
        // transportState is static, so it survives between opmode runs - start from a known state
        Robot.transportState = Robot.TransportState.HOME;
        Shooter.useTargetOverride = false;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooter = new Shooter(hardwareMap); // also resets the offsets and the side to RIGHT

        // ----- Telemetry -----
        // Items are added once and their values are only computed when telemetry is actually sent
        telemetry.setAutoClear(false);
        telemetry.addData("Side", () -> GoalTargets.leftSide ? "LEFT" : "RIGHT");
        telemetry.addData("Turret offset (deg)", () -> String.format("%+.0f", Shooter.turretAngleOffset));
        telemetry.addData("RPM offset", () -> String.format("%+.0f", Shooter.velocityOffsetRpm));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        seedPinpointPose();
        updateDriverAdjustments();
        telemetry.update();
    }

    @Override
    public void loop() {
        BulkReads.clear(hubs); // before any hardware read this loop

        // ================= LOCALIZER =================
        seedPinpointPose(); // no-op once seeded

        if (gamepad1.shareWasPressed()) {
            setFieldPose(GoalTargets.relocalizeX(), GoalTargets.relocalizeY(), GoalTargets.relocalizeHeading());
        }
        if (gamepad1.dpadDownWasPressed()) {
            setFieldPose(FollowerPose.botX, FollowerPose.botY, isRedAlliance() ? 0 : 180);
        }
        if (gamepad1.dpadUpWasPressed()) {
            // Robot is sitting on the goal point: snap x/y there, leave the Pinpoint heading alone
            pinpoint.setPosX(GoalTargets.goalX(), DistanceUnit.INCH);
            pinpoint.setPosY(GoalTargets.goalY(), DistanceUnit.INCH);
        }

        updateLocalizer();

        // ================= OPERATOR =================
        updateDriverAdjustments();

        // ================= DRIVETRAIN =================
        driveMecanum();

        // ================= BLOCKER =================
        boolean blockerIsOpen = gamepad1.a; // open only while a is held
        blocker.setPosition(blockerIsOpen ? blockerOpen : blockerClosed);
        if (blockerIsOpen != rumbling) {
            if (blockerIsOpen) gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling = blockerIsOpen;
        }

        // ================= INTAKE =================
        updateIntake();

        // ================= SHOOTER =================
        Robot.transportState = gamepad1.right_bumper
                ? Robot.TransportState.POWER_SHOOTER
                : Robot.TransportState.HOME;

        // Voltage isn't in the bulk read, so only read it every Robot.voltageReadIntervalMs
        if (voltage == 0 || voltageTimer.milliseconds() >= Robot.voltageReadIntervalMs) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        shooter.update(Robot.nominalVoltage / voltage, isDriving);

        telemetry.update(); // only actually sends every transmission interval
    }

    // ------------------------------------------------------------------
    // Operator (gamepad2)
    // ------------------------------------------------------------------

    private void updateDriverAdjustments() {
        if (gamepad2.leftBumperWasPressed()) GoalTargets.leftSide = true;
        if (gamepad2.rightBumperWasPressed()) GoalTargets.leftSide = false;

        if (gamepad2.dpadLeftWasPressed()) Shooter.turretAngleOffset += turretOffsetStepDeg;
        if (gamepad2.dpadRightWasPressed()) Shooter.turretAngleOffset -= turretOffsetStepDeg;
        if (gamepad2.dpadUpWasPressed()) Shooter.velocityOffsetRpm += rpmOffsetStep;
        if (gamepad2.dpadDownWasPressed()) Shooter.velocityOffsetRpm -= rpmOffsetStep;

        if (gamepad2.yWasPressed()) {
            Shooter.turretAngleOffset = 0;
            Shooter.velocityOffsetRpm = 0;
        }
    }

    // ------------------------------------------------------------------
    // Intake
    // ------------------------------------------------------------------

    private void updateIntake() {
        loopCount = (loopCount + 1) % stallCheckEveryLoops;
        double intakePower, transferPower;

        if (gamepad1.right_trigger > 0.1) {
            intakePower = intaking;
            if (!transferFull && loopCount == 0
                    && transferMotor.getCurrent(CurrentUnit.MILLIAMPS) > intakeStallDraw) {
                transferFull = true;
            }
            transferPower = transferFull ? 0 : intaking * transferMult;
        } else if (gamepad1.left_trigger > 0.1) {
            intakePower = -intaking;
            transferPower = -intaking;
            transferFull = false;
        } else {
            intakePower = 0;
            transferPower = 0;
            transferFull = false;
        }

        transferMotor.setPower(transferPower);
        intakeMotor.setPower(intakePower);
    }

    // ------------------------------------------------------------------
    // Localizer
    // ------------------------------------------------------------------

    /** Half a second after the IMU reset, set the pose auto left (or the middle of the field if no auto ran). */
    private void seedPinpointPose() {
        if (pinpointSeeded || pinpointTimer.seconds() <= 0.5) return;

        pinpoint.update();
        setFieldPose(startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        pinpointSeeded = true;
    }

    private void setFieldPose(double xInches, double yInches, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
    }

    /** Reads the Pinpoint and publishes the pose to the statics the Shooter uses. */
    private void updateLocalizer() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        FollowerPose.botX = pose.getX(DistanceUnit.INCH);
        FollowerPose.botY = pose.getY(DistanceUnit.INCH);
        FollowerPose.botHeading = pose.getHeading(AngleUnit.DEGREES);
    }

    // ------------------------------------------------------------------
    // Drivetrain
    // ------------------------------------------------------------------

    /** Field-centric mecanum drive. */
    private void driveMecanum() {
        double y = -gamepad1.left_stick_y;  // forward
        double x = gamepad1.left_stick_x;   // strafe
        double rx = gamepad1.right_stick_x; // turn

        isDriving = Math.abs(x) > stickThreshold
                || Math.abs(y) > stickThreshold
                || Math.abs(rx) > stickThreshold;

        // Rotate the stick by the robot's heading relative to the driver
        double driverHeading = isRedAlliance() ? redDriverHeading : blueDriverHeading;
        double relativeHeading = Math.toRadians(FollowerPose.botHeading - driverHeading);
        double cos = Math.cos(-relativeHeading);
        double sin = Math.sin(-relativeHeading);
        double rotX = (x * cos - y * sin) * 1.1; // 1.1 counteracts imperfect strafing
        double rotY = x * sin + y * cos;

        // Scale so no wheel power exceeds 1 while keeping the ratios
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        leftFront.setPower((rotY + rotX + rx) / denominator);
        leftRear.setPower((rotY - rotX + rx) / denominator);
        rightFront.setPower((rotY - rotX - rx) / denominator);
        rightRear.setPower((rotY + rotX - rx) / denominator);
    }
}
