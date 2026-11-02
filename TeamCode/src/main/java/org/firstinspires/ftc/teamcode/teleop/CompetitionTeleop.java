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
import org.firstinspires.ftc.teamcode.teleop.systems.Intake;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.BulkReads;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;

import java.util.List;

/**
 * Competition teleop. RedTeleop and BlueTeleop only pick the alliance.
 * The Shooter aims the turret and sets the hood from the LUTs, at the GoalTargets point for the current side.
 * The flywheel always targets the LUT speed for the current distance (plus the rpm offset), so it's ready to shoot.
 *
 * States (picked from gamepad1 every loop):
 *   SHOOTING .. gate open (right bumper held) AND right trigger: intake + transfer feed at shootTransferPower,
 *               no transfer stall check, flywheel P from Shooter.lutShootingKps (by distance), or
 *               Shooter.shooterP if Shooter.useShootingKpLut is false
 *   INTAKING .. right trigger (gate closed): intake + transfer with the stall check, flywheel P = Shooter.shooterP
 *   IDLING .... anything else (just driving): left trigger outtakes, flywheel P = Shooter.shooterP
 *   The flywheel's I, D, kV and kS are Shooter's in every state.
 *
 * gamepad1 (driver):
 *   left stick / right stick .... drive (field-centric) / turn
 *   right trigger ............... intake, or shoot while the gate is open
 *   left trigger ................ outtake
 *   right bumper (hold) ......... open the gate (rumbles while it's open)
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
 * gamepad1 x / b ................ aim at the LEFT / RIGHT goal point
 *
 * Loop time: manual bulk reads (one read per hub per loop), voltage read every
 * Robot.voltageReadIntervalMs, transfer current read every 5th loop and only while intaking, rumble only
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

    // ------------- GATE (blocker) -------------
    public static final double gateOpen = 0.6;
    public static final double gateClosed = 0.85;
    private Servo gate;
    private boolean rumbling = false;

    // ------------- FLOWER WEDGE -------------
    // Positions come from Intake (left/right WedgeUp / WedgeDown, editable in Panels). Starts up.
    private Servo flowerWedgeLeft, flowerWedgeRight;
    private boolean flowerWedgeIsDown = false;

    // ------------- INTAKE / TRANSFER -------------
    // Directions checked on the robot: intake reversed, transfer forward.
    public static final boolean transferReversed = false;
    public static final boolean intakeReversed = true;
    public static final double intakePower = 1;
    public static final double intakeShootPower = .95;
    public static final double intakeTransferPower = 0.7; // transfer while intaking (stops when it stalls)
    public static final double shootTransferPower = .72;    // transfer while shooting (no stall check)
    public static final double outtakePower = -1;
    public static final double intakeStallDraw = 7000;    // mA on the transfer = it's full
    public static final int stallCheckEveryLoops = 5;     // current isn't in the bulk read, so don't read it every loop

    private DcMotorEx transferMotor, intakeMotor;
    private boolean transferFull = false;
    private int loopCount = 0;

    // ------------- DRIVETRAIN -------------
    public static final double triggerThreshold = 0.1;
    public static final double redDriverHeading = 0;
    public static final double blueDriverHeading = 180;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

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

        // ----- Gate -----
        gate = hardwareMap.get(Servo.class, "transferPollenBlocker");
        gate.setDirection(Servo.Direction.FORWARD);

        // ----- Flower wedge -----
        if (Intake.flowerWedgeEnabled) {
            flowerWedgeLeft = hardwareMap.get(Servo.class, "flowerWedgeLeft");
            flowerWedgeRight = hardwareMap.get(Servo.class, "flowerWedgeRight");
            flowerWedgeRight.setDirection(Servo.Direction.REVERSE);
        }

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
        updateOperator();
        telemetry.update();
    }

    @Override
    public void loop() {
        BulkReads.clear(hubs); // before any hardware read this loop

        // ================= LOCALIZER =================
        seedPinpointPose(); // no-op once seeded

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
        updateOperator();

        // ================= DRIVETRAIN =================
        driveMecanum();

        // ================= GATE =================
        boolean gateIsOpen = gamepad1.right_bumper; // open only while held
        gate.setPosition(gateIsOpen ? gateOpen : gateClosed);
        if (gateIsOpen != rumbling) {
            if (gateIsOpen) gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling = gateIsOpen;
        }

        // ================= STATE =================
        boolean rightTrigger = gamepad1.right_trigger > triggerThreshold;
        if (gateIsOpen && rightTrigger) state = State.SHOOTING;
        else if (rightTrigger) state = State.INTAKING;
        else state = State.IDLING;

        // ================= SHOOTER AIM =================
        shooter.updateAim(); // turret, hood and the LUT lookups, in every state

        // Voltage isn't in the bulk read, so only read it every Robot.voltageReadIntervalMs
        if (voltage == 0 || voltageTimer.milliseconds() >= Robot.voltageReadIntervalMs) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        double voltageMultiplier = Robot.nominalVoltage / voltage;
        double flywheelTarget = shooter.lutTargetVelocity();

        // ================= STATE ACTIONS =================
        double intake, transfer;
        switch (state) {
            case SHOOTING:
                // Feed everything into the flywheel: no stall check, the transfer is emptying
                intake = intakeShootPower;
                transfer = shootTransferPower;
                transferFull = false;
                double shootingP = Shooter.useShootingKpLut ? Shooter.lutShootingKp : Shooter.shooterP;
                shooter.runFlywheel(flywheelTarget, shootingP, voltageMultiplier);
                break;

            case INTAKING:
                loopCount = (loopCount + 1) % stallCheckEveryLoops;
                if (!transferFull && loopCount == 0
                        && transferMotor.getCurrent(CurrentUnit.MILLIAMPS) > intakeStallDraw) {
                    transferFull = true;
                }
                intake = intakePower;
                transfer = transferFull ? 0 : intakeTransferPower;
                shooter.runFlywheel(flywheelTarget, Shooter.shooterP, voltageMultiplier);
                break;

            case IDLING:
            default:
                boolean outtaking = gamepad1.left_trigger > triggerThreshold;
                intake = outtaking ? outtakePower : 0;
                transfer = outtaking ? outtakePower : 0;
                if (outtaking) transferFull = false; // balls pushed back out
                shooter.runFlywheel(flywheelTarget, Shooter.shooterP, voltageMultiplier);
                break;
        }
        intakeMotor.setPower(intake);
        transferMotor.setPower(transfer);

        // ================= FLOWER WEDGE =================
        if (Intake.flowerWedgeEnabled) {
            Intake.setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, flowerWedgeIsDown);
        }

        telemetry.update(); // only actually sends every transmission interval
    }

    // ------------------------------------------------------------------
    // Operator (gamepad2)
    // ------------------------------------------------------------------

    private void updateOperator() {
        if (gamepad2.startWasPressed()) {
            setFieldPose(GoalTargets.relocalizeX(), GoalTargets.relocalizeY(), GoalTargets.relocalizeHeading());
        }

        // Heading only: robot's back flat against a wall that runs along x, facing +y (90 deg, a field heading,
        // so the same for both alliances). x/y are left alone.
        if (gamepad2.dpadUpWasPressed()) pinpoint.setHeading(90, AngleUnit.DEGREES);

        if (gamepad2.rightBumperWasPressed()) Shooter.velocityOffsetRpm += rpmOffsetStep;
        if (gamepad2.leftBumperWasPressed()) Shooter.velocityOffsetRpm -= rpmOffsetStep;
        // Turret angles are counterclockwise positive, so right is negative
        if (gamepad2.dpadRightWasPressed()) Shooter.turretAngleOffset -= turretOffsetStepDeg;
        if (gamepad2.dpadLeftWasPressed()) Shooter.turretAngleOffset += turretOffsetStepDeg;

        if (gamepad2.aWasPressed()) {
            Shooter.turretAngleOffset = 0;
            Shooter.velocityOffsetRpm = 0;
        }

        if (gamepad2.yWasPressed()) flowerWedgeIsDown = !flowerWedgeIsDown; // servos are set in loop()

        if (gamepad1.xWasPressed()) GoalTargets.leftSide = true;
        if (gamepad1.bWasPressed()) GoalTargets.leftSide = false;
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
