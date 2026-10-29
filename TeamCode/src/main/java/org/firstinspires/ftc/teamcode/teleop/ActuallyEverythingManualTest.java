package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.HoodPollen;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.TurretPollen;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;

/**
 * MANUAL shooter test: you set the flywheel velocity (flywheelTarget) and hood position
 * (hoodPosition) from the dashboard, with a Pinpoint localizer and plain mecanum drive.
 * Use it to find good shots, then read "Distance to target" off telemetry and add
 * (distance, velocity) and (distance, hood) points to the LUTs in Shooter.java.
 * Test the finished tables with ActuallyEverythingLutTest.
 *
 * The turret aims from the localizer pose (turretAutoAim). Aim point:
 *   useCustomTarget = false -> exactly what Shooter's turret aims at: goal x for your alliance,
 *                              with y following the robot's y, clamped to 61..83
 *   useCustomTarget = true  -> the fixed field point (targetX, targetY), in inches
 *
 * Controls (gamepad1):
 *   left stick / right stick .... drive / turn
 *   right trigger ............... intake (with transfer stall detection)
 *   left trigger ................ outtake
 *   a ........................... toggle blocker
 *   share ....................... reset pose to your alliance's corner
 *   dpad down ................... reset heading only (keeps x/y)
 */
@TeleOp(name = "ActuallyEverythingManualTest", group = "TeleOp")
@Configurable
public class ActuallyEverythingManualTest extends OpMode {
    //  ------------- BLOCKER -------------

    public Servo blocker;

    //SERVO STATES
    public static final double blockerOpen = 0.6;
    public static final double blockerClosed = 0.85;
    public double blockerPosition = blockerOpen;

    //  ------------- INTAKE -------------

    public DcMotorEx transferMotor;
    public DcMotorEx intakeMotor;
    public double intakePower = dormant;
    public double transferPower = dormant;

    public static double intakeStallDraw = 5000;
    public boolean transferFull = false;
    public double loopCount = 0;

    // Motor directions, matching IntakeTest (known working): transfer reversed, intake forward.
    // NOTE: the original ActuallyEverythingTest had these the other way around. If the intake
    // or transfer spins backwards on your robot, flip the matching constant.
    public static final boolean transferReversed = true;
    public static final boolean intakeReversed = false;

    //MOTOR POWERS
    public static double dormant = 0;
    public static double intaking = 1;
    public double outtaking = intaking * -1;
    public static double transferMult = 0.7;
    public double semiTransferring = intaking * transferMult;


    // ------------- DRIVETRAIN (plain mecanum, no Pedro) -------------
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public boolean isDriving = false;
    public static final double stickThreshold = 0.05; // for isDriving calcs


    // ------------- PINPOINT LOCALIZER -------------
    private GoBildaPinpointDriver pinpoint;
    private boolean pinpointSeeded = false;
    private final ElapsedTime pinpointTimer = new ElapsedTime();

    // EDIT THESE to match your robot (use the same numbers as your Pedro Pinpoint config).
    // Offsets are from the robot's tracking center, in inches (Pinpoint convention):
    //   X offset = how far left (+) of center the X (forward) pod sits
    //   Y offset = how far forward (+) of center the Y (strafe) pod sits
    public static final double pinpointXOffsetIn = 2.362;   // was 60.0 (mm entered as inches)
    public static final double pinpointYOffsetIn = -4.508;  // was -114.5055 (mm entered as inches)

    // ------------- FLYWHEEL (manual velocity) -------------
    Flywheel flywheel;

    public static double flywheelTarget = 0;
    public double flywheelActual = 0;


    // ------------- HOOD (manual position) -------------
    HoodPollen hood;

    public static double hoodPosition = 0;

    public static final double hoodMin = 0;

    public static final double hoodMax = 0.81;


    // ------------- TURRET -------------
    TurretPollen turret;

    public static boolean turretAutoAim = true; // true = aim at the target below, false = use turretPosition
    public static double turretPosition = .5;



    // ------------- AIM TARGET -------------
    public static boolean useCustomTarget = false;
    // Default = center of the field (141.5 x 141.5 in field, so the middle is 70.75, 70.75).
    // Same frame as the localizer: inches, corner origin, 0 deg = +x, counterclockwise positive.
    public static double targetX = 70.75;
    public static double targetY = 70.75;

    private double aimX, aimY;
    private double distanceToTarget;
    private double turretAimDegrees;
    private double turretAimServo;


    // ------------- PIDF CONTROLLER -------------
    public static PIDFController shooterController;

    public static double shooterp = 0.007, shooteri = 0, shooterd = 0, kV = .000400186335403727, kS = .001;
    public static double shooterpid;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ----- Blocker -----
        blocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        blocker.setDirection(Servo.Direction.FORWARD);

        // ----- Intake -----
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(transferReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(intakeReversed ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ----- Drivetrain (EDIT names/directions if they differ from your Pedro Constants) -----
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ----- Pinpoint (EDIT pod type / encoder directions to match your hardware) -----
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(pinpointXOffsetIn, pinpointYOffsetIn, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        pinpointTimer.reset(); // pose is seeded 0.5s later, after the IMU recalibrates (same wait Pedro uses)

        // ----- Shooter pieces -----
        flywheel = new Flywheel(hardwareMap);

        hood = new HoodPollen(hardwareMap);

        turret = new TurretPollen(hardwareMap);

        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);
    }

    @Override
    public void init_loop() {
        seedPinpointPose();
        telemetry.addData("Pinpoint pose seeded", pinpointSeeded);
        telemetry.update();
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        // ================= LOCALIZER =================
        seedPinpointPose(); // no-op once seeded

        if (gamepad1.shareWasPressed()) {
            if (isRed) setFieldPose(70.75, 70.75, 0);
            else setFieldPose(31.479822835, 136.4, 90);
        }
        if (gamepad1.dpadDownWasPressed()) {
            setFieldPose(FollowerPose.botX, FollowerPose.botY, isRed ? 0 : 180);
        }

        updateLocalizer();

        // ================= DRIVETRAIN =================
        driveMecanum();

        // ================= BLOCKER =================
        loopCount++;
        loopCount %= 5;
        if (gamepad1.aWasPressed()) {
            if (blockerPosition == blockerOpen) {
                blockerPosition = blockerClosed;
            } else {
                blockerPosition = blockerOpen;
            }
        }
        blocker.setPosition(blockerPosition);


        // ================= INTAKE =================
        outtaking = intaking * -1;
        semiTransferring = intaking * transferMult;


        if (gamepad1.right_trigger > 0.1) {
            intakePower = intaking;
            if (!transferFull){
                if (loopCount == 1) {
                    if (transferMotor.getCurrent(CurrentUnit.MILLIAMPS) > intakeStallDraw) {
                        transferPower = 0;
                        transferFull = true;
                    } else {
                        transferPower = semiTransferring;
                    }
                } else {
                    transferPower = semiTransferring;
                }
            } else {
                transferPower = 0;
            }
        } else if (gamepad1.left_trigger > 0.1) {
            transferPower = outtaking;
            intakePower = outtaking;
            transferFull = false;
        } else {
            intakePower = 0;
            transferPower = 0;
            transferFull = false;
        }

        transferMotor.setPower(transferPower);
        intakeMotor.setPower(intakePower);


        // ================= AIM =================
        updateAim();

        // ================= SHOOTER (manual values) =================
        flywheelActual = flywheel.getVelocity();
        shooterController.setPIDF(shooterp, shooteri, shooterd, kV);
        shooterpid = shooterController.calculate(flywheelActual, flywheelTarget) + (Math.signum(flywheelTarget) * kS);
        flywheel.update(Range.clip(shooterpid, -1.0, 1.0));

        hood.update(Range.clip(hoodPosition, hoodMin, hoodMax));

        double turretCommand = turretAutoAim ? turretAimServo : turretPosition;
        turret.update(turretCommand); // clamped to TurretPollen's travel limits


        // ================= TELEMETRY =================
        telemetry.addData("Distance to target (in)", distanceToTarget);
        telemetry.addData("Flywheel velocity", flywheelActual);
        telemetry.addData("Flywheel target", flywheelTarget);
        telemetry.addLine();

        telemetry.addLine("VALUES:");
        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Transfer power: ", transferPower);
        telemetry.addData("Transfer current: ", transferMotor.getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.addLine("LOCALIZER:");
        telemetry.addData("Pinpoint pose seeded", pinpointSeeded);
        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.addData("x (in)", FollowerPose.botX);
        telemetry.addData("y (in)", FollowerPose.botY);
        telemetry.addData("heading (deg)", FollowerPose.botHeading);
        telemetry.addData("isDriving", isDriving);

        telemetry.addLine("AIM:");
        telemetry.addData("Aiming at", useCustomTarget ? "custom target" : "goal");
        telemetry.addData("Target (x, y) in", "%.1f, %.1f", aimX, aimY);
        telemetry.addData("Turret auto-aim", turretAutoAim);
        telemetry.addData("Turret angle (deg)", turretAimDegrees);
        telemetry.addData("Turret servo cmd", turret.getCommandedPosition());

        telemetry.addLine("SHOOTER (manual):");
        telemetry.addData("Flywheel error", flywheelTarget - flywheelActual);
        telemetry.addData("Hood position (clamped)", Range.clip(hoodPosition, hoodMin, hoodMax));

        telemetry.update();
    }

    // ------------------------------------------------------------------
    // Aiming
    // ------------------------------------------------------------------

    /** Picks the aim point and computes distance + turret servo position from the localizer pose. */
    private void updateAim() {
        if (useCustomTarget) {
            aimX = targetX;
            aimY = targetY;
        } else {
            // Same point Shooter aims at
            aimX = isRed ? Shooter.redGoalX : Shooter.blueGoalX;
            aimY = Range.clip(FollowerPose.botY, Shooter.goalMinY, Shooter.goalMaxY);
        }

        double deltaX = aimX - FollowerPose.botX;
        double deltaY = aimY - FollowerPose.botY;
        distanceToTarget = Math.hypot(deltaX, deltaY);

        // Same math as Shooter.turretTargetToGoal(): angle to target minus robot heading,
        // wrapped, then mapped onto the servo with TurretPollen's calibration (it clamps to the travel limits).
        double goalAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        turretAimDegrees = angleWrap(goalAngle - FollowerPose.botHeading);
        turretAimServo = TurretPollen.angleToPosition(turretAimDegrees);
    }

    private static double angleWrap(double degrees) {
        while (degrees > 180)  degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    // ------------------------------------------------------------------
    // Localizer helpers
    // ------------------------------------------------------------------

    /**
     * Half a second after the IMU reset, tell the Pinpoint where we are in the same field frame
     * the Shooter's goal math uses (inches, origin at field corner, heading 0 = +x). Using
     * FollowerPose.startPose keeps this consistent with the rest of your code.
     */
    private void seedPinpointPose() {
        if (pinpointSeeded) return;

        if (pinpointTimer.seconds() > 0.5) {
            pinpoint.update();
            setFieldPose(
                    FollowerPose.startPose.x(),
                    FollowerPose.startPose.y(),
                    Math.toDegrees(FollowerPose.startPose.heading()));
            pinpointSeeded = true;
        }
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

    /** Standard robot-centric mecanum drive. */
    private void driveMecanum() {
        double y = -gamepad1.left_stick_y;       // forward
        double x = gamepad1.left_stick_x * 1.1;  // strafe (1.1 counteracts imperfect strafing)
        double rx = gamepad1.right_stick_x;      // turn

        isDriving = Math.abs(gamepad1.left_stick_x) > stickThreshold
                || Math.abs(gamepad1.left_stick_y) > stickThreshold
                || Math.abs(gamepad1.right_stick_x) > stickThreshold;

        // Scale so no wheel power exceeds 1 while keeping the ratios
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftFront.setPower((y + x + rx) / denominator);
        leftRear.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightRear.setPower((y + x - rx) / denominator);
    }
}