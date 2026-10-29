package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;

/**
 * LUT shooter test: the real Shooter class sets the turret, hood and flywheel velocity from the
 * localizer pose using its InterpLUTs (Shooter.velocityLut / Shooter.hoodAngleLut). The LUTs are
 * SafeLuts, so a distance outside the table is clamped to the nearest end instead of throwing.
 * Telemetry shows the table range and whether the current distance is clamped.
 *
 * Aim point:
 *   useCustomTarget = false -> the goal, exactly as the Shooter class computes it
 *   useCustomTarget = true  -> the fixed field point (targetX, targetY), in inches, used for turret
 *                              aim AND for the distance fed to both LUTs
 *
 * Controls (gamepad1):
 *   left stick / right stick .... drive / turn
 *   right trigger ............... intake (with transfer stall detection)
 *   left trigger ................ outtake
 *   a ........................... toggle blocker
 *   right bumper (hold) ......... POWER_SHOOTER: flywheel to full LUT speed for the current distance
 *   share ....................... reset pose to your alliance's corner
 *   dpad down ................... reset heading only (keeps x/y)
 *
 * Notes:
 *   - Shooter.update zeroes the flywheel P and kV terms while isDriving is true, so the flywheel
 *     only spins up when the drive sticks are released.
 *   - Even without the bumper the Shooter keeps the flywheel idling at the LUT speed (that is how
 *     the real robot behaves), so expect it to spin once the velocity LUT has real data.
 */
@TeleOp(name = "ActuallyEverythingLutTest", group = "TeleOp")
@Configurable
public class ActuallyEverythingLutTest extends OpMode {
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
    public static final double pinpointXOffsetIn = 2.187;
    public static final double pinpointYOffsetIn = -4.572;

    // ------------- SHOOTER CLASS (turret / hood / flywheel from the LUTs) -------------
    Shooter shooter;
    VoltageSensor voltageSensor;


    // ------------- AIM TARGET -------------
    public static boolean useCustomTarget = false;
    // Default = center of the field (141.5 x 141.5 in field, so the middle is 70.75, 70.75).
    // Same frame as the localizer: inches, corner origin, 0 deg = +x, counterclockwise positive.
    public static double targetX = 70.75;
    public static double targetY = 70.75;


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
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpointTimer.reset(); // pose is seeded 0.5s later, after the IMU recalibrates (same wait Pedro uses)

        // ----- Shooter -----
        // transportState is static, so it survives between opmode runs - start from a known state
        Robot.transportState = Robot.TransportState.HOME;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter = new Shooter(hardwareMap);
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
            if (isRed) setFieldPose(113.520177165, 136.4, 90);
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


        // ================= SHOOTER (LUT) =================
        // Aim-point override: Shooter uses this for turret aim, hood angle and flywheel velocity
        Shooter.useTargetOverride = useCustomTarget;
        Shooter.targetOverrideX = targetX;
        Shooter.targetOverrideY = targetY;

        // Shooter's flywheel target comes from Robot.transportState: hold right bumper for a full-speed shot
        Robot.transportState = gamepad1.right_bumper
                ? Robot.TransportState.POWER_SHOOTER
                : Robot.TransportState.HOME;

        double voltageMultiplier = Robot.nominalVoltage / voltageSensor.getVoltage();
        shooter.update(voltageMultiplier, isDriving);


        // ================= TELEMETRY =================
        telemetry.addData("Distance to target (in)", Shooter.distanceFromLimelightToTagInches);
        telemetry.addData("Flywheel velocity", Shooter.shooterVelocity);
        telemetry.addData("Flywheel target", Shooter.shooterVelocityTarget);
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

        double distance = Shooter.distanceFromLimelightToTagInches;

        telemetry.addLine("SHOOTER (LUT):");
        telemetry.addData("Transport state", Robot.transportState);
        telemetry.addData("Aiming at", useCustomTarget ? "custom target" : "goal");
        telemetry.addData("Turret servo target", Shooter.turretTargetPositionPollen);
        telemetry.addData("Hood target", Shooter.hoodPollenPosition);
        telemetry.addData("In range (ready to fire)", shooter.inRange());

        telemetry.addLine("LUT LIMITS:");
        telemetry.addData("Velocity LUT points", shooter.velocityLut.size());
        telemetry.addData("Velocity LUT range (in)", "%.1f to %.1f", shooter.velocityLut.minX(), shooter.velocityLut.maxX());
        telemetry.addData("Velocity LUT clamped", !shooter.velocityLut.isInRange(distance));
        telemetry.addData("Hood LUT points", shooter.hoodAngleLut.size());
        telemetry.addData("Hood LUT range (in)", "%.1f to %.1f", shooter.hoodAngleLut.minX(), shooter.hoodAngleLut.maxX());
        telemetry.addData("Hood LUT clamped", !shooter.hoodAngleLut.isInRange(distance));

        telemetry.update();
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