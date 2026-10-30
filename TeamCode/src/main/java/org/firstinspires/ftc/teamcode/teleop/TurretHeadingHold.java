package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.TurretPollen;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Turret heading-hold test: keeps the turret pointing the way the robot faced at start, using only the
 * Pinpoint heading (no x / y, no goal). Turn the robot (drive or by hand) and the turret should stay locked
 * on the same direction. Use it to check the turret direction, degreesPerPosition and centerPosition:
 *   - turret swings WITH the robot, about twice as far ......... degreesPerPosition has the wrong sign
 *   - turret lags, drifting a bit the way the robot turned ...... |degreesPerPosition| is too big, make it smaller
 *   - turret overshoots, turning further than the robot did ..... |degreesPerPosition| is too small, make it bigger
 *   - turret is off by the same amount at every heading ......... forward isn't at servo 0.5, see below
 *
 * Forward is set mechanically: servo position 0.5 (centerPosition) = turret facing robot-forward, by
 * re-meshing the gears. Press b for CENTER CHECK, which holds both servos at exactly 0.5 so you can line
 * the shooter up with the robot and move the gear a tooth if it's off. If it's off by less than a tooth,
 * nudge centerPosition in Panels instead (each 0.01 is |degreesPerPosition| / 100 degrees).
 *
 * Uses TurretPollen.aimAt, so past the travel limits it holds the nearest limit instead of flipping.
 *
 * Controls (gamepad1):
 *   left stick / right stick .... drive / turn (robot-centric)
 *   b ........................... toggle CENTER CHECK (servos at exactly 0.5) / HOLD
 *   a ........................... re-zero: the way the robot faces now becomes "forward"
 *   dpad left / right ........... rotate the held direction +/- offsetStep (left = counterclockwise)
 *   y ........................... reset the offset to 0
 */
@TeleOp(name = "TurretHeadingHold", group = "Tuning")
@Configurable
public class TurretHeadingHold extends OpMode {
    public static boolean driveEnabled = true;
    public static double offsetDegrees = 0;  // held direction relative to the start heading, counterclockwise positive
    public static double offsetStep = 15;

    private GoBildaPinpointDriver pinpoint;
    private TurretPollen turret;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private double zeroHeading;
    private boolean centerCheck = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Same Pinpoint settings as ActuallyEverythingManualTest / Pedro Constants
        pinpoint.setOffsets(2.362, -4.508, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU(); // keep the robot still for a moment during init

        turret = new TurretPollen(hardwareMap);

        // Same names / directions as Pedro Constants
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
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        telemetry.addLine("Point the robot the way the turret should hold, then press start.");
        telemetry.addData("Heading (deg)", "%.1f", heading());
        telemetry.update();
    }

    @Override
    public void start() {
        pinpoint.update();
        zeroHeading = heading();
    }

    @Override
    public void loop() {
        pinpoint.update();
        double heading = heading();

        if (gamepad1.aWasPressed()) zeroHeading = heading;
        if (gamepad1.dpadLeftWasPressed()) offsetDegrees += offsetStep;
        if (gamepad1.dpadRightWasPressed()) offsetDegrees -= offsetStep;
        if (gamepad1.yWasPressed()) offsetDegrees = 0;
        if (gamepad1.bWasPressed()) centerCheck = !centerCheck;

        // Turret angle (robot frame) that points at the held direction: held direction minus how far the robot has turned
        double robotTurned = heading - zeroHeading;
        double turretTarget = offsetDegrees - robotTurned;
        if (centerCheck) turret.setRawPosition(0.5);
        else turret.aimAt(turretTarget);

        if (driveEnabled) drive();
        else stopDrive();

        double commanded = turret.getCommandedAngle();
        double error = ((turretTarget - commanded) % 360 + 540) % 360 - 180;

        telemetry.addData("Mode (b)", centerCheck ? "CENTER CHECK - servos at 0.5, shooter should face robot-forward" : "HOLD");
        telemetry.addData("Robot turned since zero (deg)", "%.1f", robotTurned);
        telemetry.addData("Held direction offset (deg)", "%.1f", offsetDegrees);
        telemetry.addData("Turret target (deg)", "%.1f", turretTarget);
        telemetry.addData("Turret commanded (deg)", "%.1f", commanded);
        telemetry.addData("Turret servo position", "%.4f", turret.getCommandedPosition());
        if (!centerCheck && Math.abs(error) > 0.5) telemetry.addLine("Target is past the travel limits, holding the nearest limit");
        telemetry.addLine();
        telemetry.addData("degreesPerPosition", TurretPollen.degreesPerPosition);
        telemetry.addData("centerPosition", TurretPollen.centerPosition);
        telemetry.update();
    }

    private double heading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / denominator);
        leftRear.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightRear.setPower((y + x - rx) / denominator);
    }

    private void stopDrive() {
        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) m.setPower(0);
    }
}
