package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class Drive {
    public static DcMotorEx frontRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx backLeft;

    public static double rx; // Rotation value for the robot
    public static double driveP = 0, driveI = 0, driveD = 0, driveF = 0; // Change these values

    public static boolean hasPinpoint = false; // Change this to true when we get a pinpoint

//    public static double secondaryDriveP = .01, secondaryDriveI, secondaryDriveD = .001, secondaryDriveF;

    private PIDFController turnController = new PIDFController(driveP, driveI, driveD, driveF);
//    private PIDFController secondaryTurnController = new PIDFController(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF);
//    public double IMUOffset;

    public boolean RobotCentric;

    public Toggle pinpointToGoal;

    public Drive(HardwareMap hardwareMap) {
        //pinpointToGoal = new Toggle(true);  turn this on when we get a pinpoint
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.RobotCentric = robotCentric;

        rx = 0;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {
        pinpointToGoal.update(gamepad1.options);
        if (!RobotCentric && hasPinpoint) {
            //Field Centric Drive:
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-Localization.botHeading) - y * Math.sin(-Localization.botHeading);
            double rotY = x * Math.sin(-Localization.botHeading) + y * Math.cos(-Localization.botHeading);
            rotX = rotX * 1.2;  // Counteract imperfect strafing


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        } else {
            turnController.setP(driveP);
            turnController.setI(driveI);
            turnController.setD(driveD);
            turnController.setF(driveF);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing

            rx = (gamepad1.right_stick_x);

            if (gamepad1.right_bumper && pinpointToGoal.value() && hasPinpoint) {
                if (isRed) {
                    rx = calcRotBasedOnIdeal(Math.toDegrees(Localization.botHeading), Math.toDegrees(Math.atan2(144 - Localization.botY, 144 - Localization.botX)));
                } else {
                    rx = calcRotBasedOnIdeal(Math.toDegrees(Localization.botHeading), Math.toDegrees(Math.atan2(144 - Localization.botY, 0 - Localization.botX)));
                }
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
    private double calcRotBasedOnIdeal(double heading, double idealHeading) {
        // Error in rotations (should always be between (-0.5,0.5))
        double err = angleWrap(idealHeading - heading);
        return turnController.calculate(err, 0);
    }
    public double angleWrap(double angle) {
        // Wraps an angle to the range (-180, 180]
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
