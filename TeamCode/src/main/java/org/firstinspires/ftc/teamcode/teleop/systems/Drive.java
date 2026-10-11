package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drive {
    public static DcMotorEx frontRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx backLeft;
    
    public static double rx = 0; // Rotation value for the robot
    public static double frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    public static double driveP = 0, driveI = 0, driveD = 0, driveF = 0; // Change these values

    public static boolean hasPinpoint = true;

//    public static double secondaryDriveP = .01, secondaryDriveI, secondaryDriveD = .001, secondaryDriveF;

    private PIDFController turnController = new PIDFController(driveP, driveI, driveD, driveF);
//    private PIDFController secondaryTurnController = new PIDFController(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF);
//    public double IMUOffset;

    public boolean FieldCentric = false;

    public static boolean isDriving = false;




    public Drive(HardwareMap hardwareMap, boolean fieldCentric) {
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

        this.FieldCentric = fieldCentric;
    }

    public void update(Gamepad gamepad1) {
        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
            isDriving = true;
        } else {
            isDriving = false;
        }

        if (FieldCentric && hasPinpoint) {
            fieldCentricDrive();
        } else {
            robotCentricDrive();
        }
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }



    private void fieldCentricDrive() {
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

        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

    private void robotCentricDrive() {
        turnController.setP(driveP);
        turnController.setI(driveI);
        turnController.setD(driveD);
        turnController.setF(driveF);
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing
        rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;
    }
}
