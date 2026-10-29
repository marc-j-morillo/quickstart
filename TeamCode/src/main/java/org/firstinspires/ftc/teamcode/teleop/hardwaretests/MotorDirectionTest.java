package org.firstinspires.ftc.teamcode.teleop.hardwaretests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Hold a button to spin that one motor forward.
 *   a = frontLeft    b = frontRight
 *   x = backLeft     y = backRight
 *
 * Directions are set so positive power = that wheel pushes the robot FORWARD.
 * Hold all four buttons at once and the robot should drive straight forward.
 * If a wheel pushes backward, flip its direction below.
 */
@TeleOp(name = "MotorDirectionTest", group = "TeleOp")
public class MotorDirectionTest extends OpMode {
    private static final double POWER = 0.5;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Same directions as your working drive code
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        frontLeft.setPower(gamepad1.a ? POWER : 0);
        frontRight.setPower(gamepad1.b ? POWER : 0);
        backLeft.setPower(gamepad1.x ? POWER : 0);
        backRight.setPower(gamepad1.y ? POWER : 0);

        telemetry.addLine("a = frontLeft, b = frontRight, x = backLeft, y = backRight");
        telemetry.addData("frontLeft", gamepad1.a);
        telemetry.addData("frontRight", gamepad1.b);
        telemetry.addData("backLeft", gamepad1.x);
        telemetry.addData("backRight", gamepad1.y);
        telemetry.update();
    }
}