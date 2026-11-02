package org.firstinspires.ftc.teamcode.teleop.hardwaretests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.systems.Intake;

/**
 * Flower wedge + intake test.
 *
 * Controls (gamepad1):
 *   b ........................... toggle the flower wedge up / down (both servos together)
 *   a ........................... toggle the blocker
 *   right trigger ............... intake, proportional to how far it's pressed
 *   left trigger ................ outtake, proportional to how far it's pressed
 *   right bumper ................ intake + transfer at 0.9 (stays on)
 *   left bumper ................. stop intake + transfer
 *
 * Each servo has its own up/down positions: Intake.leftWedgeUp / leftWedgeDown / rightWedgeUp / rightWedgeDown.
 * Edit them live in Panels (under Intake) while this runs; every opmode uses the same values, so there's
 * nothing to copy over. Put the final numbers in Intake.java so they survive a restart.
 */
@Configurable
@TeleOp(name = "FlowerWedgeTest", group = "TeleOp")
public class FlowerWedgeTest extends OpMode {
    //  ------------- FLOWER WEDGE -------------

    public Servo flowerWedgeLeft, flowerWedgeRight;

    public boolean flowerWedgeIsDown = false; // positions are in Intake

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


    //MOTOR POWERS
    public static final double dormant = 0;
    public static final double latchedPower = 0.9;

    private double transferPower = dormant;


    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        blocker.setDirection(Servo.Direction.FORWARD);


        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(DcMotorEx.Direction.FORWARD);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flowerWedgeLeft = hardwareMap.get(Servo.class, "flowerWedgeLeft");
        //flowerWedgeLeft.setDirection(Servo.Direction.REVERSE);
        flowerWedgeRight = hardwareMap.get(Servo.class, "flowerWedgeRight");
        flowerWedgeRight.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        // ----- Blocker -----
        if (gamepad1.aWasPressed()) {
            if (blockerPosition == blockerOpen) {
                blockerPosition = blockerClosed;
            } else {
                blockerPosition = blockerOpen;
            }
        }
        blocker.setPosition(blockerPosition);

        // ----- Intake -----
        // Each "WasPressed" only reports a press once, so read each button exactly once per loop
        boolean rightBumperPressed = gamepad1.rightBumperWasPressed();
        boolean leftBumperPressed = gamepad1.leftBumperWasPressed();

        if (gamepad1.right_trigger > 0.1) {
            transferPower = gamepad1.right_trigger;
            intakePower = gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0.1) {
            transferPower = gamepad1.left_trigger * -1;
            intakePower = gamepad1.left_trigger * -1;
        } else if (rightBumperPressed) {
            transferPower = latchedPower;
            intakePower = latchedPower;
        } else if (leftBumperPressed) {
            transferPower = 0;
            intakePower = 0;
        }

        transferMotor.setPower(transferPower);
        intakeMotor.setPower(intakePower);

        // ----- Flower wedge -----
        if (gamepad1.bWasPressed()) flowerWedgeIsDown = !flowerWedgeIsDown;
        // Re-sent every loop, so Panels edits to Intake's up/down values apply right away
        Intake.setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, flowerWedgeIsDown);

        // ----- Telemetry -----
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("b - toggle flower wedge up / down");
        telemetry.addLine("a - switch blocker position");
        telemetry.addLine("right trigger - intake, proportional to amount");
        telemetry.addLine("left trigger - outtake, proportional to amount");
        telemetry.addLine("right bumper - intake + transfer at 0.9 (stays on)");
        telemetry.addLine("left bumper - stop intake + transfer");
        telemetry.addLine();

        telemetry.addLine("VALUES:");
        telemetry.addData("Flower wedge", flowerWedgeIsDown ? "DOWN" : "UP");
        telemetry.addData("Left / right position", "%.3f / %.3f",
                flowerWedgeLeft.getPosition(), flowerWedgeRight.getPosition());
        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Intake power: ", intakePower);
        telemetry.addData("Transfer power: ", transferPower);

        telemetry.update();
    }
}
