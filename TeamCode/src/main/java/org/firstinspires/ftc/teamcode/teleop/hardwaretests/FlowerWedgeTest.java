package org.firstinspires.ftc.teamcode.teleop.hardwaretests;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FlowerWedgeTest", group = "TeleOp")
public class FlowerWedgeTest extends OpMode {
    //  ------------- FLOWER WEDGE -------------

    public Servo flowerWedgeLeft, flowerWedgeRight;


    public static final double flowerWedgeDown = 0; // TEMPORARY VALUE
    public static final double flowerWedgeUp = .5; // TEMPORARY VALUE
    public boolean flowerWedgeIsDown;
    public ElapsedTime flowerWedgeWait;

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
    public static final double intaking = 0.5; // TEMPORARY VALUE
    public static final double outtaking = intaking * -1;
    public static final double semiTransferring = intaking * 0.2; // 0.1 power

    private double transferPower = dormant;


    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        blocker.setDirection(Servo.Direction.FORWARD);


        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        //transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if (gamepad1.aWasPressed()) {
            if (blockerPosition == blockerOpen) {
                blockerPosition = blockerClosed;
            } else {
                blockerPosition = blockerOpen;
            }
        }
        blocker.setPosition(blockerPosition);


        if (gamepad1.right_trigger > 0.1) {
            transferPower = gamepad1.right_trigger;
            intakePower = gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > 0.1){
            transferPower = gamepad1.left_trigger * -1;
            intakePower = gamepad1.left_trigger * -1;
        } else if (gamepad1.rightBumperWasPressed()) {
            transferPower = 0.9;
            intakePower = 0.9;
        } else if (gamepad1.leftBumperWasPressed()) {
            transferPower = 0;
            intakePower = 0;
        }

        flowerWedgeLeft.setPosition(setFlowerWedge(gamepad1));
        flowerWedgeRight.setPosition(setFlowerWedge(gamepad1));

        transferMotor.setPower(transferPower);
        intakeMotor.setPower(intakePower);
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("a - switch blocker position");
        telemetry.addLine("right trigger - intake, proportional to amount");
        telemetry.addLine("left trigger - outtake, proportional to amount");
        telemetry.addLine();
        telemetry.addLine();

        telemetry.addLine("VALUES:");
        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Transfer power: ", transferPower);

        telemetry.update();
    }

    public double setFlowerWedge(Gamepad gamepad1) {
        // Raise and lower the flower wedge with the left bumper,
        // but only if 0.5 seconds have passed since the last press to prevent rapid toggling
        if (gamepad1.leftBumperWasPressed() && flowerWedgeWait.seconds() > 0.5) {
            flowerWedgeWait.reset();
            if (flowerWedgeIsDown) {
                flowerWedgeIsDown = false;
                return flowerWedgeUp;
            } else {
                flowerWedgeIsDown = true;
                return flowerWedgeDown;
            }
        }
        else {
            if (flowerWedgeIsDown) {
                return flowerWedgeDown;
            } else {
                return flowerWedgeUp;
            }
        }
    }
}
