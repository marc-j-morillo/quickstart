package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeTest", group = "TeleOp")
public class IntakeTest extends OpMode {
    //  ------------- BLOCKER -------------

    public Servo blocker;

    //SERVO STATES
    public static final double blockerOpen = 0.6;
    public static final double blockerClosed = 0.85;
    public double blockerPosition = blockerOpen;

    //  ------------- INTAKE -------------

    public DcMotorEx transferMotor;


    //MOTOR POWERS
    public static final double dormant = 0;
    public static final double intaking = 0.5; // TEMPORARY VALUE
    public static final double outtaking = intaking * -1;
    public static final double semiTransferring = intaking * 0.2; // 0.1 power

    private double transferPower = dormant;


    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setDirection(Servo.Direction.FORWARD);


        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        //transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            blockerPosition = blockerClosed;
        } else {
            blockerPosition = blockerOpen;
        }
        blocker.setPosition(blockerPosition);


        if (gamepad1.right_trigger > 0.1) {
            transferPower = gamepad1.right_trigger;
        } else {
            transferPower = gamepad1.left_trigger * -1;
        }
        transferMotor.setPower(transferPower);

        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Transfer power: ", transferPower);

        telemetry.update();
    }
}
