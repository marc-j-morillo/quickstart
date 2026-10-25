// Maybe add intake to this whenever we actually get an intake?

package org.firstinspires.ftc.teamcode.teleop;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Flywheel;

@TeleOp(name = "IntakeTest", group = "TeleOp")
@Configurable
public class IntakeTest extends OpMode {
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
    public static double dormant = 0;
    public static double intaking = 0.5; // TEMPORARY VALUE
    public static double outtaking = intaking * -1;
    public static double semiTransferring = intaking * 0.2; // 0.1 power

    private double transferPower = dormant;



    // ------------- FLYWHEEL -------------
    Flywheel flywheel;

    public static double flywheelTarget = 1000;
    public double flywheelActual = 0;

    public boolean isAtTarget = false;

    public double power = 0;


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


        flywheel = new Flywheel(hardwareMap);
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
        } else if (gamepad1.left_trigger > 0.1) {
            transferPower = gamepad1.left_trigger * -1;
            intakePower = gamepad1.left_trigger * -1;
        } else if (gamepad1.right_bumper) {
            transferPower = intaking;
            intakePower = semiTransferring;
        } else if (gamepad1.left_bumper) {
            transferPower = outtaking;
            intakePower = outtaking;
        } else {
            intakePower = 0;
            transferPower = 0;
        }


        flywheelActual = flywheel.getVelocity();

        isAtTarget = flywheelActual >= flywheelTarget;

        if (!isAtTarget) {
            power = 1;
        } else {
            power = 0;
        }

        flywheel.update(power);



        transferMotor.setPower(transferPower);
        intakeMotor.setPower(intakePower);
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("a - switch blocker position");
        telemetry.addLine("right trigger - intake, proportional to amount");
        telemetry.addLine("left trigger - outtake, proportional to amount");
        telemetry.addLine("right button - Only intake running in");
        telemetry.addLine("left button - outtake at 100%");
        telemetry.addLine();
        telemetry.addLine();

        telemetry.addLine("VALUES:");
        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Transfer power: ", transferPower);

        telemetry.update();
    }
}
