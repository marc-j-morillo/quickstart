package org.firstinspires.ftc.teamcode.teleop;



import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Flywheel;

@TeleOp(name = "FlywheelTest", group = "TeleOp")
@Configurable
public class FlywheelTest extends OpMode {

    //  ------------- Shooter -------------
    DcMotorEx shooter1;

    public static double flywheelTarget = 1000;


    public boolean isAtTarget = false;

    public double power = 0;

    public static double F = 0;
    public static double P = 0;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooter1.setVelocity(flywheelTarget);

        double flywheelActual = shooter1.getVelocity();
        double error = flywheelTarget - flywheelActual;

        isAtTarget = flywheelActual >= flywheelTarget;

        telemetry.addData("Velocity: ", flywheelActual);
        telemetry.addLine();
        telemetry.addData("Target velocity: ", flywheelTarget);
        telemetry.addLine();
        telemetry.addData("Is at target: ", isAtTarget);
        telemetry.addData("Power: ", power);


        telemetry.update();
    }
}
