package org.firstinspires.ftc.teamcode.teleop;



import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Flywheel;

@TeleOp(name = "FlywheelTest", group = "TeleOp")
@Configurable
public class FlywheelTest extends OpMode {

    //  ------------- Shooter -------------
    Flywheel flywheel;

    public static double flywheelTarget = 1000;
    public double flywheelActual = 0;

    public boolean isAtTarget = false;

    public double power = 0;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        flywheelActual = flywheel.getVelocity();

        isAtTarget = flywheelActual >= flywheelTarget;

        if (!isAtTarget) {
            power = 1;
        } else {
            power = 0;
        }

        flywheel.update(power);

        telemetry.addData("Velocity: ", flywheelActual);
        telemetry.addLine();
        telemetry.addData("Target velocity: ", flywheelTarget);
        telemetry.addLine();
        telemetry.addData("Is at target: ", isAtTarget);
        telemetry.addData("Power: ", power);


        telemetry.update();
    }
}
