// Change the constuctor to

package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.util.InterpLUT;

public class Flywheel {
    public static DcMotorEx shooter1, shooter2;
    public static double P = 0, I = 0, D = 0, F = 0;   // copy your tuned values from FlywheelTest

    public Flywheel(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);   // match FlywheelTest
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setVelocityPIDFCoefficients(P, I, D, F);

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);   // match FlywheelTest
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void update(double ticksPerSecond) {
        shooter1.setVelocity(ticksPerSecond);
        // if shooter2 is geared to shooter1 with no encoder of its own:
        shooter2.setPower(shooter1.getPower());
    }

    public double getVelocity() { return shooter1.getVelocity(); }
}