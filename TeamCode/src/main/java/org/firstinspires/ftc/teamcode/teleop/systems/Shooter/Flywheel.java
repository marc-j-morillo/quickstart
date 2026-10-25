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
    public static MotorEx shooter1, shooter2;
    public static MotorGroup shooter;

    public Flywheel(HardwareMap hardwareMap) {
        shooter1 = new MotorEx(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);

        shooter1.setInverted(true);
        shooter2.setInverted(true);
        shooter = new MotorGroup(shooter1, shooter2);
    }

    public void update(double power) {
        shooter.set(power);
    }

    public int getVelocity() {
        return (int)(shooter1.getVelocity());
    }
}