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
    public static MotorGroup shooter;

    public Flywheel(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        //shooter2 = new DcMotorEx(hardwareMap, "shooter2");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shooter2.setInverted(true);
        //shooter = new MotorGroup(shooter1, shooter2);
    }

    public void update(double rpm) {
        //shooter.set(power);
        shooter1.setVelocity(rpm);
    }

    public int getVelocity() {
        return (int)(shooter1.getVelocity());
    }
}