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

    // Directions last applied, so a config switch (Shooter.activeConfig) re-applies them
    private boolean appliedShooter1Inverted, appliedShooter2Inverted;

    public Flywheel(HardwareMap hardwareMap) {
        shooter1 = new MotorEx(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);

        shooter = new MotorGroup(shooter1, shooter2);
        applyDirections();
    }

    /** Motor directions from the active config (Shooter.config()). */
    private void applyDirections() {
        appliedShooter1Inverted = Shooter.config().shooter1Inverted;
        appliedShooter2Inverted = Shooter.config().shooter2Inverted;
        shooter1.setInverted(appliedShooter1Inverted);
        shooter2.setInverted(appliedShooter2Inverted);
    }

    public void update(double power) {
        if (Shooter.config().shooter1Inverted != appliedShooter1Inverted
                || Shooter.config().shooter2Inverted != appliedShooter2Inverted) {
            applyDirections();
        }
        shooter.set(power);
    }

    public int getVelocity() {
        return (int)(shooter1.getVelocity());
    }
}