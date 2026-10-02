// UNCOMMENT WHEN WE HAVE A NECTAR TURRET
/*
package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class TurretNectar {
    public static boolean goalIsRight = true;

    public static Servo turretNectar;

    public static double turretTargetPosition;   // desired angle, degrees
    public static double turretCurrentPosition;  // last commanded angle, degrees (no encoder feedback)

    public static double turretHardwareLimit = 160; // max travel from center, degrees

    // Axon servo's total mechanical rotation range in degrees.
    // Confirm this against how your servo is programmed (e.g. Axon Max ~355°).
    public static double servoRangeDegrees = 355.0;

    public TurretNectar(HardwareMap hardwareMap) {
        turretNectar = hardwareMap.get(Servo.class, "turretNectar");
        // turretNectar.setDirection(Servo.Direction.REVERSE);
    }

    public void update() {
        turretTargetPosition = turretTargetToGoal();
        setTurretAngle(turretTargetPosition);
    }

    /*
     * Commands the turret to a target angle (degrees, 0 = centered),
     * clamped to the hardware limit and converted to a servo position.
     *//*

    public void setTurretAngle(double targetDegrees) {
        double clamped = clampToLimit(angleWrap(targetDegrees));
        turretCurrentPosition = clamped;
        turretNectar.setPosition(degreesToServoPosition(clamped));
    }

    private double clampToLimit(double degrees) {
        if (degrees > turretHardwareLimit) return turretHardwareLimit;
        if (degrees < -turretHardwareLimit) return -turretHardwareLimit;
        return degrees;
    }

    /*
     * Maps an angle in [-servoRangeDegrees/2, servoRangeDegrees/2] to a
     * servo position in [0.0, 1.0], with 0 degrees at center (0.5).
     *//*
    private double degreesToServoPosition(double degrees) {
        double position = 0.5 + (degrees / servoRangeDegrees);
        return Math.max(0.0, Math.min(1.0, position));
    }

    public static double angleWrap(double degrees) {
        while (degrees > 180)  degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    public double turretTargetToGoal() {
        double targetGoalX;
        double targetGoalY;

        if (isRed) targetGoalX = 58.0;
        else targetGoalX = 84.0;

        if (Localization.botY < 61.0) targetGoalY = 61.0;
        else if (Localization.botY > 83.0) targetGoalY = 83.0; 
        else targetGoalY = Localization.botY;

        double deltaX = targetGoalX - Localization.botX;
        double deltaY = targetGoalY - Localization.botY;

        double goalAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double turretTarget = goalAngle - Localization.botHeading;

        return angleWrap(turretTarget);
    }
}
*/