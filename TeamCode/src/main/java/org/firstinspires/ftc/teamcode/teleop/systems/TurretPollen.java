package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class TurretPollen {
    public static boolean goalIsRight = true;

    public static Servo turretPollenLeft, turretPollenRight;

    public static double turretTargetPosition;   // desired angle, degrees
    public static double turretCurrentPosition;  // last commanded angle, degrees (no encoder feedback)

    public static double turretHardwareLimit = 160; // max travel from center, degrees

    public static double servoRangeDegrees = 320.0;

    public Turret(HardwareMap hardwareMap) {
        turretPollenLeft = hardwareMap.get(Servo.class, "turretPollenLeft");
        turretPollenRight = hardwareMap.get(Servo.class, "turretPollenRight");
        turretPollenLeft.setDirection(Servo.Direction.REVERSE);
        // turretPollenRight.setDirection(Servo.Direction.REVERSE);
    }

    public void update() {
        turretTargetPosition = turretTargetToGoal();
        setTurretAngle(turretTargetPosition);
    }

    /*
     * Commands the turret to a target angle (degrees, 0 = centered),
     * clamped to the hardware limit and converted to a servo position.
     */
    public void setTurretAngle(double targetDegrees) {
        double clamped = clampToLimit(angleWrap(targetDegrees));
        turretCurrentPosition = clamped;
        turretPollenLeft.setPosition(degreesToServoPosition(clamped));
        turretPollenRight.setPosition(degreesToServoPosition(clamped));
    }

    private double clampToLimit(double degrees) {
        if (degrees > turretHardwareLimit) return turretHardwareLimit;
        if (degrees < -turretHardwareLimit) return -turretHardwareLimit;
        return degrees;
    }

    /*
     * Maps an angle in [-servoRangeDegrees/2, servoRangeDegrees/2] to a
     * servo position in [0.0, 1.0], with 0 degrees at center (0.5).
     */
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

        // Determine target goal coordinates
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