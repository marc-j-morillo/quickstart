/*

"Old" turret code, kept for reference. The new turret code is in TurretNectar.java and TurretPollen.java.

package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;
public class Turret {
    public static boolean goalIsRight = true;

    public static Servo turretPollen;
    //public static Servo turretNectar;

    public static double turretPollenPosition;
    //public static double turretNectarPosition;

    public static double turretPollenTargetPosition;
    //public static double turretNectarTargetPosition;

    public static PIDFController turretPollenController;
    //public static PIDFController turretNectarController;

    public static double turretPollenP, turretPollenI, turretPollenD, turretPollenF;

    public static double turretPollenPositionError;

    public static double turretHardwareLimit = 160;

    public final double turretMotorTicksPerRev = 112;

    public Turret(HardwareMap hardwareMap) {
        turretPollen = hardwareMap.get(Servo.class, "turretPollen");
        //turretPollen.setDirection(Servo.Direction.REVERSE);

        turretNectar = hardwareMap.get(Servo.class, "turretNectar");
        //turretNectar.setDirection(Servo.Direction.REVERSE);

        turretPollenController = new PIDFController(turretPollenP, turretPollenI, turretPollenD, turretPollenF);
        //turretNectarController = new PIDFController(turretNectarP, turretNectarI, turretNectarD, turretNectarF);
    }

    public void update() {
//        turretTargetPosition = turretTargetToGoal();
        turretPosition = turretPositionInDegrees();
        turretPollenPositionError = angleWrap(turretPollenTargetPosition - turretPollenPosition);
        turretPollen.setPosition(turretPollenController.calculate(turretPositionError, 0));
    }

    public double turretPositionInDegrees() {
        // Force floating-point division for gear ratio
        double gearRatio = 555.0 / 17.0;
        double totalTicksPerRev = turretMotorTicksPerRev * gearRatio;

        double turretDegrees = (turretPollen.getCurrentPosition() / totalTicksPerRev) * 360.0;
        // Normalize degrees within [-180, 180] range
        while (turretDegrees > 180)  turretDegrees -= 360;
        while (turretDegrees < -180) turretDegrees += 360;

        return turretDegrees;
    }
    public static double angleWrap(double err) {
        // Normalize err within [-180, 180] range
        while (err > 180)  err -= 360;
        while (err < -180) err += 360;

        // Enforce software safety limits to block positive rotation
        if (turretPollenPosition >= turretHardwareLimit && err > 0)  err = 0;
        if (turretPollenPosition <= -turretHardwareLimit && err < 0) err = 0;

        return err;
    }

    public double turretTargetToGoal() {
        double targetGoalX;
        double targetGoalY;

        // Determine target goal coordinates
        if (isRed) {
            targetGoalX = 58.0;
            targetGoalY = goalIsRight ? 61.0 : 83.0; // Different Y values for each lean
        } else {
            targetGoalX = 84.0;
            targetGoalY = !goalIsRight ? 61.0 : 83.0; // Different Y values for each lean
        }

        // Distance calculation from robot to goal
        double deltaX = targetGoalX - Localization.botX;
        double deltaY = targetGoalY - Localization.botY;

        // Calculate angle from robot to goal (-180 to 180)
        double goalAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Factor in bot heading to previous angle for final turret target angle
        double turretTarget = goalAngle - LocalizationV2.botHeading;

        // Normalize result within [-180, 180] range
        while (turretTarget > 180)  turretTarget -= 360;
        while (turretTarget < -180) turretTarget += 360;

        return turretTarget;
    }
}
*/