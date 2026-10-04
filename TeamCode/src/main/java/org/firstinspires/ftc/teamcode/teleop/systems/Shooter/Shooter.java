// TODO: Add turretNectar class and variables, and update methods to include nectar turret functionality
// Change add an offset for the turret position based on the exact turret position on the robot,
// and set it to 0 for now.

package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

import teleop.systems.Drive.Localization;
public class Shooter {

    // HOOD CLASS AND VARIABLES
    HoodPollen hoodPollen;



    // TURRET CLASS AND VARIABLES
    TurretPollen turretPollen;
    //TurretNectar turretNectar;

    public static double turretTargetPositionPollen;   // desired position for the servo

    public static double turretTargetPositionNectar;   // desired angle

    public static double turretHardwareLimit = 160;    // max travel from center, degrees

    public static double servoRangeDegrees = 320.0;

    // Change these offsets to the actual turret position on the robot, for now they are set to 0
    // Offsets from odometry center to turret center, in inches
    public final double turretPollenXOffset = 0.0;
    public final double turretPollenYOffset = 0.0;
    public final double turretNectarXOffset = 0.0;
    public final double turretNectarYOffset = 0.0;



    // FLYWHEEL CLASS AND VARIABLES
    Flywheel flywheel;

    public double goalHeightInches = 65.6;

    //Shooter PID
    private PIDFController shooterController;

    public double shooterp = 0.007, shooteri = 0, shooterd = 0, kV = .000400186335403727, kS = .001;
    public double shooterpid;

    //MOTOR VELOCITY
    public static double shooterVelocity;

    //MOTOR TARGETS
    public static double shooterVelocityTarget;

    //SERVO STATES
    public static double hoodPollenPosition, hoodNectarPosition;

    

    public Shooter(HardwareMap hardwareMap) {
        //SHOOTER LUT
        //DISTANCE, VELOCITY
        velocityLut.add(0, 0);
        velocityLut.createLUT();

        //HOOD ANGLE LUT
        //DISTANCE, HOOD ANGLE
        hoodAngleLut.add(0, 0);
        hoodAngleLut.createLUT();

        turretPollen = new TurretPollen(hardwareMap);
        //turretNectar = new TurretNectar(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hoodPollen = new HoodPollen(hardwareMap);
        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);
        
        shooterVelocity = 0;
        shooterVelocityTarget = 0;
        hoodPollenPosition = 0;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {
        turretTargetPositionPollen = turretTargetToGoal(true);
        turretPollen.update(turretTargetPositionPollen);
        //turretTargetPositionNectar = turretTargetToGoal(false);
        //turretNectar.update(turretTargetPosition);

        hoodPollenPosition = calcHoodAngle();
        hoodPollen.update(hoodPollenPosition);

        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            kV = 0;
        } else {
            shooterp = .025; // Change these values
            kV = .000400186335403727; // Change these values
        }
        shooterVelocityTarget = calcVelocity();
        shooterVelocity = shooter.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + (Math.signum(shooterVelocityTarget) * kS * voltageMultiplier);
        flywheel.update(Range.clip(shooterpid, -1.0, 1.0));
    }
    
    // -------------------------------- TURRET METHODS ---------------------------------
    /*
     * Commands the turret to a target angle (degrees, 0 = centered),
     * clamped to the hardware limit and converted to a servo position.
     */
    private void setTurretAngle(double targetDegrees) {
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

    public double turretTargetToGoal(boolean isPollen) {
        double xOffset;
        double yOffset;

        // Calculate the offset of the turret from the robot's center 
        // based on whether it's the pollen or nectar turret
        if (isPollen) {
            xOffset = turretPollenXOffset * Math.sin(Localization.botHeading);
            yOffset = turretPollenYOffset * Math.cos(Localization.botHeading);
        } else {
            xOffset = turretNectarXOffset * Math.sin(Localization.botHeading);
            yOffset = turretNectarYOffset * Math.cos(Localization.botHeading);
        }

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

        turretTarget = angleWrap(turretTarget);
        turretTarget = clampToLimit(turretTarget);
        turretTarget = degreesToServoPosition(turretTarget);
        return turretTarget;
    }

    // -------------------------------- HOOD METHODS ---------------------------------

    public double calcHoodAngle(boolean isPollen) {
        double xOffset;
        double yOffset;

        // Calculate the offset of the turret from the robot's center 
        // based on whether it's the pollen or nectar turret
        if (isPollen) {
            xOffset = turretPollenXOffset * Math.sin(Localization.botHeading);
            yOffset = turretPollenYOffset * Math.cos(Localization.botHeading);
        } else {
            xOffset = turretNectarXOffset * Math.sin(Localization.botHeading);
            yOffset = turretNectarYOffset * Math.cos(Localization.botHeading);
        }

        double targetGoalX;
        double targetGoalY;

        // Determine target goal coordinates
        if (isRed) targetGoalX = 58.0;
        else targetGoalX = 84.0;

        if (Localization.botY < 51.0) targetGoalY = 61.0;
        else if (Localization.botY > 93.0) targetGoalY = 83.0;
        else targetGoalY = Localization.botY;

        // Calculate Distance Vector
        double deltaX = targetGoalX - Localization.botX;
        double deltaY = targetGoalY - Localization.botY;
        distanceFromLimelightToTagInches = Math.hypot(deltaX, deltaY);

        // Lookup Hood Angle and return
        return hoodAngleLut.get(distanceFromLimelightToTagInches);;
    }


    // -------------------------------- FLYWHEEL METHODS ---------------------------------

    public static double fireTolerance = 40;
    public boolean inRange(double currentVelocity, double targetVelocity) {
        if (Math.abs(targetVelocity - currentVelocity) < fireTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public double calcVelocity() {
        double targetGoalX;
        double targetGoalY;

        // Determine target goal coordinates
        if (isRed) targetGoalX = 58.0;
        else targetGoalX = 84.0;

        if (Localization.botY < 51.0) targetGoalY = 61.0;
        else if (Localization.botY > 93.0) targetGoalY = 83.0;
        else targetGoalY = Localization.botY;


        // Calculate Distance Vector
        double deltaX = targetGoalX - Localization.botX;
        double deltaY = targetGoalY - Localization.botY;
        distanceFromLimelightToTagInches = Math.hypot(deltaX, deltaY);

        // Lookup Velocity & Round to Nearest 10 RPM
        double rawVelocity = velocityLut.get(distanceFromLimelightToTagInches);
        double velocityTarget = Math.round(rawVelocity / 10.0) * 10.0;

        return velocityTarget;
    }

}
