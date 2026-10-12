
package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

public class Shooter {
    public static final double dormant = 0;

    // HOOD CLASS AND VARIABLES
    HoodPollen hoodPollen;

    //SERVO STATES
    public static double hoodPollenPosition, hoodNectarPosition;

    // TURRET CLASS AND VARIABLES
    TurretPollen turretPollen;
    //TurretNectar turretNectar;

    public static double turretTargetPositionPollen;   // desired position for the servo

    public static double turretTargetPositionNectar;   // desired angle

    public static final double turretHardwareLimit = 160;    // max travel from center, degrees

    public static final double servoRangeDegrees = 320.0;

    // Change these offsets to the actual turret position on the robot, for now they are set to 0
    // Offsets from odometry center to turret center, in inches
    public static final double turretPollenXOffset = 0.0;
    public static final double turretPollenYOffset = 0.0;
    public static final double turretNectarXOffset = 0.0;
    public static final double turretNectarYOffset = 0.0;



    // FLYWHEEL CLASS AND VARIABLES
    Flywheel flywheel;

    public static final double goalHeightInches = 65.6;

    //Shooter PID
    private PIDFController shooterController;

    public double shooterp = 0.007, shooteri = 0, shooterd = 0, kV = .000400186335403727, kS = .001;
    public double shooterpid;

    //MOTOR VELOCITY
    public static double shooterVelocity;

    //MOTOR TARGETS
    public static double shooterVelocityTarget;

    public static double fireTolerance = 40;
    public static final double notShootingTolerance = 50;
    public static final double poweringTolerance = 40;
    public static final double shootingTolerance = 150;


    public static final double emergencyEject = 0;

    public InterpLUT velocityLut = new InterpLUT();
    public InterpLUT hoodAngleLut = new InterpLUT();
    public static double distanceFromLimelightToTagInches;
    public static double turretCurrentPosition;

    public Shooter(HardwareMap hardwareMap) {
        //SHOOTER LUT
        //DISTANCE, VELOCITY
        velocityLut.add(0.0, 0.0);
        velocityLut.createLUT();

        //HOOD ANGLE LUT
        //DISTANCE, HOOD ANGLE
        hoodAngleLut.add(0.0, 0.0);
        hoodAngleLut.createLUT();

        turretPollen = new TurretPollen(hardwareMap);
        //turretNectar = new TurretNectar(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hoodPollen = new HoodPollen(hardwareMap);
        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);
        
        shooterVelocity = dormant;
        shooterVelocityTarget = dormant;
        hoodPollenPosition = 0;
    }

    public void update(double voltageMultiplier, boolean isDriving) {
        turretTargetPositionPollen = turretTargetToGoal(true);
        turretPollen.update(turretTargetPositionPollen);
        //turretTargetPositionNectar = turretTargetToGoal(false);
        //turretNectar.update(turretTargetPosition);

        hoodPollenPosition = calcHoodAngle(true);
        hoodPollen.update(hoodPollenPosition);

        if (shooterVelocityTarget == 0 || isDriving) {
            shooterp = 0;
            kV = 0;
        } else {
            shooterp = .025; // Change these values
            kV = .000400186335403727; // Change these values
        }
        
        shooterVelocity = Flywheel.leftShooter.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + (Math.signum(shooterVelocityTarget) * kS * voltageMultiplier);
        flywheel.update(Range.clip(shooterpid, -1.0, 1.0));

        switch (Robot.transportState) {
            case HOME:
                shooterVelocityTarget = calcVelocity(isDriving, false);
                fireTolerance = notShootingTolerance;
                break;
            case INTAKE:
                shooterVelocityTarget = calcVelocity(isDriving, false);
                fireTolerance = notShootingTolerance;
                break;
            case OUTTAKE:
                shooterVelocityTarget = calcVelocity(isDriving, false);
                fireTolerance = notShootingTolerance;
                break;
            case POWER_SHOOTER:
                shooterVelocityTarget = calcVelocity(isDriving, true);
                fireTolerance = poweringTolerance;
                break;
            case SHOOT:
                shooterVelocityTarget = calcVelocity(isDriving, true);
                fireTolerance = shootingTolerance;
                break;
        }
    }

    
    // -------------------------------- TURRET METHODS ---------------------------------
    /*
     * Commands the turret to a target angle (degrees, 0 = centered),
     * clamped to the hardware limit and converted to a servo position.
     */
    private void setTurretAngle(double targetDegrees) {
        double clamped = clampToLimit(angleWrap(targetDegrees));
        turretCurrentPosition = clamped;
        turretPollen.update(degreesToServoPosition(clamped));
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

    private static double angleWrap(double degrees) {
        while (degrees > 180)  degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    private double turretTargetToGoal(boolean isPollen) {
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

    private double calcHoodAngle(boolean isPollen) {
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
        return hoodAngleLut.get(distanceFromLimelightToTagInches);
    }


    // -------------------------------- FLYWHEEL METHODS ---------------------------------

    
    public boolean inRange() {
        if (Math.abs(shooterVelocity - shooterVelocityTarget) < fireTolerance) {
            return true;
        } else {
            return false;
        }
    }

    private double calcVelocity(boolean isDriving, boolean isShooting) {
        double targetGoalX;
        double targetGoalY;

        double velocityMultiplier = 1;

        // Determine target goal coordinates
        if (isRed) {
            targetGoalX = 58.0;
        } else {
            targetGoalX = 84.0;
        }

        if (Localization.botY < 51.0) {
            targetGoalY = 61.0;
        } else if (Localization.botY > 93.0) {
            targetGoalY = 83.0;
        } else {
            targetGoalY = Localization.botY;
        }

        // Calculate Distance Vector
        double deltaX = targetGoalX - Localization.botX;
        double deltaY = targetGoalY - Localization.botY;
        distanceFromLimelightToTagInches = Math.hypot(deltaX, deltaY);
        

        // Lookup Velocity & Round to Nearest 10 RPM
        double rawVelocity = velocityLut.get(distanceFromLimelightToTagInches);
        double velocityTarget = Math.round(rawVelocity / 10.0) * 10.0;
        
        // Change velocity depending if on we can shoot or if we are driving.
        if (deltaY < 1) { // Will hit the walls of the goal, so can't shoot
            velocityMultiplier = 0.5;
        } else if (isDriving) {
            velocityMultiplier = 0.8;
        } else {
            velocityMultiplier = 1;
        }
        if (isShooting) {
            velocityMultiplier = 1;
        }
        if (inAuto) {
            velocityMultiplier = 1;
        }
        velocityTarget *= velocityMultiplier;

        return velocityTarget;
    }
}