
package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botX;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botY;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botHeading;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.teleop.systems.Robot;


import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

public class Shooter {
    public static final double dormant = 0;

    // ---------- HOOD CLASS AND VARIABLES ----------
    HoodPollen hoodPollen;

    //SERVO STATES
    public static double hoodPollenPosition, hoodNectarPosition;

    // ---------- TURRET CLASS AND VARIABLES ----------
    TurretPollen turretPollen;
    //TurretNectar turretNectar;

    public static double turretTargetPositionPollen;   // desired position for the servo

    public static double turretTargetPositionNectar;   // desired angle

    public static final double turretHardwareLimit = 120;    // max travel from center, degrees

    public static final double servoRangeDegrees = 240.0;    // turretHardWareLimit * 2, total range

    // Change these offsets to the actual turret position on the robot, for now they are set to 0
    // Offsets from odometry center to turret center, in inches
    public static final double turretPollenXOffset = 0.0;
    public static final double turretPollenYOffset = 0.0;
    public static final double turretNectarXOffset = 0.0;
    public static final double turretNectarYOffset = 0.0;



    // ---------- FLYWHEEL CLASS AND VARIABLES ----------
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
        velocityLut.add(1, 100);
        velocityLut.createLUT();

        //HOOD ANGLE LUT
        //DISTANCE, HOOD ANGLE
        hoodAngleLut.add(0.0, 0.0);
        velocityLut.add(10, 0.1);
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
        
        shooterVelocity = flywheel.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + (Math.signum(shooterVelocityTarget) * kS * voltageMultiplier);
        flywheel.update(shooterVelocityTarget);

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
        double targetGoalX;
        double targetGoalY;

        // Determine target goal coordinates
        if (isRed) targetGoalX = 58.0;
        else targetGoalX = 84.0;

        if (botY < 61.0) targetGoalY = 61.0;
        else if (botY > 83.0) targetGoalY = 83.0; 
        else targetGoalY = botY;

        double deltaX = targetGoalX - botX;
        double deltaY = targetGoalY - botY;

        double goalAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double turretTarget = goalAngle - Math.toDegrees(botHeading);

        turretTarget = angleWrap(turretTarget);
        turretTarget = Range.clip(turretTarget, -turretHardwareLimit, turretHardwareLimit);
        turretTarget = degreesToServoPosition(turretTarget);
        return Range.clip(turretTarget, 0.01, 0.99);
    }

    // -------------------------------- HOOD METHODS ---------------------------------

    private double calcHoodAngle(boolean isPollen) {
        double distance = this.getDistance();

        // Lookup Hood Angle and return
        return hoodAngleLut.get(distance);
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
        double distance = this.getDistance();

        // Lookup Velocity & Round to Nearest 10 RPM
        double rawVelocity = velocityLut.get(distance);
        double velocityTarget = Math.round(rawVelocity / 10.0) * 10.0;
        
        // Change velocity depending if on we can shoot or if we are driving.
        double velocityMultiplier;
        if (botY <= 83 && botY >= 61) { // Will hit the walls of the goal, so can't shoot
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

    private double getDistance() {
        double targetGoalX;
        double targetGoalY;

        // Determine target goal coordinates
        if (isRed) {
            targetGoalX = 58.0;
        } else {
            targetGoalX = 84.0;
        }

        if (botY < 61.0) {
            targetGoalY = 61.0;
        } else if (botY > 83.0) {
            targetGoalY = 83.0;
        } else {
            targetGoalY = botY;
        }

        // Calculate Distance Vector
        double deltaX = Math.abs(targetGoalX - botX);
        double deltaY = Math.abs(targetGoalY - botY);
        return Math.hypot(deltaX, deltaY);
    }
}