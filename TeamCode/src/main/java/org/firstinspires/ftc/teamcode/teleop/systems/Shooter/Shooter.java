package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botX;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botY;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.teleop.systems.Robot;


import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;

@Configurable
public class Shooter {
    public static final double dormant = 0;

    // HOOD CLASS AND VARIABLES
    HoodPollen hoodPollen;

    //SERVO STATES
    @IgnoreConfigurable public static double hoodPollenPosition, hoodNectarPosition;

    // TURRET CLASS AND VARIABLES (angle <-> servo mapping and limits live in TurretPollen)
    TurretPollen turretPollen;
    //TurretNectar turretNectar;

    @IgnoreConfigurable public static double turretTargetAnglePollen;     // desired turret angle, degrees
    @IgnoreConfigurable public static double turretTargetPositionPollen;  // servo position actually commanded
    @IgnoreConfigurable public static double turretTargetPositionNectar;

    // Offsets from odometry center to turret center, in inches, in the robot frame:
    // X = forward, Y = left. Measure these on the robot, for now they are set to 0
    public static double turretPollenXOffset = 0.0;
    public static double turretPollenYOffset = 0.0;
    public static double turretNectarXOffset = 0.0;
    public static double turretNectarYOffset = 0.0;

    // -------------- Goal --------------
    // The aim point is at goalX for your alliance, with y following the shooter's y, clamped to goalMinY..goalMaxY
    public static double redGoalX = 58.0;
    public static double blueGoalX = 84.0;
    public static double goalMinY = 61.0;
    public static double goalMaxY = 83.0;

    // Optional aim-point override (used by the test opmodes). When enabled, turret aim, hood angle and
    // flywheel velocity all use this field point (inches, same frame as botX/botY) instead of the goal.
    public static boolean useTargetOverride = false;
    public static double targetOverrideX = 70.75;
    public static double targetOverrideY = 70.75;


    // FLYWHEEL CLASS AND VARIABLES
    Flywheel flywheel;

    public static final double goalHeightInches = 65.6;

    //Shooter PID (gains are zeroed while driving or when the target is 0)
    private PIDFController shooterController;

    public static double shooterP = .025, shooterI = 0, shooterD = 0, kV = .000400186335403727, kS = .001;
    public double shooterpid;

    //MOTOR VELOCITY
    @IgnoreConfigurable public static double shooterVelocity;

    //MOTOR TARGETS
    @IgnoreConfigurable public static double shooterVelocityTarget;

    @IgnoreConfigurable public static double fireTolerance = 40;
    public static double notShootingTolerance = 50;
    public static double poweringTolerance = 40;
    public static double shootingTolerance = 150;

    // Velocity target changes
    public static double velocityRounding = 10;            // round the LUT velocity to the nearest this many ticks/s
    public static double drivingVelocityMultiplier = 0.8;  // idle slower while driving
    public static double blockedVelocityMultiplier = 0.5;  // idle slower when the goal walls are in the way
    public static double blockedDeltaY = 1;                // goal y - shooter y below this counts as blocked

    // -------------- Shot table --------------
    // Editable live from Panels: Shooter rebuilds the LUTs on its next update whenever these change.
    // Each index is one shot: at lutDistances[i] inches, spin to lutVelocities[i] and set the hood to lutHoods[i].
    // A distance <= 0 marks an empty slot. Find the points with LutTuner.
    public static double[] lutDistances  = new double[12];
    public static double[] lutVelocities = new double[12];
    public static double[] lutHoods      = new double[12];

    // Added on top of the LUT output (LutTuner adjusts these, then bakes them into a new point)
    public static double velocityTrim = 0;
    public static double hoodTrim = 0;

    // SafeLut never throws and clamps the lookup distance to the table's range (see SafeLut at the bottom).
    public SafeLut velocityLut = new SafeLut();
    public SafeLut hoodAngleLut = new SafeLut();
    private double[] builtDistances, builtVelocities, builtHoods;

    @IgnoreConfigurable public static double distanceFromLimelightToTagInches;
    @IgnoreConfigurable public static double lutVelocity, lutHood; // raw LUT output at the current distance
    @IgnoreConfigurable public static double shooterX, shooterY;   // field position of the turret

    public Shooter(HardwareMap hardwareMap) {
        rebuildLutsIfChanged();

        turretPollen = new TurretPollen(hardwareMap);
        //turretNectar = new TurretNectar(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hoodPollen = new HoodPollen(hardwareMap);
        shooterController = new PIDFController(shooterP, shooterI, shooterD, kV);

        shooterVelocity = dormant;
        shooterVelocityTarget = dormant;
        hoodPollenPosition = 0;
    }

    public void update(double voltageMultiplier, boolean isDriving) {
        rebuildLutsIfChanged();
        updateShooterPosition(true);

        double deltaX = goalX() - shooterX;
        double deltaY = goalY() - shooterY;
        distanceFromLimelightToTagInches = Math.hypot(deltaX, deltaY);
        lutVelocity = velocityLut.get(distanceFromLimelightToTagInches);
        lutHood = hoodAngleLut.get(distanceFromLimelightToTagInches);

        turretTargetAnglePollen = turretTargetToGoal(deltaX, deltaY);
        turretPollen.setAngle(turretTargetAnglePollen);
        turretTargetPositionPollen = turretPollen.getCommandedPosition();
        //turretTargetPositionNectar = turretTargetToGoal(false);
        //turretNectar.update(turretTargetPosition);

        hoodPollenPosition = HoodPollen.clamp(lutHood + hoodTrim);
        hoodPollen.update(hoodPollenPosition);

        // Set this loop's target before running the controller, so it isn't a loop behind
        switch (Robot.transportState) {
            case HOME:
                shooterVelocityTarget = calcVelocity(deltaY, isDriving, false);
                fireTolerance = notShootingTolerance;
                break;
            case INTAKE:
                shooterVelocityTarget = calcVelocity(deltaY, isDriving, false);
                fireTolerance = notShootingTolerance;
                break;
            case OUTTAKE:
                shooterVelocityTarget = calcVelocity(deltaY, isDriving, false);
                fireTolerance = notShootingTolerance;
                break;
            case POWER_SHOOTER:
                shooterVelocityTarget = calcVelocity(deltaY, isDriving, true);
                fireTolerance = poweringTolerance;
                break;
            case SHOOT:
                shooterVelocityTarget = calcVelocity(deltaY, isDriving, true);
                fireTolerance = shootingTolerance;
                break;
        }

        boolean idle = shooterVelocityTarget == 0 || isDriving;
        double p = idle ? 0 : shooterP;
        double f = idle ? 0 : kV * voltageMultiplier;

        shooterVelocity = flywheel.getVelocity();
        shooterController.setPIDF(p, shooterI, shooterD, f);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + (Math.signum(shooterVelocityTarget) * kS * voltageMultiplier);
        flywheel.update(Range.clip(shooterpid, -1.0, 1.0));
    }


    // -------------------------------- AIM POINT ---------------------------------

    /** Field position of the turret: odometry center plus the turret offset rotated by the heading. */
    private void updateShooterPosition(boolean isPollen) {
        double xOffset = isPollen ? turretPollenXOffset : turretNectarXOffset;
        double yOffset = isPollen ? turretPollenYOffset : turretNectarYOffset;
        double heading = Math.toRadians(botHeading);

        shooterX = botX + xOffset * Math.cos(heading) - yOffset * Math.sin(heading);
        shooterY = botY + xOffset * Math.sin(heading) + yOffset * Math.cos(heading);
    }

    private double goalX() {
        if (useTargetOverride) return targetOverrideX;
        return isRed ? redGoalX : blueGoalX;
    }

    private double goalY() {
        if (useTargetOverride) return targetOverrideY;
        return Range.clip(shooterY, goalMinY, goalMaxY);
    }


    // -------------------------------- TURRET METHODS ---------------------------------

    private static double angleWrap(double degrees) {
        while (degrees > 180)  degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    /** Turret angle (degrees, 0 = robot-forward) that points at the goal. TurretPollen clamps it to its limits. */
    private double turretTargetToGoal(double deltaX, double deltaY) {
        double goalAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        return angleWrap(goalAngle - botHeading);
    }


    // -------------------------------- FLYWHEEL METHODS ---------------------------------


    public boolean inRange() {
        if (Math.abs(shooterVelocity - shooterVelocityTarget) < fireTolerance) {
            return true;
        } else {
            return false;
        }
    }

    private double calcVelocity(double deltaY, boolean isDriving, boolean isShooting) {
        // Lookup Velocity & Round to Nearest velocityRounding
        double rawVelocity = lutVelocity + velocityTrim;
        double velocityTarget = velocityRounding > 0
                ? Math.round(rawVelocity / velocityRounding) * velocityRounding
                : rawVelocity;

        // Change velocity depending if on we can shoot or if we are driving.
        double velocityMultiplier;
        if (deltaY < blockedDeltaY) { // Will hit the walls of the goal, so can't shoot
            velocityMultiplier = blockedVelocityMultiplier;
        } else if (isDriving) {
            velocityMultiplier = drivingVelocityMultiplier;
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

    // -------------------------------- SHOT TABLE ---------------------------------

    /** Rebuilds velocityLut / hoodAngleLut from the lut arrays if any of them changed since the last build. */
    public void rebuildLutsIfChanged() {
        if (Arrays.equals(lutDistances, builtDistances)
                && Arrays.equals(lutVelocities, builtVelocities)
                && Arrays.equals(lutHoods, builtHoods)) {
            return;
        }

        SafeLut velocities = new SafeLut();
        SafeLut hoods = new SafeLut();
        int points = Math.min(lutDistances.length, Math.min(lutVelocities.length, lutHoods.length));
        for (int i = 0; i < points; i++) {
            if (lutDistances[i] <= 0) continue; // empty slot
            velocities.add(lutDistances[i], lutVelocities[i]);
            hoods.add(lutDistances[i], lutHoods[i]);
        }
        velocities.createLUT();
        hoods.createLUT();
        velocityLut = velocities;
        hoodAngleLut = hoods;

        builtDistances = lutDistances.clone();
        builtVelocities = lutVelocities.clone();
        builtHoods = lutHoods.clone();
    }

    // -------------------------------- SAFE LUT ---------------------------------

    /**
     * Drop-in replacement for FTCLib's InterpLUT that can never throw and never extrapolate.
     *  - Points can be added in any order; adding an X that already exists overwrites it.
     *  - 0 points: get() returns 0.  1 point: get() returns that point's value.
     *  - Inputs outside the table (and NaN) are clamped to the nearest end of the table.
     * FTCLib's InterpLUT throws in createLUT() with fewer than 2 points, throws if the points are not
     * strictly increasing, and throws in get() when the input is at or past either end of the table.
     */
    public static class SafeLut {
        private static final double EDGE_EPSILON = 1e-6;

        private final TreeMap<Double, Double> points = new TreeMap<>();
        private InterpLUT lut = null;

        public void add(double input, double output) {
            if (Double.isNaN(input) || Double.isNaN(output)) return;
            points.put(input, output);
            lut = null; // rebuilt on the next createLUT()/get()
        }

        public void createLUT() {
            lut = null;
            if (points.size() < 2) return;

            InterpLUT fresh = new InterpLUT();
            for (Map.Entry<Double, Double> p : points.entrySet()) {
                fresh.add(p.getKey(), p.getValue());
            }
            fresh.createLUT(); // points are sorted and unique here, so this cannot throw
            lut = fresh;
        }

        public int size() {
            return points.size();
        }

        public double minX() {
            return points.isEmpty() ? 0 : points.firstKey();
        }

        public double maxX() {
            return points.isEmpty() ? 0 : points.lastKey();
        }

        /** True if get(input) is a real interpolation (not clamped to an edge or a constant). */
        public boolean isInRange(double input) {
            if (points.size() < 2) return true;
            return input >= points.firstKey() && input <= points.lastKey();
        }

        public double get(double input) {
            if (points.isEmpty()) return 0;
            if (points.size() == 1) return points.firstEntry().getValue();
            if (lut == null) createLUT();

            // InterpLUT.get() throws at the exact endpoints too, so stay a hair inside them.
            double lo = points.firstKey() + EDGE_EPSILON;
            double hi = points.lastKey() - EDGE_EPSILON;
            if (hi <= lo) return points.firstEntry().getValue();

            double x = Double.isNaN(input) ? lo : Math.min(Math.max(input, lo), hi);
            return lut.get(x);
        }
    }
}
