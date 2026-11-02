package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botX;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botY;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.config.CRShooter;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.config.UnderShooter;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;


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

    // -------------- Settings --------------
    // Every tunable setting lives in the config folder: ShooterConfig lists them, and CRShooter / UnderShooter each
    // hold a full set of values. activeConfig (Panels, under Shooter) picks the set the Shooter uses; switching it
    // applies on the next update. Read the active set in code with Shooter.config().
    public enum Config { CR_SHOOTER, UNDER_SHOOTER }
    public static Config activeConfig = Config.CR_SHOOTER;

    /** The settings the Shooter is using right now (the set picked by activeConfig). */
    public static ShooterConfig config() {
        switch (activeConfig) {
            case UNDER_SHOOTER: return UnderShooter.values;
            case CR_SHOOTER:
            default: return CRShooter.values;
        }
    }

    // -------------- Goal --------------
    // The goal points (RIGHT / LEFT, red tuned, blue mirrored) live in GoalTargets.

    /** Goal x for the current alliance and side. */
    public static double allianceGoalX() {
        return GoalTargets.goalX();
    }

    /** Goal y for the current alliance and side. */
    public static double allianceGoalY() {
        return GoalTargets.goalY();
    }

    // -------------- Driver adjustments --------------
    // Changed with gamepad2 in the competition teleops. Reset to 0 whenever a Shooter is created,
    // so one opmode's adjustments never leak into the next (e.g. teleop into the next auto).
    @IgnoreConfigurable public static double turretAngleOffset = 0; // degrees, + = counterclockwise (left)
    @IgnoreConfigurable public static double velocityOffsetRpm = 0; // rpm, added to the shooting speed

    // Optional aim-point override (used by the test opmodes). When enabled, turret aim, hood angle and
    // flywheel velocity all use this field point (inches, same frame as botX/botY) instead of the goal.
    @IgnoreConfigurable public static boolean useTargetOverride = false;
    @IgnoreConfigurable public static double targetOverrideX = 70.75;
    @IgnoreConfigurable public static double targetOverrideY = 70.75;


    // FLYWHEEL CLASS AND VARIABLES
    Flywheel flywheel;

    public static final double goalHeightInches = 65.6;

    //Shooter PID (gains are zeroed while driving or when the target is 0)
    private PIDFController shooterController;

    public double shooterpid;

    //MOTOR VELOCITY
    @IgnoreConfigurable public static double shooterVelocity;

    //MOTOR TARGETS
    @IgnoreConfigurable public static double shooterVelocityTarget;

    @IgnoreConfigurable public static double fireTolerance = 40;


    // SafeLut never throws and clamps the lookup distance to the table's range (see SafeLut at the bottom).
    public SafeLut velocityLut = new SafeLut();
    public SafeLut hoodAngleLut = new SafeLut();
    public SafeLut shootingKpLut = new SafeLut();
    private double[] builtDistances, builtVelocities, builtHoods, builtShootingKps;

    @IgnoreConfigurable public static double distanceFromLimelightToTagInches;
    @IgnoreConfigurable public static double lutVelocity, lutHood, lutShootingKp; // raw LUT output at the current distance
    @IgnoreConfigurable public static double shooterX, shooterY;   // field position of the turret

    public Shooter(HardwareMap hardwareMap) {
        rebuildLutsIfChanged();

        turretPollen = new TurretPollen(hardwareMap);
        //turretNectar = new TurretNectar(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hoodPollen = new HoodPollen(hardwareMap);
        shooterController = new PIDFController(config().shooterP, config().shooterI, config().shooterD, config().kV);

        shooterVelocity = dormant;
        shooterVelocityTarget = dormant;
        hoodPollenPosition = 0;

        turretAngleOffset = 0;
        velocityOffsetRpm = 0;
        GoalTargets.leftSide = false;
    }

    public void update(double voltageMultiplier, boolean isDriving) {
        updateAim();

        // Set this loop's target before running the controller, so it isn't a loop behind
        switch (Robot.transportState) {
            case HOME:
                shooterVelocityTarget = calcVelocity(false);
                fireTolerance = config().notShootingTolerance;
                break;
            case INTAKE:
                shooterVelocityTarget = calcVelocity(false);
                fireTolerance = config().notShootingTolerance;
                break;
            case OUTTAKE:
                shooterVelocityTarget = calcVelocity(false);
                fireTolerance = config().notShootingTolerance;
                break;
            case POWER_SHOOTER:
                shooterVelocityTarget = calcVelocity(true);
                fireTolerance = config().poweringTolerance;
                break;
            case SHOOT:
                shooterVelocityTarget = calcVelocity(true);
                fireTolerance = config().shootingTolerance;
                break;
        }

        boolean idle = shooterVelocityTarget == 0 || isDriving;
        double p = idle ? 0 : config().shooterP;
        double f = idle ? 0 : config().kV * voltageMultiplier;
        runController(shooterVelocityTarget, p, f, voltageMultiplier);
    }

    /**
     * Turret aim, hood and the LUT lookups (velocity, hood, shooting kP) for the current pose. update() calls this;
     * opmodes that run the flywheel themselves call this and then runFlywheel().
     */
    public void updateAim() {
        rebuildLutsIfChanged();
        updateShooterPosition(true);

        double deltaX = goalX() - shooterX;
        double deltaY = goalY() - shooterY;
        distanceFromLimelightToTagInches = Math.hypot(deltaX, deltaY);
        lutVelocity = velocityLut.get(distanceFromLimelightToTagInches);
        lutHood = hoodAngleLut.get(distanceFromLimelightToTagInches);
        lutShootingKp = shootingKpLut.get(distanceFromLimelightToTagInches);

        turretTargetAnglePollen = turretTargetToGoal(deltaX, deltaY) + turretAngleOffset;
        turretPollen.aimAt(turretTargetAnglePollen);
        turretTargetPositionPollen = turretPollen.getCommandedPosition();
        //turretTargetPositionNectar = turretTargetToGoal(false);
        //turretNectar.update(turretTargetPosition);

        hoodPollenPosition = HoodPollen.clamp(lutHood + config().hoodTrim);
        hoodPollen.update(hoodPollenPosition);
    }

    /**
     * Runs the flywheel PIDF toward targetVelocity (ticks/s) with this P and the shared shooterI, shooterD, kV, kS.
     * kV and kS are scaled by voltageMultiplier. A target of 0 turns the flywheel off.
     */
    public void runFlywheel(double targetVelocity, double p, double voltageMultiplier) {
        double f = targetVelocity == 0 ? 0 : config().kV * voltageMultiplier;
        runController(targetVelocity, p, f, voltageMultiplier);
    }

    /** The LUT shooting speed (ticks/s) for the current distance, with velocityTrim, rounding and the rpm offset. */
    public double lutTargetVelocity() {
        return calcVelocity(true);
    }

    private void runController(double targetVelocity, double p, double f, double voltageMultiplier) {
        shooterVelocityTarget = targetVelocity;
        shooterVelocity = flywheel.getVelocity();
        shooterController.setPIDF(p, config().shooterI, config().shooterD, f);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + (Math.signum(shooterVelocityTarget) * config().kS * voltageMultiplier);
        flywheel.update(Range.clip(shooterpid, -1.0, 1.0));
    }


    // -------------------------------- AIM POINT ---------------------------------

    /** Field position of the turret: odometry center plus the turret offset rotated by the heading. */
    private void updateShooterPosition(boolean isPollen) {
        double xOffset = isPollen ? config().turretPollenXOffset : config().turretNectarXOffset;
        double yOffset = isPollen ? config().turretPollenYOffset : config().turretNectarYOffset;
        double heading = Math.toRadians(botHeading);

        shooterX = botX + xOffset * Math.cos(heading) - yOffset * Math.sin(heading);
        shooterY = botY + xOffset * Math.sin(heading) + yOffset * Math.cos(heading);
    }

    private double goalX() {
        if (useTargetOverride) return targetOverrideX;
        return allianceGoalX();
    }

    private double goalY() {
        if (useTargetOverride) return targetOverrideY;
        return allianceGoalY();
    }


    // -------------------------------- TURRET METHODS ---------------------------------

    /** Wraps a turret angle to one turn, biased by config().turretWrapBias (see ShooterConfig). */
    public static double angleWrap(double degrees) {
        while (degrees > 180 + config().turretWrapBias)  degrees -= 360;
        while (degrees <= -180 + config().turretWrapBias) degrees += 360;
        return degrees;
    }

    /** Turret angle (degrees, 0 = robot-forward) that points at the goal. TurretPollen clamps it to its limits. */
    private double turretTargetToGoal(double deltaX, double deltaY) {
        double goalAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        return angleWrap(goalAngle - botHeading);
    }


    /** Turns the turret servo output off or on. The Shooter keeps aiming either way. */
    public void setTurretOutputEnabled(boolean enabled) {
        turretPollen.setOutputEnabled(enabled);
    }

    public boolean isTurretOutputEnabled() {
        return turretPollen.isOutputEnabled();
    }


    // -------------------------------- FLYWHEEL METHODS ---------------------------------


    public boolean inRange() {
        if (Math.abs(shooterVelocity - shooterVelocityTarget) < fireTolerance) {
            return true;
        } else {
            return false;
        }
    }

    /** Flywheel target in ticks/s: the LUT speed when shooting (or in auto), otherwise idleVelocity. */
    private double calcVelocity(boolean isShooting) {
        if (!isShooting && !inAuto) {
            return config().idleVelocity;
        }

        // Lookup Velocity & Round to Nearest velocityRounding
        double rawVelocity = lutVelocity + config().velocityTrim;
        double rounded = config().velocityRounding > 0
                ? Math.round(rawVelocity / config().velocityRounding) * config().velocityRounding
                : rawVelocity;
        // The driver offset goes on after rounding, so every 15 rpm step actually changes the target
        return rounded + rpmToTicksPerSecond(velocityOffsetRpm);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * config().flywheelTicksPerRev / 60.0;
    }

    // -------------------------------- SHOT TABLE ---------------------------------

    /** Rebuilds velocityLut / hoodAngleLut / shootingKpLut from the lut arrays if any of them changed since the last build. */
    public void rebuildLutsIfChanged() {
        // Read the active config once, so switching configs mid-rebuild can't mix two configs' tables.
        // Switching configs changes the arrays, so the tables rebuild automatically.
        ShooterConfig c = config();
        if (Arrays.equals(c.lutDistances, builtDistances)
                && Arrays.equals(c.lutVelocities, builtVelocities)
                && Arrays.equals(c.lutHoods, builtHoods)
                && Arrays.equals(c.lutShootingKps, builtShootingKps)) {
            return;
        }

        SafeLut kps = new SafeLut();
        int kpPoints = Math.min(c.lutDistances.length, c.lutShootingKps.length);
        for (int i = 0; i < kpPoints; i++) {
            if (c.lutDistances[i] <= 0) continue; // empty slot
            kps.add(c.lutDistances[i], c.lutShootingKps[i]);
        }
        kps.createLUT();
        shootingKpLut = kps;
        builtShootingKps = c.lutShootingKps.clone();

        SafeLut velocities = new SafeLut();
        SafeLut hoods = new SafeLut();
        int points = Math.min(c.lutDistances.length, Math.min(c.lutVelocities.length, c.lutHoods.length));
        for (int i = 0; i < points; i++) {
            if (c.lutDistances[i] <= 0) continue; // empty slot
            velocities.add(c.lutDistances[i], c.lutVelocities[i]);
            hoods.add(c.lutDistances[i], c.lutHoods[i]);
        }
        velocities.createLUT();
        hoods.createLUT();
        velocityLut = velocities;
        hoodAngleLut = hoods;

        builtDistances = c.lutDistances.clone();
        builtVelocities = c.lutVelocities.clone();
        builtHoods = c.lutHoods.clone();
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
