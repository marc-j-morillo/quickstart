package org.firstinspires.ftc.teamcode.teleop.systems.Shooter.config;

/**
 * Every tunable Shooter setting. This class only lists the settings: the values live in CRShooter and
 * UnderShooter (one full set each), and Shooter.activeConfig in Panels picks which set
 * the Shooter uses. Read the active set in code with Shooter.config().
 *
 * Why a separate class: Panels only shows the fields a class declares itself, so the configs can't be
 * subclasses of each other. Instead each one holds its own ShooterConfig object, which Panels shows in full.
 */
public class ShooterConfig {
    // -------------- Turret position --------------
    // Offsets from odometry center to turret center, in inches, in the robot frame: X = forward, Y = left.
    public double turretPollenXOffset;
    public double turretPollenYOffset;
    public double turretNectarXOffset;
    public double turretNectarYOffset;

    // The turret angle wrap goes to (-180 + turretWrapBias, 180 + turretWrapBias], so a target straight behind
    // the robot always resolves to the + side instead of flipping between +180 and -180 with localizer noise.
    public double turretWrapBias;

    // -------------- Flywheel PIDF --------------
    public double shooterP, shooterI, shooterD, kV, kS;

    // Ready-to-fire tolerances (ticks/s) for each transport state
    public double notShootingTolerance;
    public double poweringTolerance;
    public double shootingTolerance;

    // -------------- Flywheel speed --------------
    public double velocityRounding;     // round the LUT velocity to the nearest this many ticks/s
    // When not shooting (HOME / INTAKE / OUTTAKE) in teleop, the flywheel idles at this speed instead of the LUT
    // speed. In ticks/s, the same units as the LUT velocities and the "Flywheel target" telemetry.
    public double idleVelocity;
    public double flywheelTicksPerRev;  // flywheel motor encoder ticks per revolution (goBILDA bare motor = 28)

    // -------------- Shot table --------------
    // Each index is one shot: at lutDistances[i] inches, spin to lutVelocities[i] and set the hood to lutHoods[i].
    // A distance <= 0 marks an empty slot. Only slots that exist in ALL three arrays are used, so keep them the
    // same length. Editable live from Panels: Shooter rebuilds the LUTs on its next update whenever these change.
    public double[] lutDistances = new double[0];
    public double[] lutVelocities = new double[0];
    public double[] lutHoods = new double[0];

    // Flywheel kP while SHOOTING in the competition teleops, by distance (same distances as lutDistances).
    // Has its own slot count, so it can be shorter than lutDistances without dropping velocity/hood points.
    // useShootingKpLut = false: SHOOTING uses plain shooterP like the other states, and this table is ignored.
    public boolean useShootingKpLut;
    public double[] lutShootingKps = new double[0];

    // Ball time of flight in SECONDS, by distance (same distances as lutDistances, own slot count like the kP table):
    // from leaving the shooter to reaching the goal. Used by ShootOnTheMove to aim ahead of the robot's motion.
    // Measure with a slow-motion video: frames from leaving to reaching the goal / the video's fps.
    // Only CRShooter's table is used (whichever config is active), so UnderShooter leaves this empty.
    // Empty = no ball-flight correction (the lead correction still runs).
    public double[] lutTimesOfFlight = new double[0];

    // Added on top of the LUT output (LutTuner adjusts these, then bakes them into a new point)
    public double velocityTrim;
    public double hoodTrim;
}
