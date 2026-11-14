package org.firstinspires.ftc.teamcode.teleop.systems.Shooter.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * UnderShooter: one full set of Shooter settings (see ShooterConfig for what each one does).
 * The Shooter uses it when Shooter.activeConfig is UNDER_SHOOTER (change it in Panels, under Shooter).
 * Edit the values live in Panels under UnderShooter -> values, then copy the ones you keep in here.
 *
 * Hardware:
 *   - 2 in wheel
 *   - 5800 rpm output
 */
@Configurable
public class UnderShooter {
    public static ShooterConfig values = new ShooterConfig();

    static {
        // -------------- Turret position --------------
        values.turretPollenXOffset = 0.0;
        values.turretPollenYOffset = 0.0;
        values.turretNectarXOffset = 0.0;
        values.turretNectarYOffset = 0.0;
        values.turretWrapBias = 10;

        // -------------- Flywheel motors --------------
        values.shooter1Inverted = true;
        values.shooter2Inverted = false; // reversed from CRShooter

        // -------------- Flywheel PIDF --------------
        values.shooterP = .002654;
        values.shooterI = 0;
        values.shooterD = 0;
        values.kV = .00037;
        values.kS = .04;

        values.notShootingTolerance = 50;
        values.poweringTolerance = 40;
        values.shootingTolerance = 40;

        // -------------- Flywheel speed --------------
        values.velocityRounding = 10;
        values.idleVelocity = 1800;
        values.flywheelTicksPerRev = 28;

        // -------------- Shot table --------------
        // The comments are just labels: if you change a distance, update the comments to match.
        values.lutDistances = new double[]{
                0, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70
        };
        // 1.8x CRShooter's speeds
        values.lutVelocities = new double[]{
                1800, //  0 in (empty slot, ignored)
                1800, // 20 in
                1825, // 25 in
                1850, // 30 in
                1875, // 35 in
                1900, // 40 in
                1925, // 45 in
                1950, // 50 in
                1967, // 55 in
                1980, // 60 in
                2000, // 65 in
                2000, // 70 in
        };
        values.lutHoods = new double[]{
                .7, //  0 in (empty slot, ignored)
                .7, // 20 in
                0.47, // 25 in
                0.4, // 30 in
                0.34, // 35 in
                0.28, // 40 in
                0.3, // 45 in
                0.22, // 50 in
                0.16, // 55 in
                0.12, // 60 in
                0.10, // 65 in
                0.05, // 70 in
        };

        values.useShootingKpLut = true;
        values.lutShootingKps = new double[]{
                0.008, //  0 in (empty slot, ignored)
                0.008, // 20 in
                0.008, // 25 in
                0.008, // 30 in
                0.008, // 35 in
                0.008, // 40 in
                0.01, // 45 in
                0.01, // 50 in
                0.01, // 55 in
                0.01, // 60 in
                0.012, // 65 in
                0.012, // 70 in
        };

        values.velocityTrim = 0;
        values.hoodTrim = 0;
    }
}
