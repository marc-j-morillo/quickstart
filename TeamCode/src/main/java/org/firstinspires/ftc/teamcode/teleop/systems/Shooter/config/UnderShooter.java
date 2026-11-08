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

        // -------------- Flywheel PIDF --------------
        values.shooterP = .002654;
        values.shooterI = 0;
        values.shooterD = 0;
        values.kV = .000325;
        values.kS = .032;

        values.notShootingTolerance = 50;
        values.poweringTolerance = 40;
        values.shootingTolerance = 150;

        // -------------- Flywheel speed --------------
        values.velocityRounding = 10;
        values.idleVelocity = 2880; // 1.8x CRShooter's 1600
        values.flywheelTicksPerRev = 28;

        // -------------- Shot table --------------
        // The comments are just labels: if you change a distance, update the comments to match.
        values.lutDistances = new double[]{
                0, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70
        };
        // 1.8x CRShooter's speeds
        values.lutVelocities = new double[]{
                2988, //  0 in (empty slot, ignored)
                2988, // 20 in
                2988, // 25 in
                2988, // 30 in
                3024, // 35 in
                3024, // 40 in
                3060, // 45 in
                3105, // 50 in
                3150, // 55 in
                3177, // 60 in
                3204, // 65 in
                3222, // 70 in
        };
        values.lutHoods = new double[]{
                0, //  0 in (empty slot, ignored)
                0, // 20 in
                0.05, // 25 in
                0.075, // 30 in
                0.15, // 35 in
                0.18, // 40 in
                0.22, // 45 in
                0.25, // 50 in
                0.275, // 55 in
                0.3, // 60 in
                0.325, // 65 in
                0.35, // 70 in
        };

        values.useShootingKpLut = true;
        values.lutShootingKps = new double[]{
                0.00265, //  0 in (empty slot, ignored)
                0.00265, // 20 in
                0.00265, // 25 in
                0.00265, // 30 in
                0.00265, // 35 in
                0.00265, // 40 in
                0.00265, // 45 in
                0.00265, // 50 in
                0.00265, // 55 in
                0.00265, // 60 in
                0.00265, // 65 in
                0.00265, // 70 in
        };

        values.velocityTrim = 0;
        values.hoodTrim = 0;
    }
}
