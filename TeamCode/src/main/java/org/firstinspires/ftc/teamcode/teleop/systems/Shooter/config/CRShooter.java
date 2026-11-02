package org.firstinspires.ftc.teamcode.teleop.systems.Shooter.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * CRShooter: one full set of Shooter settings (see ShooterConfig for what each one does).
 * The Shooter uses it when Shooter.activeConfig is CR_SHOOTER (change it in Panels, under Shooter).
 * Edit the values live in Panels under CRShooter -> values, then copy the ones you keep in here.
 *
 * Hardware:
 *   - 2 in wheel
 *   - CR 1:1 tangential velocity
 *   - 2900 rpm final output
 *   - two motors
 *   - one goBILDA weight, switching to two
 */
@Configurable
public class CRShooter {
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
        values.idleVelocity = 1600;
        values.flywheelTicksPerRev = 28;

        // -------------- Shot table --------------
        // The comments are just labels: if you change a distance, update the comments to match.
        values.lutDistances = new double[]{
                0, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70
        };
        values.lutVelocities = new double[]{
                1660, //  0 in (empty slot, ignored)
                1660, // 20 in
                1660, // 25 in
                1660, // 30 in
                1680, // 35 in
                1680, // 40 in
                1700, // 45 in
                1725, // 50 in
                1750, // 55 in
                1765, // 60 in
                1780, // 65 in
                1790, // 70 in
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
