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
                1800, //  0 in (empty slot, ignored) - 0
                1800, // 20 in - 1
                1825, // 25 in - 2
                1850, // 30 in - 3
                1875, // 35 in - 4
                1925, // 40 in - 5
                1950, // 45 in - 6
                1970, // 50 in - 7
                1990, // 55 in - 8
                1990, // 60 in - 9
                2000, // 65 in - 10
                2000, // 70 in - 11
        };
        values.lutHoods = new double[]{
                .7, //  0 in (empty slot, ignored) - 0
                .7, // 20 in - 1
                0.47, // 25 in - 2
                0.4, // 30 in - 3
                0.34, // 35 in - 4
                0.28, // 40 in - 5
                0.3, // 45 in - 6
                0.22, // 50 in - 7
                0.16, // 55 in - 8
                0.12, // 60 in - 9
                0.10, // 65 in - 10
                0.05, // 70 in - 11
        };

        values.useShootingKpLut = true;
        values.lutShootingKps = new double[]{
                0.008, //  0 in (empty slot, ignored) - 0
                0.008, // 20 in - 1
                0.008, // 25 in - 2
                0.008, // 30 in - 3
                0.008, // 35 in - 4
                0.008, // 40 in - 5
                0.01, // 45 in - 6
                0.01, // 50 in - 7
                0.01, // 55 in - 8
                0.01, // 60 in - 9
                0.012, // 65 in - 10
                0.012, // 70 in - 11
        };

        values.velocityTrim = 0;
        values.hoodTrim = 0;
    }
}
