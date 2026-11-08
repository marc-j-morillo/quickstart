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
 *   - two goBILDA weights
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
        values.shooterP = .00234;
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
                1660, //  0 in (empty slot, ignored) - 0
                1660, // 20 in - 1
                1665, // 25 in - 2
                1665, // 30 in - 3
                1680, // 35 in - 4
                1680, // 40 in - 5
                1700, // 45 in - 6
                1750, // 50 in - 7
                1740, // 55 in - 8
                1765, // 60 in - 9
                1780, // 65 in - 10
                1790, // 70 in - 11
        };
        values.lutHoods = new double[]{
                0, //  0 in (empty slot, ignored) - 0
                0, // 20 in - 1
                0.05, // 25 in - 2
                0.075, // 30 in - 3
                0.15, // 35 in - 4
                0.18, // 40 in - 5
                0.22, // 45 in - 6
                0.25, // 50 in - 7
                0.275, // 55 in - 8
                0.3, // 60 in - 9
                0.325, // 65 in - 10
                0.35, // 70 in - 11
        };

        values.useShootingKpLut = true;
        values.lutShootingKps = new double[]{
                0.00265, //  0 in (empty slot, ignored) - 0
                0.0026, // 20 in - 1
                0.0027, // 25 in - 2
                0.0027, // 30 in - 3
                0.0027, // 35 in - 4
                0.00275, // 40 in - 5
                0.0027, // 45 in - 6
                0.0029, // 50 in - 7
                0.00285, // 55 in - 8
                0.00285, // 60 in - 9
                0.0029, // 65 in - 10
                0.0029, // 70 in - 11
        };

        // PLACEHOLDER GUESSES (0.40 s at 20 in to 0.60 s at 70 in): measure with slow-mo video and replace.
        // A time that is 20% off only leaves 20% of the ball-flight correction undone.
        values.lutTimesOfFlight = new double[]{
                0.40, //  0 in (empty slot, ignored) - 0
                0.40, // 20 in - 1
                0.42, // 25 in - 2
                0.44, // 30 in - 3
                0.46, // 35 in - 4
                0.48, // 40 in - 5
                0.50, // 45 in - 6
                0.52, // 50 in - 7
                0.54, // 55 in - 8
                0.56, // 60 in - 9
                0.58, // 65 in - 10
                0.60, // 70 in - 11
        };

        values.velocityTrim = 0;
        values.hoodTrim = 0;
    }
}
