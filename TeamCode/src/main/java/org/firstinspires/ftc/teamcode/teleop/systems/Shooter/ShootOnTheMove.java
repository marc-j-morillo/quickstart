package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;

/**
 * Shoot on the move: corrects the turret aim (and a little of the flywheel speed / hood) for the robot moving,
 * e.g. while it gets pushed. The Shooter runs this every updateAim(), so every opmode that uses the Shooter gets it.
 *
 * Two effects are corrected:
 *  1. Lag ("lead"). The turret reaches its target a bit late (output smoothing + servo + loop time), so the aim is
 *     computed for the pose the robot will be at leadSeconds from now: pose + velocity x lead, heading + turn rate x lead.
 *     This matters most when the robot gets spun: at 90 deg/s, 0.1 s of lag is 9 deg of turret error.
 *  2. Ball flight. The ball keeps the robot's velocity after it leaves, so it drifts by velocity x time of flight.
 *     Aim at a "virtual goal" instead: virtual goal = goal - shooter velocity x time of flight. The time of flight
 *     depends on the distance, which depends on the virtual goal, so it is repeated a few times (it settles fast).
 *     The flywheel speed and hood use the distance to the virtual goal too (compensateDistance). That change is small:
 *     10 in/s straight toward the goal x 0.5 s of flight is 5 in closer, about 7-10 ticks/s on the CRShooter table.
 *
 * Velocity comes from the Pinpoint (Pedro's follower.velocity(), field frame), or, if that isn't available or
 * velocitySource = POSE_DIFFERENCE, from the pose change between loops. Both are low-pass filtered, then a small
 * deadband zeroes the noise, so a robot sitting still aims EXACTLY like it did without this.
 * Speeds are capped (maxSpeed / maxTurnRate) so a bad reading or full-speed driving can't swing the turret far.
 *
 * Frames: field inches, heading degrees counterclockwise (same as FollowerPose). Ball height is not involved:
 * the robot only moves the ball sideways.
 */
@Configurable
public class ShootOnTheMove {
    // -------------- On / off --------------
    public static boolean enabled = true;
    public static boolean enabledInAuto = true; // also needs enabled

    // -------------- What gets corrected --------------
    public static boolean compensateFlight = true;   // virtual goal (ball keeps the robot's velocity)
    public static boolean compensateDistance = true; // flywheel speed + hood from the virtual goal's distance
    // Lead = extraLeadMs + TurretPollen.smoothingTimeMs (if leadIncludesTurretSmoothing). The smoothing filter makes
    // the turret trail a steady turn by exactly smoothingTimeMs; extraLeadMs covers the servo and the loop.
    public static double extraLeadMs = 40;
    public static boolean leadIncludesTurretSmoothing = true;
    public static int iterations = 3; // virtual goal repeats (1 is usually within a fraction of an inch already)

    // -------------- Velocity reading --------------
    public enum VelocitySource { PINPOINT, POSE_DIFFERENCE }
    public static VelocitySource velocitySource = VelocitySource.PINPOINT;
    public static double pinpointStaleMs = 100;   // older Pinpoint readings than this -> use the pose difference
    public static double filterTimeMs = 40;       // low-pass time constant on the velocity, 0 = off
    // Pose-difference samples faster than this are a relocalize / heading reset, not motion, and are skipped
    public static double maxPlausibleSpeed = 150;     // in/s (the drivetrain tops out around 91)
    public static double maxPlausibleTurnRate = 720;  // deg/s

    // -------------- Deadband and caps --------------
    // Below the deadband the correction is 0; it ramps in up to 2x the deadband, then is the full measured value.
    public static double speedDeadband = 1.5;     // in/s
    public static double turnRateDeadband = 4;    // deg/s
    public static double maxSpeed = 20;           // in/s, pushes are slow; caps the correction while driving
    public static double maxTurnRate = 180;       // deg/s

    // -------------- Telemetry (outputs) --------------
    @IgnoreConfigurable public static double pinpointVelX, pinpointVelY, pinpointTurnRate; // last Pinpoint reading
    @IgnoreConfigurable public static double poseDiffVelX, poseDiffVelY, poseDiffTurnRate; // from the pose change
    @IgnoreConfigurable public static boolean usingPinpoint;  // false = this loop used the pose difference
    @IgnoreConfigurable public static double velX, velY, turnRate; // what the correction used (filtered, deadband, cap)
    @IgnoreConfigurable public static double leadSeconds, timeOfFlight;
    @IgnoreConfigurable public static double aimShiftDeg;     // turret angle change from the correction
    @IgnoreConfigurable public static double distanceShift;   // inches, virtual goal distance - real distance

    // -------------- Result of the last update() --------------
    public double aimX, aimY;          // point to aim at (the virtual goal; the real goal if off / still)
    public double shooterX, shooterY;  // predicted turret position (field inches)
    public double headingDeg;          // predicted robot heading
    public double aimDistance;         // predicted turret position -> aim point
    public double realDistance;        // turret position now -> real goal (no correction)

    // -------------- State --------------
    private long lastPoseNanos = 0;
    private double lastX, lastY, lastHeading;
    private boolean filterStarted = false;
    private long lastSampleNanos = 0;
    private double filteredX, filteredY, filteredTurn;

    /**
     * Call once per loop with the current pose. xOffset / yOffset = turret center from the odometry center
     * (robot frame, inches, x forward, y left). timeOfFlightLut = seconds of flight by distance (empty = no
     * flight correction).
     */
    public void update(double botX, double botY, double botHeadingDeg, double xOffset, double yOffset,
                       double goalX, double goalY, Shooter.SafeLut timeOfFlightLut) {
        long now = System.nanoTime();
        measureVelocity(botX, botY, botHeadingDeg, now);

        // Uncorrected aim, for realDistance and the aimShiftDeg telemetry
        double h = Math.toRadians(botHeadingDeg);
        double nowShooterX = botX + xOffset * Math.cos(h) - yOffset * Math.sin(h);
        double nowShooterY = botY + xOffset * Math.sin(h) + yOffset * Math.cos(h);
        realDistance = Math.hypot(goalX - nowShooterX, goalY - nowShooterY);

        boolean active = enabled && (!inAuto || enabledInAuto);
        if (!active) {
            velX = velY = turnRate = 0;
        } else {
            applyDeadbandAndCaps();
        }

        // ----- 1. Lead: where the robot will be when the turret gets there -----
        leadSeconds = active
                ? Math.max(0, extraLeadMs + (leadIncludesTurretSmoothing ? TurretPollen.smoothingTimeMs : 0)) / 1000.0
                : 0;
        headingDeg = botHeadingDeg + turnRate * leadSeconds;
        double ph = Math.toRadians(headingDeg);
        double offsetX = xOffset * Math.cos(ph) - yOffset * Math.sin(ph); // offset in the field frame
        double offsetY = xOffset * Math.sin(ph) + yOffset * Math.cos(ph);
        shooterX = botX + velX * leadSeconds + offsetX;
        shooterY = botY + velY * leadSeconds + offsetY;

        // ----- 2. Ball flight: aim at goal - turret velocity x time of flight -----
        // Turret velocity = center velocity + turn rate x offset (a turret off-center swings sideways when turning)
        double w = Math.toRadians(turnRate);
        double shooterVelX = velX - w * offsetY;
        double shooterVelY = velY + w * offsetX;

        aimX = goalX;
        aimY = goalY;
        timeOfFlight = 0;
        if (active && compensateFlight && (shooterVelX != 0 || shooterVelY != 0)) {
            for (int i = 0; i < Math.max(1, iterations); i++) {
                timeOfFlight = Math.max(0, timeOfFlightLut.get(Math.hypot(aimX - shooterX, aimY - shooterY)));
                aimX = goalX - shooterVelX * timeOfFlight;
                aimY = goalY - shooterVelY * timeOfFlight;
            }
        }
        aimDistance = Math.hypot(aimX - shooterX, aimY - shooterY);

        // ----- Telemetry -----
        double correctedAngle = Math.toDegrees(Math.atan2(aimY - shooterY, aimX - shooterX)) - headingDeg;
        double plainAngle = Math.toDegrees(Math.atan2(goalY - nowShooterY, goalX - nowShooterX)) - botHeadingDeg;
        aimShiftDeg = wrap180(correctedAngle - plainAngle);
        distanceShift = aimDistance - realDistance;
    }

    /** Forgets the last pose and filter, e.g. after a relocalize (the pose difference skips jumps anyway). */
    public void reset() {
        lastPoseNanos = 0;
        filterStarted = false;
        filteredX = filteredY = filteredTurn = 0;
    }

    // -------------------------------- VELOCITY ---------------------------------

    private void measureVelocity(double x, double y, double headingDeg, long now) {
        // ----- Pose difference (always computed, so it can be compared with the Pinpoint in telemetry) -----
        boolean poseDiffValid = false;
        double dt = (now - lastPoseNanos) / 1e9;
        if (lastPoseNanos != 0 && dt < 1e-3) return; // called twice in one loop: wait for a real time step
        if (lastPoseNanos != 0 && dt < 0.25) {
            double vx = (x - lastX) / dt;
            double vy = (y - lastY) / dt;
            double w = wrap180(headingDeg - lastHeading) / dt;
            if (Math.hypot(vx, vy) <= maxPlausibleSpeed && Math.abs(w) <= maxPlausibleTurnRate) {
                poseDiffVelX = vx;
                poseDiffVelY = vy;
                poseDiffTurnRate = w;
                poseDiffValid = true;
            }
        }
        lastPoseNanos = now;
        lastX = x;
        lastY = y;
        lastHeading = headingDeg;

        // ----- Pinpoint (set in FollowerPose by Drive / Robot from Pedro's follower.velocity()) -----
        pinpointVelX = FollowerPose.velX;
        pinpointVelY = FollowerPose.velY;
        pinpointTurnRate = FollowerPose.turnRateDeg;
        boolean pinpointFresh = FollowerPose.velocityTimeNanos != 0
                && (now - FollowerPose.velocityTimeNanos) / 1e6 <= pinpointStaleMs;
        boolean pinpointValid = pinpointFresh
                && Double.isFinite(pinpointVelX) && Double.isFinite(pinpointVelY) && Double.isFinite(pinpointTurnRate)
                && Math.hypot(pinpointVelX, pinpointVelY) <= maxPlausibleSpeed
                && Math.abs(pinpointTurnRate) <= maxPlausibleTurnRate;

        // ----- Pick one -----
        usingPinpoint = velocitySource == VelocitySource.PINPOINT && pinpointFresh;
        boolean valid;
        double rawX, rawY, rawTurn;
        if (usingPinpoint) {
            valid = pinpointValid;
            rawX = pinpointVelX;
            rawY = pinpointVelY;
            rawTurn = pinpointTurnRate;
        } else {
            valid = poseDiffValid;
            rawX = poseDiffVelX;
            rawY = poseDiffVelY;
            rawTurn = poseDiffTurnRate;
        }

        if (!valid) {
            // A skipped sample holds the last value; no good sample for a while means we don't know: use 0
            if (lastSampleNanos == 0 || (now - lastSampleNanos) / 1e6 > 250) {
                filteredX = filteredY = filteredTurn = 0;
                filterStarted = false;
            }
            return;
        }

        // ----- Low-pass filter (time-based, so it acts the same at any loop time) -----
        if (!filterStarted || filterTimeMs <= 0) {
            filteredX = rawX;
            filteredY = rawY;
            filteredTurn = rawTurn;
            filterStarted = true;
        } else {
            double dtMs = Math.min((now - lastSampleNanos) / 1e6, 100);
            double alpha = 1 - Math.exp(-dtMs / filterTimeMs);
            filteredX += alpha * (rawX - filteredX);
            filteredY += alpha * (rawY - filteredY);
            filteredTurn += alpha * (rawTurn - filteredTurn);
        }
        lastSampleNanos = now;
    }

    /** Deadband (ramps in from 1x to 2x the deadband, so the turret never jumps) and caps, into velX/velY/turnRate. */
    private void applyDeadbandAndCaps() {
        double speed = Math.hypot(filteredX, filteredY);
        double scale = rampIn(speed, speedDeadband);
        double capped = Math.min(speed * scale, Math.max(0, maxSpeed));
        double toSpeed = speed > 1e-9 ? capped / speed : 0;
        velX = filteredX * toSpeed;
        velY = filteredY * toSpeed;

        double turn = Math.abs(filteredTurn);
        double turnCapped = Math.min(turn * rampIn(turn, turnRateDeadband), Math.max(0, maxTurnRate));
        turnRate = Math.signum(filteredTurn) * turnCapped;
    }

    /** 0 at or below the deadband, 1 at 2x the deadband and above, a straight line between. */
    private static double rampIn(double value, double deadband) {
        if (deadband <= 0) return 1;
        return Math.min(Math.max((value - deadband) / deadband, 0), 1);
    }

    private static double wrap180(double degrees) {
        return ((degrees % 360) + 540) % 360 - 180;
    }
}
