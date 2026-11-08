package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

public class FollowerPose {
    public static final Pose defaultStartPose = new Pose(70.75, 70.75, 0); // middle of the field, 0 deg
    public static Pose startPose = defaultStartPose; // autos store their end pose here for teleop

    /**
     * Returns the pose the last auto left in startPose, then resets startPose to defaultStartPose.
     * So a teleop run with no auto before it starts at the middle of the field, not at an old auto's pose.
     */
    public static Pose takeStartPose() {
        Pose pose = startPose;
        startPose = defaultStartPose;
        return pose;
    }
    public static Pose currentPose = new Pose(70.75, 70.75, 0);
    public static double botX = currentPose.x();
    public static double botY = currentPose.y();
    public static double botHeading = Math.toDegrees(currentPose.heading()); // DEGREES (Shooter turret math expects degrees)

    // Velocity measured by the localizer (Pinpoint), FIELD frame: in/s along field x / y, deg/s counterclockwise.
    // Only set by update(Pose, Velocity) / setVelocity(). velocityTimeNanos = when it was last set (0 = never),
    // so the Shooter can tell a fresh reading from an old one (see ShootOnTheMove).
    public static double velX = 0, velY = 0, turnRateDeg = 0;
    public static long velocityTimeNanos = 0;


    /** Pose only. The velocity is left alone (it goes stale, and ShootOnTheMove falls back to pose differences). */
    public static void update(Pose newPose) {
        currentPose = newPose;

        botX = currentPose.x();
        botY = currentPose.y();
        botHeading = Math.toDegrees(currentPose.heading());
    }

    /** Pose plus the localizer's velocity (Pedro's follower.velocity(): field frame, in/s and rad/s). */
    public static void update(Pose newPose, Velocity velocity) {
        update(newPose);
        setVelocity(velocity.vx, velocity.vy, Math.toDegrees(velocity.omega));
    }

    /** Field-frame velocity: in/s along field x / y, turn rate in deg/s (counterclockwise positive). */
    public static void setVelocity(double xInPerSec, double yInPerSec, double turnRateDegPerSec) {
        velX = xInPerSec;
        velY = yInPerSec;
        turnRateDeg = turnRateDegPerSec;
        velocityTimeNanos = System.nanoTime();
    }
}
