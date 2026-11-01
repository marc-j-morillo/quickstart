package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.pedropathing.math.Pose;

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


    public static void update(Pose newPose) {
        currentPose = newPose;

        botX = currentPose.x();
        botY = currentPose.y();
        botHeading = Math.toDegrees(currentPose.heading());
    }
}
