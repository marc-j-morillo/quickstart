package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.pedropathing.math.Pose;

public class FollowerPose {
    public static Pose startPose = new Pose(70.75, 70.75, 0);
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
