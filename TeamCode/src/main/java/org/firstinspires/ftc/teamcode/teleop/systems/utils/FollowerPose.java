package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.pedropathing.math.Pose;

public class FollowerPose {
    public static Pose startPose = new Pose(15, 90, 0);
    public static Pose currentPose = new Pose(15, 90, 0);
    public static double botX = currentPose.x();
    public static double botY = currentPose.y();
    public static double botHeading = currentPose.heading();


    public static void update(Pose newPose) {
        currentPose = newPose;

        botX = currentPose.x();
        botY = currentPose.y();
        botHeading = currentPose.heading();
    }
}
