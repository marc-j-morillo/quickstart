package org.firstinspires.ftc.teamcode.teleop.systems.utils;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;

public class GlobalVars {
    public static boolean inAuto = false;

    public static boolean isRed = false;

    public static Pose startPose = new Pose(15, 90, 0);

    public static int waitAfterShoot = 300; // Temp value for the amount of time it takes to shoot in auto
}
