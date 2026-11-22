package org.firstinspires.ftc.teamcode.teleop.systems.utils;
import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

@Configurable
public class GlobalVars {
    @IgnoreConfigurable public static boolean inAuto = false;

    @IgnoreConfigurable public static boolean isRed = false;

    public static int waitBeforeShoot = 750; // ms to settle after a path ends, before powering the shooter in auto
    public static int waitAfterShoot = 1000; // Temp value for the amount of time it takes to shoot in auto
    public static int flowerIntakeWait = 3000; // ms to sit at a flower after the intake path ends, still intaking
    // Most ms a path into a flower may take. If the robot hasn't reached the end by then (stuck on the flower), the
    // auto moves on to flowerIntakeWait anyway; the follower keeps pushing toward the end until the next path starts.
    public static int flowerPathTimeout = 1000;
    public static int allianceIntakeTimeout = 1000;
    public static int allianceIntakeWait = 1000;
    public static int gardenWait = 1000;
    public static int gardenTimeout = 750;
    public static double doubleParkSpeed = 0.7; // RedSolo3TipPartnerPark's last park path runs at this fraction of normal speed
    public static double flowerIntakeSpeed = 0.5; // paths into a flower run at this fraction of normal speed (0-1]
    public static double flowerHeadingEnd = 0.75; // paths into a flower finish turning at this fraction of the path
}
