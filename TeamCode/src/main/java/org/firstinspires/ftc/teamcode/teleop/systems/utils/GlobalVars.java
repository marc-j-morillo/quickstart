package org.firstinspires.ftc.teamcode.teleop.systems.utils;
import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

@Configurable
public class GlobalVars {
    @IgnoreConfigurable public static boolean inAuto = false;

    @IgnoreConfigurable public static boolean isRed = false;

    public static int waitBeforeShoot = 300; // ms to settle after a path ends, before powering the shooter in auto
    public static int waitAfterShoot = 600; // Temp value for the amount of time it takes to shoot in auto
    public static int flowerIntakeWait = 3000; // ms to sit at a flower after the intake path ends, still intaking
    public static double flowerIntakeSpeed = 0.5; // paths into a flower run at this fraction of normal speed (0-1]
    public static double flowerHeadingEnd = 0.75; // paths into a flower finish turning at this fraction of the path
}
