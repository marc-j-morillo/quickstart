package org.firstinspires.ftc.teamcode.teleop.systems.utils;
import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

@Configurable
public class GlobalVars {
    @IgnoreConfigurable public static boolean inAuto = false;

    @IgnoreConfigurable public static boolean isRed = false;

    public static int waitAfterShoot = 300; // Temp value for the amount of time it takes to shoot in auto
}
