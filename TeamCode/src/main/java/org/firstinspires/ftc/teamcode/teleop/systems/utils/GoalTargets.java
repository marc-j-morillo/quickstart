package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

/**
 * Field points (inches, same frame as the localizer): the points the Shooter aims at, and the
 * relocalize pose for the competition teleops.
 * There are two goal points per alliance, RIGHT and LEFT. Only red is tuned: blue uses the same point
 * mirrored across the field's center line (x -> fieldWidth - x, same y).
 */
@Configurable
public class GoalTargets {
    public static double redRightX = 58;
    public static double redRightY = 58;
    public static double redLeftX = 62.0;
    public static double redLeftY = 83;
    public static double fieldWidth = 141.5; // inches

    // Which goal point is in use. The competition teleops switch it with gamepad2.
    // The Shooter constructor resets it to RIGHT, so a previous opmode's choice doesn't carry over.
    @IgnoreConfigurable public static boolean leftSide = false;

    /** Goal x for the current alliance and side. */
    public static double goalX() {
        double redX = leftSide ? redLeftX : redRightX;
        return isRed ? redX : fieldWidth - redX;
    }

    /** Goal y for the current side (the mirror keeps y). */
    public static double goalY() {
        return leftSide ? redLeftY : redRightY;
    }

    // -------------- Relocalize pose --------------
    // Where the robot is when the driver presses share in the competition teleops (field inches, heading in degrees).
    // Red is tuned, blue is mirrored: x -> fieldWidth - x, heading -> 180 - heading.
    public static double redRelocalizeX = 113.520177165;
    public static double redRelocalizeY = 136.4;
    public static double redRelocalizeHeading = 90;

    public static double relocalizeX() {
        return isRed ? redRelocalizeX : fieldWidth - redRelocalizeX;
    }

    public static double relocalizeY() {
        return redRelocalizeY;
    }

    public static double relocalizeHeading() {
        return isRed ? redRelocalizeHeading : 180 - redRelocalizeHeading;
    }
}
