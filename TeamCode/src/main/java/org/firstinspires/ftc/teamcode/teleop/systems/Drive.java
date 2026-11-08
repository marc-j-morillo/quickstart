package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

//import kotlin._Assertions;

/**
 * Drivetrain + localization through the Pedro follower (pedro/Constants: Mecanum drivetrain, Pinpoint localizer).
 * There is only one follower: autos path with it through Robot.getFollower(), teleops drive it with drive(gamepad).
 * Every update publishes the pose to FollowerPose, which the Shooter aims from.
 *
 * Pedro's DrivePowers are (forward, strafe, turn) with strafe + = left and turn + = counterclockwise,
 * so the stick x axes are negated.
 */
@Configurable
public class Drive {
    Follower follower;

    public boolean fieldCentric;

    @IgnoreConfigurable public static boolean isDriving = false;

    public static double stickThreshold = 0.05;  // for isDriving
    public static double strafeMultiplier = 1.1; // counteracts imperfect mecanum strafing

    // Field-centric: "away from the driver" is this field heading (deg). Also what resetHeading() sets.
    public static double redDriverHeading = 0, blueDriverHeading = 180;



    public Drive(HardwareMap hardwareMap, boolean fieldCentric) {
        follower = Constants.create(hardwareMap); // resets the Pinpoint and waits for its IMU to recalibrate
        // The pose the last auto ended at (used once), or the middle of the field at 0 deg if no auto ran.
        // Autos set their own start pose right after this.
        follower.setPose(FollowerPose.takeStartPose());
        update();

        this.fieldCentric = fieldCentric;
    }


    // Constructor for competition tele that automatically sets the start position
    public Drive(HardwareMap hardwareMap, boolean fieldCentric, boolean isCompTele) {
        follower = Constants.create(hardwareMap); // resets the Pinpoint and waits for its IMU to recalibrate
        // The pose the last auto ended at (used once), or the middle of the field at 0 deg if no auto ran.
        // Autos set their own start pose right after this.
        if (isCompTele) {
            setStartTeleOp();
        } else {
            follower.setPose(FollowerPose.takeStartPose());
        }
        update();

        this.fieldCentric = fieldCentric;
    }

    /**
     * Teleop drive from gamepad sticks (left stick = translate, right stick x = turn), field-centric if fieldCentric.
     * Also updates the follower and FollowerPose, so call this instead of update() once per loop.
     */
    public void drive(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x; // Pedro: + = left
        double turn = -gamepad.right_stick_x;  // Pedro: + = counterclockwise

        isDriving = Math.abs(forward) > stickThreshold
                || Math.abs(strafe) > stickThreshold
                || Math.abs(turn) > stickThreshold;

        DrivePowers powers = fieldCentric
                // Rotates the stick by -(heading - driverHeading), so "up" is always away from the driver
                ? ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading(), -Math.toRadians(driverHeading()))
                : new DrivePowers(forward, strafe, turn);
        follower.manual(new DrivePowers(powers.forward(), powers.strafe() * strafeMultiplier, powers.turn()));

        update();
    }

    /**
     * Teleop drive with the Robot-teleop buttons: left stick button toggles field-centric, dpad down resets the
     * heading, share relocalizes to the GoalTargets relocalize pose.
     */
    public void update(Gamepad gamepad1) {
        if (gamepad1.leftStickButtonWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.dpadDownWasPressed()) resetHeading();
        if (gamepad1.shareWasPressed()) relocalize();
        FollowerPose.update(follower.pose());

        drive(gamepad1);
    }

    /**
     * Updates the follower (localizer + whatever it's doing) and FollowerPose (pose + the Pinpoint's field-frame
     * velocity, for shoot on the move). For autos and when not driving.
     */
    public void update() {
        follower.update();
        FollowerPose.update(follower.pose(), follower.velocity());
    }

    public Follower getFollower() {
        return follower;
    }

    // -------------- Relocalization (field inches / degrees) --------------

    public void setPose(double xInches, double yInches, double headingDegrees) {
        follower.setPose(new Pose(xInches, yInches, Math.toRadians(headingDegrees)));
    }

    /** Moves x/y only, keeps the heading. */
    public void setPosition(double xInches, double yInches) {
        follower.setX(xInches);
        follower.setY(yInches);
    }

    /** Sets the heading only, keeps x/y. */
    public void setHeading(double headingDegrees) {
        follower.setHeading(Math.toRadians(headingDegrees));
    }

    /** Heading reset with the robot facing away from the driver. */
    public void resetHeading() {
        setHeading(driverHeading());
    }

    /** Relocalize to the GoalTargets relocalize pose (configurable, blue mirrored). */
    public void relocalize() {
        setPose(GoalTargets.relocalizeX(), GoalTargets.relocalizeY(), GoalTargets.relocalizeHeading());
    }

    public void setStartTeleOp() {
        setPose(15, 90, 90);
    }

    public static double driverHeading() {
        return isRed ? redDriverHeading : blueDriverHeading;
    }
}
