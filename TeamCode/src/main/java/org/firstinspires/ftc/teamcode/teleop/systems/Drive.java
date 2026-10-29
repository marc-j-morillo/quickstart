package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botX;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botY;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.startPose;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;


@Configurable
public class Drive {


    Follower follower;
    /*
//    public static double secondaryDriveP = .01, secondaryDriveI, secondaryDriveD = .001, secondaryDriveF;
    public static double driveP = 0, driveI = 0, driveD = 0, driveF = 0; // Change these values
    private PIDFController turnController = new PIDFController(driveP, driveI, driveD, driveF);
//    private PIDFController secondaryTurnController = new PIDFController(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF);
//    public double IMUOffset;
    */
    public boolean fieldCentric;

    @IgnoreConfigurable public static boolean isDriving = false;

    public static double stickThreshold = 0.05; // for isDriving calcs

    // Relocalization (share = corner pose, dpad down = heading only), field inches / degrees
    public static double redCornerX = 113.520177165, blueCornerX = 31.479822835, cornerY = 136.4, cornerHeading = 90;
    public static double redResetHeading = 0, blueResetHeading = 180;


    public Drive(HardwareMap hardwareMap, boolean fieldCentric) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        this.fieldCentric = fieldCentric;
    }

    public void update(Gamepad gamepad1) {

        // Set isDriving and fieldCentric according to gamepad imputs
        if (Math.abs(gamepad1.left_stick_x) > stickThreshold
                || Math.abs(gamepad1.left_stick_y) > stickThreshold
                || Math.abs(gamepad1.right_stick_x) > stickThreshold) {
            isDriving = true;
        } else {
            isDriving = false;
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        // Take inputs to relocalize
        if (gamepad1.dpadDownWasPressed()) {
            resetIMU();
        }

        if (gamepad1.shareWasPressed()) {
            resetLocalization();
        }

        // Drive
        if (fieldCentric) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);
        } else {
            follower.manual(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }
        follower.update();

        // Update the FolowerPose.java variables
        FollowerPose.update(follower.pose());
    }

    public void update() {
        follower.update();
        FollowerPose.update(follower.pose());
    }

    public void resetLocalization() {
        Pose cornerPose;
        if (isRed) {
            cornerPose = new Pose(redCornerX, cornerY, Math.toRadians(cornerHeading));
        } else {
            cornerPose = new Pose(blueCornerX, cornerY, Math.toRadians(cornerHeading));
        }
        follower.setPose(cornerPose);
    }

    public void resetIMU() {
        Pose newHeadingPose;
        if (isRed) {
            newHeadingPose = new Pose(botX, botY, Math.toRadians(redResetHeading));
        } else {
            newHeadingPose = new Pose(botX, botY, Math.toRadians(blueResetHeading));
        }
        follower.setPose(newHeadingPose);
    }
}
