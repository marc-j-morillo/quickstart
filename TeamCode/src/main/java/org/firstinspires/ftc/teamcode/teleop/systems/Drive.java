package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botX;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botY;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.botHeading;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


public class Drive {


    Follower follower;
    Pose robotPose;
    /*
//    public static double secondaryDriveP = .01, secondaryDriveI, secondaryDriveD = .001, secondaryDriveF;
    public static double driveP = 0, driveI = 0, driveD = 0, driveF = 0; // Change these values
    private PIDFController turnController = new PIDFController(driveP, driveI, driveD, driveF);
//    private PIDFController secondaryTurnController = new PIDFController(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF);
//    public double IMUOffset;
    */
    public boolean fieldCentric;

    public static boolean isDriving = false;

    public static final double stickThreshold = 0.05; // for isDriving calcs


    public Drive(HardwareMap hardwareMap, boolean fieldCentric) {
        follower = Constants.create(hardwareMap);

        this.fieldCentric = fieldCentric;
    }

    public void update(Gamepad gamepad1) {
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
        robotPose = follower.pose();

        botX = robotPose.x();
        botY = robotPose.y();
        botHeading = Math.toDegrees(robotPose.heading());

        if (gamepad1.dpadDownWasPressed()) {
            resetIMU();
        }

        if (gamepad1.shareWasPressed()) {
            resetLocalization();
        }
    }

    public void update() {
        follower.update();
        robotPose = follower.pose();

        botX = robotPose.x();
        botY = robotPose.y();
        botHeading = Math.toDegrees(robotPose.heading());
    }

    public void resetLocalization() {
        Pose cornerPose;
        if (isRed) {
            cornerPose = new Pose(113.520177165, 136.4, Math.toRadians(90));
        } else {
            cornerPose = new Pose(31.479822835, 136.4, Math.toRadians(90));
        }
        follower.setPose(cornerPose);
    }

    public void resetIMU() {
        Pose newHeadingPose;
        if (isRed) {
            newHeadingPose = new Pose(botX, botY, Math.toRadians(0));
        } else {
            newHeadingPose = new Pose(botX, botY, Math.toRadians(180));
        }
        follower.setPose(newHeadingPose);
    }
}
