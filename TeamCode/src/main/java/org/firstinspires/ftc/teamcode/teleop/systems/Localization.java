package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;

public class Localization {
    //public static GoBildaPinpointDriver odo;
    public static double botHeading;
    public static double botX;
    public static double botY;

    Follower follower;
    Pose robotPose;

    public Localization(HardwareMap hardwareMap) {
        // odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        // odo.setOffsets(3.7795275591, 5.1968503937, DistanceUnit.INCH); // Change these values
        // odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // botHeading = odo.getHeading(AngleUnit.RADIANS);
        // botX = odo.getPosX(DistanceUnit.INCH);
        // botY = odo.getPosY(DistanceUnit.INCH);
    }

    public void createFollower(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    public void update() {
        follower.update();
        robotPose = follower.pose();

        botX = robotPose.x();
        botY = robotPose.y();
        botHeading = Math.toDegrees(robotPose.heading());
    }

    public void update(Gamepad gamepad1) {
        if (gamepad1.dpad_down) {
            resetIMU();
        }

        if (gamepad1.share) {
            resetLocalization();
        }
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
            //cornerPose = new Pose(113.520177165, 136.4, folower.pose().heading());
        } else {
            cornerPose = new Pose(31.479822835, 136.4, Math.toRadians(90));
            //cornerPose = new Pose(113.520177165, 136.4, folower.pose().heading());
        }
        follower.setPose(cornerPose);
    }

    public void resetIMU() {
        Pose newHeadingPose;
        if (isRed) {
            newHeadingPose = new Pose(botX, botY, Math.toRadians(45));
        } else {
            newHeadingPose = new Pose(botX, botY, Math.toRadians(135));
        }
        follower.setPose(newHeadingPose);
    }
}