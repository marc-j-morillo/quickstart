package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.test.pinpoint.GoBildaPinpointDriver;

// uncomment if we have a pinpoint
/*
public class Localization {
    public static GoBildaPinpointDriver odo;
    public static double botHeading;
    public static double botX;
    public static double botY;
    public Localization(HardwareMap hardwareMap) {
        // odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        // odo.setOffsets(3.7795275591, 5.1968503937, DistanceUnit.INCH);
        // odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        botHeading = odo.getHeading(AngleUnit.RADIANS);
        botX = odo.getPosX(DistanceUnit.INCH);
        botY = odo.getPosY(DistanceUnit.INCH);
    }

    public void update() {
        odo.update();
        botHeading = odo.getHeading(AngleUnit.RADIANS);
        botX = odo.getPosX(DistanceUnit.INCH);
        botY = odo.getPosY(DistanceUnit.INCH);
    }

    public void update(Gamepad gamepad1) {
        if (gamepad1.dpad_down) {
            resetIMU();
        }

        if (gamepad1.share) {
            resetLocalization();
        }
        odo.update();
        botHeading = odo.getHeading(AngleUnit.RADIANS);
        botX = odo.getPosX(DistanceUnit.INCH);
        botY = odo.getPosY(DistanceUnit.INCH);
    }

    public void resetLocalization() {
        if (isRed) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 113.520177165, 136.4, AngleUnit.RADIANS, Math.toRadians(90)));
        } else {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 31.479822835, 136.4, AngleUnit.RADIANS, Math.toRadians(90)));
        }
//        Localization.odo.setPosX(Vision.visionBotX, DistanceUnit.INCH);
//        Localization.odo.setPosY(Vision.visionBotY, DistanceUnit.INCH);
    }

    public void resetIMU() {
        if (isRed) {
            odo.setHeading(Math.toRadians(45), AngleUnit.RADIANS);
        } else {
            odo.setHeading(Math.toRadians(135), AngleUnit.RADIANS);
        }
    }
}
*/