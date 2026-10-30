package org.firstinspires.ftc.teamcode.teleop;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
//import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;

@TeleOp(name = "DriveTest", group = "TeleOp")
//@Configurable
public class DriveTest extends OpMode {

    //  ------------- Drive -------------
    Drive drive;

    @Override
    public void init() {
        FollowerPose.update(new Pose(70.75, 70.75, Math.toRadians(90)));
        drive = new Drive(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        drive.update();
    }
}
