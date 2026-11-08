package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.waitAfterShoot;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.startPose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.ResetStaticValues;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.LoopTimer;

@Autonomous(name = "ShootPreload_Left", group = "Autonomous")
public class ShootPreload_Left extends LinearOpMode {
    Robot robot;
    ResetStaticValues resetStaticValues;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 7, 90);
    private final Pose intakeAllianceFlower = poseFactory.of(11.8235, 46.9519, 180);
    private final Pose intakeAllianceFlowerControl1 = poseFactory.of(25.1704, 47.4704, 0);
    private final Pose shootLeft1Start = poseFactory.of(11.8235, 46.9519, 180);
    private final Pose shootLeft1 = poseFactory.of(47.1667, 115, 60);
    private final Pose shootLeft1Control1 = poseFactory.of(32.4282, 44.863, 0);
    private final Pose shootLeft1Control2 = poseFactory.of(24.7852, 95.1585, 0);
    private final Pose parkStart = poseFactory.of(47.1667, 115, 60);
    private final Pose park = poseFactory.of(15, 90, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                waitMs(200),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitMs(1000),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                follow(follower, intakeAllianceFlower()),
                instant(() -> GoalTargets.leftSide = true),
                parallel(
                        follow(follower, shootLeft1()),
                        sequential(
                                waitMs(2000),
                                instant(() -> Robot.flowerWedgeIsActive = false),
                                waitMs(1000),
                                instant(() -> Robot.transportState = Robot.TransportState.HOME)
                        )
                ),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                waitMs(1000),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        resetStaticValues = new ResetStaticValues();
        resetStaticValues.resetStaticValues();

        inAuto = true;
        isRed = true;
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        follower = robot.getFollower();
        follower.setPose(start);
        follower.update();

        startPose = follower.pose();
        FollowerPose.update(follower.pose());

        waitForStart();
        schedule(autoRoutine());

        LoopTimer loopTimer = new LoopTimer();
        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            robot.update(follower.pose());
            loopTimer.update();
            loopTimer.addTelemetry(telemetry);

            telemetry.addData("Robot state", Robot.transportState);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
                telemetry.addData("Current path", follower.currentPath());
            }

            telemetry.update();
        }
        startPose = follower.pose();
    }

    public Path intakeAllianceFlower() {
        return Paths.curve(start, intakeAllianceFlowerControl1, intakeAllianceFlower).linear(start, intakeAllianceFlower);
    }

    public Path shootLeft1() {
        return Paths.curve(shootLeft1Start, shootLeft1Control1, shootLeft1Control2, shootLeft1).linear(shootLeft1Start, shootLeft1);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
