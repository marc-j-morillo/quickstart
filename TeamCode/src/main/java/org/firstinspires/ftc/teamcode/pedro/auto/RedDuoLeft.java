package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.waitAfterShoot;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.startPose;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.LoopTimer;

@Autonomous(name = "RedDuoLeft", group = "Autonomous")
public class RedDuoLeft extends LinearOpMode {

    Robot robot;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.2382, 124.7217, 90);
    private final Pose shootLeftStart = poseFactory.of(8.2382, 124.7217, 0);
    private final Pose shootLeft = poseFactory.of(47.1667, 115, 45);
    private final Pose intakeFlower = poseFactory.of(47.1667, 129.6765, 90);
    private final Pose intakeFlowerSegment1Start = poseFactory.of(47.1667, 129.6765, 45);
    private final Pose intakeFlowerSegment1End = poseFactory.of(47.1667, 129.6765, 90);
    private final Pose intakeFlowerSegment2Heading = poseFactory.of(47.1667, 129.6765, 90);
    private final Pose shootLeft2 = poseFactory.of(47.1667, 115, 45);
    private final Pose shootLeft2Segment1Heading = poseFactory.of(47.1667, 115, 90);
    private final Pose shootLeft2Segment2Start = poseFactory.of(47.1667, 115, 90);
    private final Pose shootLeft2Segment2End = poseFactory.of(47.1667, 115, 45);
    private final Pose park = poseFactory.of(15, 120, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, shootLeft()),
                waitMs(5000),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),

                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),

                follow(follower, intakeFlower()),

                parallel(
                        follow(follower, shootLeft2()),
                        sequential(
                                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                                waitMs(waitAfterShoot)
                        )
                ),
                waitMs(500),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                instant(() -> Robot.flowerWedgeIsActive = false),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        inAuto = true;
        isRed = true;
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        follower = robot.getFollower();
        follower.setPose(shootLeftStart);
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

    public Path shootLeft() {
        return line(shootLeftStart, shootLeft).linear(shootLeftStart, shootLeft);
    }

    public Path intakeFlower() {
        return line(shootLeft, intakeFlower).heading(Interpolator.piecewise().until(0.4485, Interpolator.linear(intakeFlowerSegment1Start, intakeFlowerSegment1End)).until(1, Interpolator.constant(intakeFlowerSegment2Heading)));
    }

    public Path shootLeft2() {
        return line(intakeFlower, shootLeft2).heading(Interpolator.piecewise().until(0.4875, Interpolator.constant(shootLeft2Segment1Heading)).until(1, Interpolator.linear(shootLeft2Segment2Start, shootLeft2Segment2End)));
    }

    public Path park() {
        return line(shootLeft2, park).linear(shootLeft2, park);
    }
}
