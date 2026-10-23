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

@Autonomous(name = "RedSolo3Flower", group = "Autonomous")
public class RedSolo3Flower extends LinearOpMode {

    Robot robot;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startAndShootPreload = poseFactory.of(54.0612, 8.2201, 90);
    private final Pose intakeAllianceFlower = poseFactory.of(11.8235, 46.9519, 180);
    private final Pose intakeAllianceFlowerControl1 = poseFactory.of(25.1704, 47.4704, 0);
    private final Pose shootLeft1 = poseFactory.of(47.1667, 115, 60);
    private final Pose shootLeft1Control1 = poseFactory.of(32.4282, 44.863, 0);
    private final Pose shootLeft1Control2 = poseFactory.of(24.7852, 95.1585, 0);
    private final Pose intakeLeftFlower = poseFactory.of(47.1667, 129.6765, 90);
    private final Pose shootLeft2 = poseFactory.of(47.1667, 115, 60);
    private final Pose shootLeft2Segment1Heading = poseFactory.of(47.1667, 115, 90);
    private final Pose shootLeft2Segment2Start = poseFactory.of(47.1667, 115, 90);
    private final Pose shootLeft2Segment2End = poseFactory.of(47.1667, 115, 60);
    private final Pose moveToGarden = poseFactory.of(27.1281, 24.1733, -89.1749);
    private final Pose moveToGardenControl1 = poseFactory.of(25.9496, 98.5156, 0);
    private final Pose moveToGardenSegment1Start = poseFactory.of(27.1281, 24.1733, 60);
    private final Pose moveToGardenSegment1End = poseFactory.of(27.1281, 24.1733, -100);
    private final Pose intakeGarden = poseFactory.of(7.694, 6.7067, 180);
    private final Pose intakeGardenControl1 = poseFactory.of(30.29, 7.694, 0);
    private final Pose intakeGardenSegment1Start = poseFactory.of(7.694, 6.7067, -90);
    private final Pose intakeGardenSegment1End = poseFactory.of(7.694, 6.7067, 180);
    private final Pose intakeGardenSegment2Heading = poseFactory.of(7.694, 6.7067, 180);
    private final Pose shootGarden = poseFactory.of(60, 20, 192);
    private final Pose intakeRightFlower = poseFactory.of(94.4381, 11.8235, 270);
    private final Pose intakeRightFlowerControl1 = poseFactory.of(94.7193, 26.6519, 0);
    private final Pose shootFinal = poseFactory.of(60, 20, 11.7889);
    private final Pose shootFinalControl1 = poseFactory.of(94.1615, 27.2074, 0);
    private final Pose park = poseFactory.of(15, 90, 90);
    private final Pose parkControl1 = poseFactory.of(11.4339, 56.772, 0);
    private final Pose parkSegment1Start = poseFactory.of(15, 90, 11.7889);
    private final Pose parkSegment1End = poseFactory.of(15, 90, 125);
    private final Pose parkSegment3Start = poseFactory.of(15, 90, 87.7245);
    private final Pose parkSegment3End = poseFactory.of(15, 90, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                follow(follower, intakeAllianceFlower()),

                parallel(
                        follow(follower, shootLeft1()),
                        sequential(
                                waitMs(1000),
                                instant(() -> Robot.flowerWedgeIsActive = false),
                                waitMs(500),
                                instant(() -> Robot.transportState = Robot.TransportState.HOME)
                        )
                ),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),

                follow(follower, intakeLeftFlower()),

                parallel(
                        follow(follower, shootLeft2()),
                        sequential(
                                waitMs(500),
                                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                                instant(() -> Robot.flowerWedgeIsActive = false),
                                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                                waitMs(waitAfterShoot)
                        )
                ),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, moveToGarden()),

                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                follow(follower, intakeGarden()),

                parallel(
                        follow(follower, shootGarden()),
                        sequential(
                                waitMs(700),
                                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                                waitMs(waitAfterShoot)
                        )
                ),

                instant(() -> Robot.flowerWedgeIsActive = true),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                follow(follower, intakeRightFlower()),

                parallel(
                        follow(follower, shootFinal()),
                        sequential(
                                waitMs(700),
                                instant(() -> Robot.flowerWedgeIsActive = false),
                                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                                waitMs(waitAfterShoot)
                        )
                ),

                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        inAuto = true;
        isRed = true;
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startAndShootPreload);
        follower.update();

        startPose = follower.pose();
        FollowerPose.update(follower.pose());

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            robot.update(follower.pose());

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
        return curve(startAndShootPreload, intakeAllianceFlowerControl1, intakeAllianceFlower).linear(startAndShootPreload, intakeAllianceFlower);
    }

    public Path shootLeft1() {
        return curve(intakeAllianceFlower, shootLeft1Control1, shootLeft1Control2, shootLeft1).linear(intakeAllianceFlower, shootLeft1);
    }

    public Path intakeLeftFlower() {
        return line(shootLeft1, intakeLeftFlower).constant(intakeLeftFlower);
    }

    public Path shootLeft2() {
        return line(intakeLeftFlower, shootLeft2).heading(Interpolator.piecewise().until(0.5076, Interpolator.constant(shootLeft2Segment1Heading)).until(1, Interpolator.linear(shootLeft2Segment2Start, shootLeft2Segment2End)));
    }

    public Path moveToGarden() {
        return curve(shootLeft2, moveToGardenControl1, moveToGarden).heading(Interpolator.piecewise().until(0.4918, Interpolator.linear(moveToGardenSegment1Start, moveToGardenSegment1End)).until(1, Interpolator.tangent));
    }

    public Path intakeGarden() {
        return curve(moveToGarden, intakeGardenControl1, intakeGarden).heading(Interpolator.piecewise().until(0.3326, Interpolator.linear(intakeGardenSegment1Start, intakeGardenSegment1End)).until(1, Interpolator.constant(intakeGardenSegment2Heading)));
    }

    public Path shootGarden() {
        return line(intakeGarden, shootGarden).linear(intakeGarden, shootGarden);
    }

    public Path intakeRightFlower() {
        return curve(shootGarden, intakeRightFlowerControl1, intakeRightFlower).linear(shootGarden, intakeRightFlower);
    }

    public Path shootFinal() {
        return curve(intakeRightFlower, shootFinalControl1, shootFinal).reverseTangent();
    }

    public Path park() {
        return curve(shootFinal, parkControl1, park).heading(Interpolator.piecewise().until(0.3593, Interpolator.linear(parkSegment1Start, parkSegment1End)).until(0.9631, Interpolator.tangent).until(1, Interpolator.linear(parkSegment3Start, parkSegment3End)));
    }
}