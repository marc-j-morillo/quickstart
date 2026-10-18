package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

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
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;

@Autonomous(name = "BlueSolo3Flower", group = "Autonomous")
public class BlueSolo3Flower extends LinearOpMode {
    Robot robot;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startAndShootPreload = poseFactory.of(87.4, 133.5, 90);
    private final Pose intakeAllianceFlower = poseFactory.of(129.6765, 94.5, 0);
    private final Pose intakeAllianceFlowerControl1 = poseFactory.of(116.3296, 94, 0);
    private final Pose shootLeft1 = poseFactory.of(94.3333, 26.5, -120);
    private final Pose shootLeft1Control1 = poseFactory.of(105.7177, 90.3526, 0);
    private final Pose shootLeft1Control2 = poseFactory.of(116.7148, 46.38, 0);
    private final Pose intakeLeftFlower = poseFactory.of(94.3333, 11.8235, -90);
    private final Pose shootLeft2 = poseFactory.of(94.3333, 26.5, -120);
    private final Pose shootLeft2Segment1Heading = poseFactory.of(94.3333, 26.5, -90);
    private final Pose shootLeft2Segment2Start = poseFactory.of(94.3333, 26.5, -90);
    private final Pose shootLeft2Segment2End = poseFactory.of(94.3333, 26.5, -120);
    private final Pose moveToGarden = poseFactory.of(114.3719, 117.3, 90.8256);
    private final Pose moveToGardenControl1 = poseFactory.of(115.5504, 43, 0);
    private final Pose moveToGardenSegment1Start = poseFactory.of(114.3719, 117.3, -120);
    private final Pose moveToGardenSegment1End = poseFactory.of(114.3719, 117.3, -280);
    private final Pose intakeGarden = poseFactory.of(133.8059, 134.8, 0);
    private final Pose intakeGardenControl1 = poseFactory.of(109.7236, 136.1782, 0);
    private final Pose intakeGardenSegment1Start = poseFactory.of(133.8059, 134.8, 90);
    private final Pose intakeGardenSegment1End = poseFactory.of(133.8059, 134.8, 0);
    private final Pose intakeGardenSegment2Heading = poseFactory.of(133.8059, 134.8, 0);
    private final Pose shootGarden = poseFactory.of(81.5, 121.5, 12);
    private final Pose intakeRightFlower = poseFactory.of(47.0619, 129.7, 90);
    private final Pose intakeRightFlowerControl1 = poseFactory.of(46.7807, 114.8, 0);
    private final Pose shootFinal = poseFactory.of(81.5, 121.5, -169.025);
    private final Pose shootFinalControl1 = poseFactory.of(47.3385, 114.8, 0);
    private final Pose park = poseFactory.of(126.5, 51.5, -90);
    private final Pose parkControl1 = poseFactory.of(130.0661, 84.73, 0);
    private final Pose parkSegment1Start = poseFactory.of(126.5, 51.5, -168);
    private final Pose parkSegment1End = poseFactory.of(126.5, 51.5, -55);
    private final Pose parkSegment3Start = poseFactory.of(126.5, 51.5, -92.2757);
    private final Pose parkSegment3End = poseFactory.of(126.5, 51.5, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(300),
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
                waitMs(300),
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
                                waitMs(300)
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
                                waitMs(300)
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
                                waitMs(300)
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

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            robot.update();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.startPose = follower.pose();
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
        return line(intakeLeftFlower, shootLeft2).heading(Interpolator.piecewise().until(0.6262, Interpolator.constant(shootLeft2Segment1Heading)).until(1, Interpolator.linear(shootLeft2Segment2Start, shootLeft2Segment2End)));
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