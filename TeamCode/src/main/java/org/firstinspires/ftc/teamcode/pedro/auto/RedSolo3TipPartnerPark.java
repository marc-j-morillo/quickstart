package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.allianceIntakeTimeout;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.allianceIntakeWait;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.doubleParkSpeed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.gardenTimeout;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.gardenWait;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.waitAfterShoot;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.waitBeforeShoot;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerIntakeWait;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerPathTimeout;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerIntakeSpeed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerHeadingEnd;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.startPose;

import com.pedropathing.api.Paths;
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
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GoalTargets;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.LoopTimer;

@Autonomous(name = "RedSolo3TipPartnerPark", group = "Autonomous")
public class RedSolo3TipPartnerPark extends LinearOpMode {
    Robot robot;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Blue copies extend this class and override isRedAlliance(). Every pose below is the red one, mirrored for blue
    // (x -> fieldWidth - x, heading -> 180 - heading, same as GoalTargets), so blue stays an exact copy of red.
    protected boolean isRedAlliance() {
        return true;
    }

    private Pose pose(double x, double y, double headingDeg) {
        if (isRedAlliance()) return poseFactory.of(x, y, headingDeg);
        return poseFactory.of(GoalTargets.fieldWidth - x, y, 180 - headingDeg);
    }

    private final Pose start = pose(58, 8, 90);
    private final Pose shootPreload = pose(58, 20, 90);
    private final Pose driveToAllianceFlowerStart = pose(58, 20, 90);
    private final Pose driveToAllianceFlower = pose(24, 46.67, 180);
    private final Pose driveToAllianceFlowerControl1 = pose(44.6976, 41.9276, 0);
    private final Pose intakeAllianceFlower = pose(15.7, 46.67, 180);
    private final Pose backOutofAllianceFlower = pose(24, 46.67, 180);
    private final Pose shootLeft1 = pose(47.1667, 115, 60);
    private final Pose shootLeft1Control1 = pose(24.7852, 95.1585, 0);
    private final Pose shootLeft1Segment1Start = pose(47.1667, 115, 180);
    private final Pose shootLeft1Segment1End = pose(47.1667, 115, 90);
    private final Pose shootLeft1Segment3Start = pose(47.1667, 115, 49.8226);
    private final Pose shootLeft1Segment3End = pose(47.1667, 115, 60);
    private final Pose intakeLeftFlower = pose(47.1667, 128.7, 90);
    private final Pose shootLeft2 = pose(47.1667, 115, 60);
    private final Pose shootLeft2Segment1Heading = pose(47.1667, 115, 90);
    private final Pose shootLeft2Segment2Start = pose(47.1667, 115, 90);
    private final Pose shootLeft2Segment2End = pose(47.1667, 115, 60);
    private final Pose drivetoAlliancePreloadsStart = pose(47.1667, 115, 60);
    private final Pose drivetoAlliancePreloads = pose(30, 87.8, 180);
    private final Pose drivetoAlliancePreloadsControl1 = pose(34.4659, 109.8879, 0);
    private final Pose intakeAlliancePreloads = pose(20, 87.8, 180);
    private final Pose backoutofAlliancePreloads = pose(30, 87.8, 180);
    private final Pose shootAlliancePreloads = pose(58, 20, 147.0709);
    private final Pose shootAlliancePreloadsControl1 = pose(26.461, 40.4262, 0);
    private final Pose shootAlliancePreloadsSegment1Start = pose(58, 20, 180);
    private final Pose shootAlliancePreloadsSegment1End = pose(58, 20, 110);
    private final Pose drivetoGarden = pose(20.2256, 6.9, 180);
    private final Pose intakeGarden = pose(7.6, 6.9, 180);
    private final Pose shootGardenStart = pose(7.6, 6.9, 180);
    private final Pose shootGarden = pose(58, 20, 192);
    private final Pose drivetoDoublePark = pose(7.5, 73, 90);
    private final Pose drivetoDoubleParkControl1 = pose(19.7554, 33.2781, 0);
    private final Pose drivetoDoubleParkSegment1Start = pose(7.5, 73, 192);
    private final Pose drivetoDoubleParkSegment1End = pose(7.5, 73, 143);
    private final Pose drivetoDoubleParkSegment3Start = pose(7.5, 73, 114.9256);
    private final Pose drivetoDoubleParkSegment3End = pose(7.5, 73, 90);
    private final Pose doubleParkSLOW = pose(7.5, 92, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, shootPreload()), // flywheel spins up on the way (intake off)
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitMs(100),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),


                // Both flower shots go to the LEFT goal (set now, so the turret / Limelight pipeline switch while driving)
                instant(() -> GoalTargets.leftSide = true),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, driveToAllianceFlower()),


                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                waitMs(100), // Wait for wedge to go down
                race(follow(follower, intakeAllianceFlower()), waitMs(flowerPathTimeout)),
                waitMs(flowerIntakeWait),

                follow(follower, backOutofAllianceFlower()),


                parallel(
                        follow(follower, shootLeft1()),
                        sequential(
                                waitMs(1000),
                                instant(() -> Robot.transportState = Robot.TransportState.HOME)
                        )
                ),
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),

                // Path into the flower, given up after flowerPathTimeout if it doesn't reach the end
                race(follow(follower, intakeLeftFlower()), waitMs(flowerPathTimeout + 1000)),
                waitMs(flowerIntakeWait), // keep intaking at the flower

                parallel(
                        follow(follower, shootLeft2()),
                        sequential(
                                waitMs(500),
                                instant(() -> Robot.flowerWedgeIsActive = false)
                        )
                ),
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),


                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, drivetoAlliancePreloads()),

                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                race(follow(follower, intakeAlliancePreloads()), waitMs(allianceIntakeTimeout)),
                waitMs(allianceIntakeWait),

                follow(follower, backoutofAlliancePreloads()),
                instant(() -> GoalTargets.leftSide = false),

                parallel(
                        follow(follower, shootAlliancePreloads()),
                        sequential(
                                waitMs(1000),
                                instant(() -> Robot.transportState = Robot.TransportState.HOME)
                        )
                ),
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),


                follow(follower, drivetoGarden()),


                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                race(follow(follower, intakeGarden()), waitMs(gardenTimeout)),
                waitMs(gardenWait),


                follow(follower, shootGarden()),
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),


                follow(follower, drivetoDoublePark()),

                follow(follower, doubleParkSLOW())
        );
    }

    @Override
    public void runOpMode() {
        inAuto = true;
        isRed = isRedAlliance();
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        follower = robot.getFollower();
        follower.setPose(start);
        follower.update();
        GoalTargets.redRightY = 62;
        GoalTargets.redLeftY = 90;
        //GoalTargets.redLeftX = 61;
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
        Robot.transportState = Robot.TransportState.HOME;
    }

    public Path shootPreload() {
        return Paths.line(start, shootPreload).constant(shootPreload);
    }

    public Path driveToAllianceFlower() {
        return Paths.curve(driveToAllianceFlowerStart, driveToAllianceFlowerControl1, driveToAllianceFlower).linear(driveToAllianceFlowerStart, driveToAllianceFlower);
    }

    public Path intakeAllianceFlower() {
        return Paths.line(driveToAllianceFlower, intakeAllianceFlower).constant(intakeAllianceFlower)
                .with(Constants.foresightConfig.maxPathSpeed.at(flowerIntakeSpeed)); // slower into the flower
    }

    public Path backOutofAllianceFlower() {
        return Paths.line(intakeAllianceFlower, backOutofAllianceFlower).constant(backOutofAllianceFlower);
    }

    public Path shootLeft1() {
        return Paths.curve(backOutofAllianceFlower, shootLeft1Control1, shootLeft1).heading(Interpolator.piecewise().until(0.3651, Interpolator.linear(shootLeft1Segment1Start, shootLeft1Segment1End)).until(0.9, Interpolator.tangent).until(1, Interpolator.linear(shootLeft1Segment3Start, shootLeft1Segment3End)));
    }

    public Path intakeLeftFlower() {
        return Paths.line(shootLeft1, intakeLeftFlower).constant(intakeLeftFlower)
                .with(Constants.foresightConfig.maxPathSpeed.at(flowerIntakeSpeed)); // slower into the flower
    }

    public Path shootLeft2() {
        return Paths.line(intakeLeftFlower, shootLeft2).heading(Interpolator.piecewise().until(0.5013, Interpolator.constant(shootLeft2Segment1Heading)).until(1, Interpolator.linear(shootLeft2Segment2Start, shootLeft2Segment2End)));
    }

    public Path drivetoAlliancePreloads() {
        return Paths.curve(drivetoAlliancePreloadsStart, drivetoAlliancePreloadsControl1, drivetoAlliancePreloads).linear(drivetoAlliancePreloadsStart, drivetoAlliancePreloads);
    }

    public Path intakeAlliancePreloads() {
        return Paths.line(drivetoAlliancePreloads, intakeAlliancePreloads).constant(intakeAlliancePreloads);
    }

    public Path backoutofAlliancePreloads() {
        return Paths.line(intakeAlliancePreloads, backoutofAlliancePreloads).reverseTangent();
    }

    public Path shootAlliancePreloads() {
        return Paths.curve(backoutofAlliancePreloads, shootAlliancePreloadsControl1, shootAlliancePreloads).heading(Interpolator.piecewise().until(0.4, Interpolator.linear(shootAlliancePreloadsSegment1Start, shootAlliancePreloadsSegment1End)).until(1, Interpolator.tangent.reverse()));
    }

    // Starts where (and at the heading) shootAlliancePreloads ends
    public Path drivetoGarden() {
        return Paths.line(shootAlliancePreloads, drivetoGarden).linear(shootAlliancePreloads, drivetoGarden);
    }

    public Path intakeGarden() {
        return Paths.line(drivetoGarden, intakeGarden).constant(intakeGarden);
    }

    public Path shootGarden() {
        return Paths.line(shootGardenStart, shootGarden).linear(shootGardenStart, shootGarden);
    }

    public Path drivetoDoublePark() {
        return Paths.curve(shootGarden, drivetoDoubleParkControl1, drivetoDoublePark).heading(Interpolator.piecewise().until(0.28, Interpolator.linear(drivetoDoubleParkSegment1Start, drivetoDoubleParkSegment1End)).until(0.8288, Interpolator.tangent).until(1, Interpolator.linear(drivetoDoubleParkSegment3Start, drivetoDoubleParkSegment3End)));
    }

    public Path doubleParkSLOW() {
        return Paths.line(drivetoDoublePark, doubleParkSLOW).constant(doubleParkSLOW)
                .with(Constants.foresightConfig.maxPathSpeed.at(doubleParkSpeed)); // slow final push into the park
    }
}