package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

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

@Autonomous(name = "RedSolo2Flower", group = "Autonomous")
public class RedSolo2Flower extends LinearOpMode {

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
    private final Pose shootPreload = pose(58, 16, 135); // preload shot spot, driven to while the flywheel spins up
    private final Pose intakeAllianceFlower = pose(15.7, 48, 180);
    private final Pose intakeAllianceFlowerControl1 = pose(32, 49, 0);
    private final Pose shootLeft1Start = pose(11.8235, 46.9519, 180);
    private final Pose shootLeft1 = pose(44.5, 124, 60);
    private final Pose shootLeft1Control1 = pose(32.4282, 44.863, 0);
    private final Pose shootLeft1Control2 = pose(24.7852, 95.1585, 0);
    private final Pose intakeLeftFlower = pose(50, 128.7, 90);
    private final Pose shootLeft2 = pose(44.5, 124, 60);
    private final Pose shootLeft2Segment1Heading = pose(47.1667, 115, 90);
    private final Pose shootLeft2Segment2Start = pose(47.1667, 115, 90);
    private final Pose shootLeft2Segment2End = pose(47.1667, 115, 60);
    private final Pose moveToGarden = pose(27.1281, 24.1733, -89.0918);
    private final Pose moveToGardenControl1 = pose(25.9496, 98.5156, 0);
    private final Pose moveToGardenSegment1Start = pose(27.1281, 24.1733, 60);
    private final Pose moveToGardenSegment1End = pose(27.1281, 24.1733, -100);
    private final Pose intakeGarden = pose(7.694, 6.7067, 180);
    private final Pose intakeGardenControl1 = pose(29.6504, 8, 0);
    private final Pose intakeGardenSegment1Start = pose(7.694, 6.7067, -90);
    private final Pose intakeGardenSegment1End = pose(7.694, 6.7067, 180);
    private final Pose intakeGardenSegment2Heading = pose(7.694, 6.7067, 180);
    private final Pose shootGardenStart = pose(7.694, 6.7067, 180);
    private final Pose shootGarden = pose(60, 20, 192);
    private final Pose park = pose(15, 90, 90);
    private final Pose parkControl1 = pose(11.4339, 56.772, 0);
    private final Pose parkSegment1Start = pose(15, 90, 192);
    private final Pose parkSegment1End = pose(15, 90, 125);
    private final Pose parkSegment3Start = pose(15, 90, 89.7178);
    private final Pose parkSegment3End = pose(15, 90, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, moveToShootPreload()), // flywheel spins up on the way (intake off)
                waitMs(200),
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitMs(2000),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                // Both flower shots go to the LEFT goal (set now, so the turret / Limelight pipeline switch while driving)
                instant(() -> GoalTargets.leftSide = true),
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                waitMs(300),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                // Path into the flower, given up after flowerPathTimeout if it doesn't reach the end
                race(follow(follower, intakeAllianceFlower()), waitMs(flowerPathTimeout)),
                waitMs(flowerIntakeWait), // keep intaking at the flower

                follow(follower, shootLeft1()),

                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),

                // Path into the flower, given up after flowerPathTimeout if it doesn't reach the end
                race(follow(follower, intakeLeftFlower()), waitMs(flowerPathTimeout)),
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

                // Third shot done (preload, flower, flower): park. The garden paths below are unused for now.
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, park())
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

    public Path moveToShootPreload() {
        return line(start, shootPreload).linear(start, shootPreload);
    }

    public Path intakeAllianceFlower() {
        return Paths.curve(shootPreload, intakeAllianceFlowerControl1, intakeAllianceFlower).linear(shootPreload, intakeAllianceFlower, flowerHeadingEnd)
                .with(Constants.foresightConfig.maxPathSpeed.at(flowerIntakeSpeed)); // slower into the flower
    }

    public Path shootLeft1() {
        return Paths.curve(shootLeft1Start, shootLeft1Control1, shootLeft1Control2, shootLeft1).linear(shootLeft1Start, shootLeft1);
    }

    public Path intakeLeftFlower() {
        return Paths.line(shootLeft1, intakeLeftFlower).constant(intakeLeftFlower)
                .with(Constants.foresightConfig.maxPathSpeed.at(flowerIntakeSpeed)); // slower into the flower
    }

    public Path shootLeft2() {
        return Paths.line(intakeLeftFlower, shootLeft2).heading(Interpolator.piecewise().until(0.5013, Interpolator.constant(shootLeft2Segment1Heading)).until(1, Interpolator.linear(shootLeft2Segment2Start, shootLeft2Segment2End)));
    }

    public Path moveToGarden() {
        return Paths.curve(shootLeft2, moveToGardenControl1, moveToGarden).heading(Interpolator.piecewise().until(0.4918, Interpolator.linear(moveToGardenSegment1Start, moveToGardenSegment1End)).until(1, Interpolator.tangent));
    }

    public Path intakeGarden() {
        return Paths.curve(moveToGarden, intakeGardenControl1, intakeGarden).heading(Interpolator.piecewise().until(0.3326, Interpolator.linear(intakeGardenSegment1Start, intakeGardenSegment1End)).until(1, Interpolator.constant(intakeGardenSegment2Heading)));
    }

    public Path shootGarden() {
        return Paths.line(shootGardenStart, shootGarden).linear(shootGardenStart, shootGarden);
    }

    /** Park straight from the second flower shot (shootLeft2). */
    public Path park() {
        return Paths.line(shootLeft2, park).linear(shootLeft2, park);
    }

    /** The old park from the garden shot, unused while the auto parks after the second flower. */
    public Path parkFromGarden() {
        return Paths.curve(shootGarden, parkControl1, park).heading(Interpolator.piecewise().until(0.3593, Interpolator.linear(parkSegment1Start, parkSegment1End)).until(0.9498, Interpolator.tangent).until(1, Interpolator.linear(parkSegment3Start, parkSegment3End)));
    }
}