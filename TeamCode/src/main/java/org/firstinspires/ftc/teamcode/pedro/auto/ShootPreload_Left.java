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
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.waitBeforeShoot;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerIntakeWait;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerIntakeSpeed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.flowerHeadingEnd;
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

    // Blue copies extend this class and override isRedAlliance(). Every pose below is the red one, mirrored for blue
    // (x -> fieldWidth - x, heading -> 180 - heading, same as GoalTargets), so blue stays an exact copy of red.
    protected boolean isRedAlliance() {
        return true;
    }

    private Pose pose(double x, double y, double headingDeg) {
        if (isRedAlliance()) return poseFactory.of(x, y, headingDeg);
        return poseFactory.of(GoalTargets.fieldWidth - x, y, 180 - headingDeg);
    }

    private final Pose start = pose(57, 7, 90);
    private final Pose shootPreload = pose(58, 20, 135); // preload shot spot, driven to while the flywheel spins up
    private final Pose intakeAllianceFlower = pose(11.8235, 46.9519, 180);
    private final Pose intakeAllianceFlowerControl1 = pose(25.1704, 47.4704, 0);
    private final Pose shootLeft1Start = pose(11.8235, 46.9519, 180);
    private final Pose shootLeft1 = pose(47.1667, 115, 60);
    private final Pose shootLeft1Control1 = pose(32.4282, 44.863, 0);
    private final Pose shootLeft1Control2 = pose(24.7852, 95.1585, 0);
    private final Pose parkStart = pose(47.1667, 115, 60);
    private final Pose park = pose(15, 90, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, moveToShootPreload()), // flywheel spins up on the way (intake off)
                waitMs(200),
                waitMs(waitBeforeShoot),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitMs(1000),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                follow(follower, intakeAllianceFlower()),
                waitMs(flowerIntakeWait), // keep intaking at the flower
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
                waitMs(waitBeforeShoot),
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
        isRed = isRedAlliance();
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

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
