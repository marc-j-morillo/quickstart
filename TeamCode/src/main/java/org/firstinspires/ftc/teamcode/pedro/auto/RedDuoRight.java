
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

@Autonomous(name = "RedDuoRight", group = "Autonomous")
public class RedDuoRight extends LinearOpMode{
    Robot robot;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(35.0422, 8.4494, 90);
    private final Pose intakeAllianceFlower = poseFactory.of(11.8235, 46.9519, 180);
    private final Pose intakeAllianceFlowerControl1 = poseFactory.of(25.1704, 47.4704, 0);
    private final Pose shoot2 = poseFactory.of(60, 20, 180);
    private final Pose intakeGarden = poseFactory.of(7.694, 6.7067, 180);
    private final Pose intakeGardenControl1 = poseFactory.of(42.1188, 6.843, 0);
    private final Pose shootGarden = poseFactory.of(60, 20, 145);
    private final Pose park = poseFactory.of(15, 90, 90);
    private final Pose parkControl1 = poseFactory.of(11.0068, 52.7803, 0);
    private final Pose parkSegment2Start = poseFactory.of(15, 90, 88.147);
    private final Pose parkSegment2End = poseFactory.of(15, 90, 90);



    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                follow(follower, intakeAllianceFlower()),


                follow(follower, shoot2()),
                instant(() -> Robot.flowerWedgeIsActive = false),

                waitMs(8000), // TEMPORARY VALUE
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),

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
        follower.setPose(start);
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
        return curve(start, intakeAllianceFlowerControl1, intakeAllianceFlower).linear(start, intakeAllianceFlower);
    }

    public Path shoot2() {
        return line(intakeAllianceFlower, shoot2).linear(intakeAllianceFlower, shoot2);
    }

    public Path intakeGarden() {
        return curve(shoot2, intakeGardenControl1, intakeGarden).constant(intakeGarden);
    }

    public Path shootGarden() {
        return line(intakeGarden, shootGarden).linear(intakeGarden, shootGarden);
    }

    public Path park() {
        return curve(shootGarden, parkControl1, park).heading(Interpolator.piecewise().until(0.951, Interpolator.tangent).until(1, Interpolator.linear(parkSegment2Start, parkSegment2End)));
    }
}
