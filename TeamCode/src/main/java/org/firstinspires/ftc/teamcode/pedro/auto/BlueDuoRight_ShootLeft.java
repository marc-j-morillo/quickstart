
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

@Autonomous(name = "BlueDuoRight_ShootLeft", group = "Autonomous")
public class BlueDuoRight_ShootLeft extends LinearOpMode{
    Robot robot;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startAndShootPreload = poseFactory.of(87.4, 133.5, 90);
    private final Pose intakeAllianceFlowerStart = poseFactory.of(87.4, 133.5, -90);
    private final Pose intakeAllianceFlower = poseFactory.of(129.6765, 94.5, 0);
    private final Pose intakeAllianceFlowerControl1 = poseFactory.of(116.5437, 92.8226, 0);
    private final Pose shoot2 = poseFactory.of(81.5, 121.5, 0);
    private final Pose intakeGarden = poseFactory.of(133.8059, 134.8, 0);
    private final Pose intakeGardenControl1 = poseFactory.of(106.1413, 134.8988, 0);
    private final Pose shootGarden = poseFactory.of(81.5, 121.5, 12);
    private final Pose intakeRightFlower = poseFactory.of(47.0619, 129.7, 90);
    private final Pose intakeRightFlowerControl1 = poseFactory.of(46.7807, 114.8, 0);
    private final Pose shootLeft = poseFactory.of(101.2171, 29.9856, 66.8585);
    private final Pose shootLeftControl1 = poseFactory.of(58.4, 112.7, 0);
    private final Pose shootLeftControl2 = poseFactory.of(138.9, 116.3, 0);
    private final Pose shootLeftSegment1Start = poseFactory.of(101.2171, 29.9856, 90);
    private final Pose shootLeftSegment1End = poseFactory.of(101.2171, 29.9856, 160);
    private final Pose parkStart = poseFactory.of(101.2171, 29.9856, 67);
    private final Pose park = poseFactory.of(126.5, 51.5, 270);
    private final Pose parkControl1 = poseFactory.of(111, 50, 0);

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

                instant(() -> Robot.flowerWedgeIsActive = true),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                follow(follower, intakeRightFlower()),

                parallel(
                        follow(follower, shootLeft()),
                        sequential(
                                waitMs(700),
                                instant(() -> Robot.flowerWedgeIsActive = false)
                        )
                ),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(waitAfterShoot),


                instant(() -> Robot.transportState = Robot.TransportState.HOME),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        inAuto = true;
        isRed = false;

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
        return curve(intakeAllianceFlowerStart, intakeAllianceFlowerControl1, intakeAllianceFlower).linear(intakeAllianceFlowerStart, intakeAllianceFlower);
    }

    public Path shoot2() {
        return line(intakeAllianceFlower, shoot2).linear(intakeAllianceFlower, shoot2);
    }

    public Path intakeGarden() {
        return curve(shoot2, intakeGardenControl1, intakeGarden).linear(shoot2, intakeGarden);
    }

    public Path shootGarden() {
        return line(intakeGarden, shootGarden).linear(intakeGarden, shootGarden);
    }

    public Path intakeRightFlower() {
        return curve(shootGarden, intakeRightFlowerControl1, intakeRightFlower).linear(shootGarden, intakeRightFlower);
    }

    public Path shootLeft() {
        return curve(intakeRightFlower, shootLeftControl1, shootLeftControl2, shootLeft).heading(Interpolator.piecewise().until(0.2448, Interpolator.linear(shootLeftSegment1Start, shootLeftSegment1End)).until(1, Interpolator.tangent.reverse()));
    }

    public Path park() {
        return curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
