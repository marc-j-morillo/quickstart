/*
package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;

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
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

@Autonomous(name = "RedDuoRight_ShootLeft", group = "Autonomous")
public class RedDuoRight_ShootLeft extends LinearOpMode{
    Robot robot;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(35.0422, 8.4494, 90);
    private final Pose intakeAllianceFlower = poseFactory.of(11.8235, 46.9519, 180);
    private final Pose intakeAllianceFlowerControl1 = poseFactory.of(25.1704, 47.4704, 0);
    private final Pose shoot2 = poseFactory.of(60, 20, 192);
    private final Pose moveToGarden = poseFactory.of(20, 6.7067, 180);
    private final Pose intakeGarden = poseFactory.of(7.694, 6.7067, 180);
    private final Pose shootGarden = poseFactory.of(60, 20, 192);
    private final Pose intakeRightFlower = poseFactory.of(94.4381, 11.8235, 270);
    private final Pose intakeRightFlowerControl1 = poseFactory.of(94.7193, 26.6519, 0);
    private final Pose shootLeft = poseFactory.of(36.3007, 105.7997, -112.2112);
    private final Pose shootLeftControl1 = poseFactory.of(83.1078, 28.8365, 0);
    private final Pose shootLeftControl2 = poseFactory.of(2.6151, 25.245, 0);
    private final Pose shootLeftSegment1Start = poseFactory.of(36.3007, 105.7997, -90);
    private final Pose shootLeftSegment1End = poseFactory.of(36.3007, 105.7997, -30);
    private final Pose parkStart = poseFactory.of(36.3007, 105.7997, 250);
    private final Pose park = poseFactory.of(15, 90, 90);
    private final Pose parkControl1 = poseFactory.of(31.7995, 92.0676, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(300),
                instant(() -> Robot.transportState = Robot.TransportState.INTAKE),
                instant(() -> Robot.flowerWedgeIsActive = true),
                follow(follower, intakeAllianceFlower()),


                follow(follower, shoot2()),

                waitMs(8000),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                instant(() -> Robot.flowerWedgeIsActive = false),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(300),


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








                follow(follower, moveToGarden()),

                parallel(
                        follow(follower, shootLeft()),
                        sequential(
                                waitMs(700),
                                instant(() -> Robot.flowerWedgeIsActive = false)
                        )
                ),
                instant(() -> Robot.transportState = Robot.TransportState.POWER_SHOOTER),
                waitUntil(() -> Robot.transportState == Robot.TransportState.SHOOT),
                waitMs(300),


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
        GlobalVars.startPose = follower.pose();
    }

    public Path intakeAllianceFlower() {
        return curve(start, intakeAllianceFlowerControl1, intakeAllianceFlower).linear(start, intakeAllianceFlower);
    }

    public Path shoot2() {
        return line(intakeAllianceFlower, shoot2).linear(intakeAllianceFlower, shoot2);
    }

    public Path moveToGarden() {
        return line(shoot2, moveToGarden).linear(shoot2, moveToGarden);
    }

    public Path intakeGarden() {
        return line(moveToGarden, intakeGarden).constant(intakeGarden);
    }

    public Path shootGarden() {
        return line(intakeGarden, shootGarden).linear(intakeGarden, shootGarden);
    }

    public Path intakeRightFlower() {
        return curve(shootGarden, intakeRightFlowerControl1, intakeRightFlower).linear(shootGarden, intakeRightFlower);
    }

    public Path shootLeft() {
        return curve(intakeRightFlower, shootLeftControl1, shootLeftControl2, shootLeft).heading(Interpolator.piecewise().until(0.2061, Interpolator.linear(shootLeftSegment1Start, shootLeftSegment1End)).until(1, Interpolator.tangent.reverse()));
    }

    public Path park() {
        return curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
*/