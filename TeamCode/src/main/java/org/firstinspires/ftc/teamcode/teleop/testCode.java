package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.startPose;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.LoopTimer;

import java.util.List;

@TeleOp(name = "testCode", group = "TeleOp")
public class testCode extends OpMode {
    Robot robot;
    private ElapsedTime matchTimer = new ElapsedTime();
    private final LoopTimer loopTimer = new LoopTimer();
    private boolean rumbling = false;

    @Override
    public void init() {
        GlobalVars.inAuto = false;
        GlobalVars.isRed = true;
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        matchTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.update();

        // Rumble during the last 10 s of teleop. Only send on a change: every rumble call is a message to the DS
        boolean shouldRumble = matchTimer.seconds() > 110 && matchTimer.seconds() < 120;
        if (shouldRumble && !rumbling) gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        if (!shouldRumble && rumbling) gamepad1.stopRumble();
        rumbling = shouldRumble;

        robot.update(gamepad1, gamepad2);
        loopTimer.addTelemetry(telemetry);
        /*

        telemetry.addData("Transition Heading", transitionHeading);
        telemetry.addData("Distance To Goal", Transport.distanceFromLimelightToTagInches);
        telemetry.addData("TransitionX", transitionBotX);
        telemetry.addData("TransitionY", transitionBotY);
        telemetry.addData("PinpointToGoal Toggle", drive.pinpointToGoal.value());
        telemetry.addData("Match Timer", matchTimer.seconds());
        telemetry.addData("REV Hub Loop Time in Ms: ", loopTime * 1000); //prints the control system refresh rate
        telemetry.addData("IsRed", GlobalVars.isRed);
        telemetry.addData("rx", Drive.rx);
        telemetry.addData("BotHeading", Math.toDegrees(Localization.botHeading));
        telemetry.addData("BotX", Localization.botX);
        telemetry.addData("BotY", Localization.botY);
//        telemetry.addData("VisionBotX", Vision.visionBotX);
//        telemetry.addData("VisionBotY", Vision.visionBotY);
//        telemetry.addData("tx", Vision.tX);
//        telemetry.addData("ty", Vision.tY);
//        telemetry.addData("Valid Result", Vision.validResult);
//        telemetry.addData("Vision Connected", Vision.isConnected);
//        telemetry.addData("Vision Running", Vision.isRunning);
//        telemetry.addData("IsNull", Vision.result == null);
//        telemetry.addData("IsValid", Vision.result.isValid());
        telemetry.addData("ShooterP", transport.shooterp);
        telemetry.addData("Flywheel Velocity", Transport.shooterVelocity);
        telemetry.addData("Flywheel Target Velocity", Transport.shooterVelocityTarget);
        telemetry.addData("Flywheel Fire Tolerance", Transport.fireTolerance);
        telemetry.addData("Flywheel Ready", transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget));
        telemetry.addData("Flywheel Linear Velocity M/S", (Transport.shooterVelocity / 28 * 2 * Math.PI * .048)); //Shooter Velocity in TPS, Divide by CPR, Multiply by 2PI, and then Multiply by Radius of Flywheel
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Voltage Multiplier", voltage);
        telemetry.addData("Intake Power", Transport.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer Power", Transport.transfer.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FrontLeft Power", Drive.frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FrontRight Power", Drive.frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BackLeft Power", Drive.backLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BackRight Power", Drive.backRight.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Low Level Color Value", Transport.lowLevel);
//        telemetry.addData("Transition Heading", Math.toDegrees(transitionHeading));
*/
        telemetry.update();
    }
}