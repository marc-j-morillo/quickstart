package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionBotX;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionBotY;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionHeading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Localization;
import org.firstinspires.ftc.teamcode.teleop.systems.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

import java.util.List;

@TeleOp(name = "testCode", group = "TeleOp")
public class testCode extends OpMode {
    Drive drive;
    Transport transport;

//    Vision vision;

    Localization localization;
    TurretPollen turretPollen;

    public VoltageSensor voltageSensor;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;

    public double voltage;

    public double oldTime = 0;

    public ElapsedTime matchTimer;

    public static double nominalVoltage = 14;


    @Override
    public void init() {
        voltageSensor =  hardwareMap.voltageSensor.iterator().next();
        GlobalVars.inAuto = false;
        GlobalVars.isRed = false;
        drive = new Drive(hardwareMap, true);
        transport = new Transport(hardwareMap);
//        vision = new Vision(hardwareMap);
        localization = new Localization(hardwareMap);
        turretPollen = new TurretPollen(hardwareMap);
        matchTimer = new ElapsedTime();
        matchTimer.reset();

        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.update();
    }

    @Override
    public void start() {
        matchTimer.reset();
    }

    @Override
    public void loop() {
        if (matchTimer.seconds() > 110 && matchTimer.seconds() < 120) {
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);  // 200 mSec burst on left motor.
        } else {
            gamepad1.stopRumble();
        }

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        oldTime = newTime;

        voltage = voltageSensor.getVoltage();
        drive.update(gamepad1, gamepad2, nominalVoltage / voltage);
        transport.update(gamepad1, gamepad2, nominalVoltage / voltage);
        localization.update(gamepad1);
        turretPollen.update();

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
        telemetry.update();
    }
}