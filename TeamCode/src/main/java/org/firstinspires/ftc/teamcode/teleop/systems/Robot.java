package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;

public class Robot {
    Shooter shooter;
    Drive drive;
    Intake intake;
    Localization localization;

    public VoltageSensor voltageSensor;
    public voltage;
    public static final double nominalVoltage = 14.0;
    public voltageMultiplier;

    public static final double triggerThreshold = 0.2;

    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;
    public LynxModule ExpHub;

    public static boolean flowerWedgeIsActive = false; // Auto only

    //FSMs
    public static enum TransportState {
        HOME,
        INTAKE,
        OUTTAKE,
        POWER_SHOOTER,
        SHOOT
    }
    public static TransportState transportState = TransportState.HOME;
    public static TransportState returnCase = TransportState.HOME;


    public Robot(HardwareMap hardwareMap) {
        voltageSensor =  hardwareMap.voltageSensor.iterator().next();
        shooter = new Shooter(hardwareMap);
        drive = new Drive(hardwareMap, true);
        intake = new Intake(hardwareMap);
        localization = new Localication(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    public void init(HardwareMap hardwareMap, Pose startPose) {
        localization.createFollower(hardwareMap, startPose);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        voltage = voltageSensor.getVoltage();
        voltageMultiplier = nominalVoltage / voltage;

        drive.update(gamepad1);
        shooter.update(voltageMultiplier, Drive.isDriving);
        intake.update(gamepad1, gamepad2);
        localization.update(gamepad1);

        setTransportState();
    }
    public void update() {
        voltage = voltageSensor.getVoltage();
        voltageMultiplier = nominalVoltage / voltage;

        shooter.update(voltageMultiplier, false);
        intake.update(flowerWedgeIsActive);
        localization.update();
        // Automatically change from POWER_SHOOTER to SHOOT when the shooter is at the target velocity
        if (transportState == TransportState.POWER_SHOOTER) {
            if (shooter.inRange()) {
                transportState = TransportState.SHOOT;
            }
        }
    }

    /** Current inputs for the controller:
     *      left trigger - intake (INTAKE)
     * 
     *      right trigger - "power up" shooter and shoot (POWER_SHOOTER and SHOOT)
     *          when powering/shooting, cannot drive
     *          a - reset to nothing (if shooting), can shoot (HOME)
     * 
     *      b - outtake, then change back when released (OUTTAKE, store prev. in returnCase)
     */

    private void setTransportState() {
        // Automatically change from POWER_SHOOTER to SHOOT when the shooter is at the target velocity
        if (transportState == TransportState.POWER_SHOOTER && gamepad.right_trigger > triggerThreshold) {
            if (shooter.inRange()) {
                transportState = TransportState.SHOOT;
            }
        } else if (gamepad1.a && transportState != TransportState.HOME) {
            transportState = TransportState.HOME;
        } else if (gamepad1.right_trigger > triggerThreshold && transportState != TransportState.POWER_SHOOTER) {
            transportState = TransportState.POWER_SHOOTER;
        } else if (gamepad1.left_trigger > triggerThreshold && transportState != TransportState.INTAKE) {
            transportState = TransportState.INTAKE;
        } else if (gamepad1.b && transportState != TransportState.OUTTAKE) {
            returnCase = transportState;
            transportState = TransportState.OUTTAKE;
        } else if (!gamepad.b && transportState == TransportState.OUTTAKE) {
            transportState = returnCase;
        } else {
            transportState = TransportState.HOME;
        }
        
        if (transportState == TransportState.SHOOT || transportState == TransportState.POWER_SHOOTER) {
            if (gamepad1.a) {
                transportState = TransportState.HOME;
            }
        }
    }
}