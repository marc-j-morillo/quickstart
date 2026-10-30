package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose.startPose;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.BulkReads;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.FollowerPose;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import java.util.List;

@Configurable
public class Robot {
    Shooter shooter;
    Drive drive;
    Intake intake;

    public VoltageSensor voltageSensor;
    public double voltage;
    public static double nominalVoltage = 14.0;
    public double voltageMultiplier;
    // The voltage sensor isn't in the bulk read (each read is its own ~1-2 ms command) and battery voltage
    // changes slowly, so only read it this often
    public static double voltageReadIntervalMs = 250;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    public static double triggerThreshold = 0.2;

    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;
    public LynxModule ExpHub;

    @IgnoreConfigurable public static boolean flowerWedgeIsActive = false; // Auto only

    //FSMs
    public static enum TransportState {
        HOME,
        INTAKE,
        OUTTAKE,
        POWER_SHOOTER,
        SHOOT
    }
    @IgnoreConfigurable public static TransportState transportState = TransportState.HOME;
    private boolean shotCancelled = false; // set by a, cleared when right trigger is released


    public Robot(HardwareMap hardwareMap, boolean fieldCentric) {
        voltageSensor =  hardwareMap.voltageSensor.iterator().next();
        shooter = new Shooter(hardwareMap);
        drive = new Drive(hardwareMap, fieldCentric);
        intake = new Intake(hardwareMap);

        // MANUAL bulk reads: one read per hub per loop, cleared at the start of each update()
        allHubs = BulkReads.enableManual(hardwareMap);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);

        readVoltage();
    }

    public Robot(HardwareMap hardwareMap) {
        voltageSensor =  hardwareMap.voltageSensor.iterator().next();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // MANUAL bulk reads: one read per hub per loop, cleared at the start of each update()
        allHubs = BulkReads.enableManual(hardwareMap);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);

        readVoltage();
    }

    private void readVoltage() {
        voltage = voltageSensor.getVoltage();
        voltageMultiplier = nominalVoltage / voltage;
        voltageTimer.reset();
    }

    private void updateVoltage() {
        if (voltageTimer.milliseconds() >= voltageReadIntervalMs) readVoltage();
    }



    // TELEOP UPDATE LOOP
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        BulkReads.clear(allHubs); // before any hardware read this loop
        setTransportState(gamepad1);
        updateVoltage();

        drive.update(gamepad1);
        shooter.update(voltageMultiplier, Drive.isDriving);
        intake.update(gamepad1, gamepad2);
    }

    // AUTO UPDATE LOOP
    public void update(Pose currentPose) {
        BulkReads.clear(allHubs); // before any hardware read this loop
        if (transportState == TransportState.POWER_SHOOTER) {
            if (shooter.inRange()) {
                transportState = TransportState.SHOOT;
            }
        }

        updateVoltage();

        FollowerPose.update(currentPose);
        shooter.update(voltageMultiplier, false);
        intake.update(flowerWedgeIsActive);

        // Automatically change from POWER_SHOOTER to SHOOT when the shooter is at the target velocity

    }

    /** Current inputs for the controller (all are hold-to-run, nothing held = HOME):
     *      right trigger - "power up" shooter and shoot (POWER_SHOOTER, then SHOOT once in range)
     *          when powering/shooting, cannot drive
     *          a - cancel the shot (HOME) until right trigger is released
     *
     *      left trigger - intake (INTAKE)
     *
     *      b - outtake (OUTTAKE)
     */

    private void setTransportState(Gamepad gamepad1) {
        // *WasPressed() clears itself when read, so read it exactly once per loop
        boolean cancelPressed = gamepad1.aWasPressed();
        boolean shootHeld = gamepad1.right_trigger > triggerThreshold;
        boolean intakeHeld = gamepad1.left_trigger > triggerThreshold;

        if (!shootHeld) {
            shotCancelled = false;
        } else if (cancelPressed) {
            shotCancelled = true;
        }

        if (shootHeld && !shotCancelled) {
            // Automatically change from POWER_SHOOTER to SHOOT when the shooter is at the target velocity,
            // then stay in SHOOT until the trigger is released
            if (transportState == TransportState.POWER_SHOOTER && shooter.inRange()) {
                transportState = TransportState.SHOOT;
            } else if (transportState != TransportState.SHOOT) {
                transportState = TransportState.POWER_SHOOTER;
            }
        } else if (intakeHeld) {
            transportState = TransportState.INTAKE;
        } else if (gamepad1.b) {
            transportState = TransportState.OUTTAKE;
        } else {
            transportState = TransportState.HOME;
        }
    }
}