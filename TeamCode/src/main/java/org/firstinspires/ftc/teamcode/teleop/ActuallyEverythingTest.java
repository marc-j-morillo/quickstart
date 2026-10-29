package org.firstinspires.ftc.teamcode.teleop;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.HoodPollen;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.TurretPollen;

@TeleOp(name = "ActuallyEverythingTest", group = "TeleOp")
@Configurable
public class ActuallyEverythingTest extends OpMode {
    //  ------------- BLOCKER -------------

    public Servo blocker;

    //SERVO STATES
    public static final double blockerOpen = 0.6;
    public static final double blockerClosed = 0.85;
    public double blockerPosition = blockerOpen;

    //  ------------- INTAKE -------------

    public DcMotorEx transferMotor;
    public DcMotorEx intakeMotor;
    public double intakePower = dormant;
    public double transferPower = dormant;

    public static double intakeStallDraw = 5000;
    public boolean transferFull = false;
    public double loopCount = 0;


    //MOTOR POWERS
    public static double dormant = 0;
    public static double intaking = 1;
    public double outtaking = intaking * -1;
    public static double transferMult = 0.7;
    public double semiTransferring = intaking * transferMult;


    // ------------- FLYWHEEL -------------
    Flywheel flywheel;

    public static double flywheelTarget = 0;
    public double flywheelActual = 0;

    public boolean isAtTarget = false;

    public double power = 0;



    // ------------- HOOD -------------
    HoodPollen hood;

    public static double hoodPosition = 0;

    public static final double hoodMin = 0;

    public static final double hoodMax = 0.81;


    // ------------- TURRET -------------

    TurretPollen turret;
    public static double turretPosition = .5;


    // ------------- PIDF CONTROLLER -------------

    public static  PIDFController shooterController;

    public static double shooterp = 0.007, shooteri = 0, shooterd = 0, kV = .000400186335403727, kS = .001;
    public static double shooterpid;



    // ------------- DRIVE  -------------
    Drive drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        blocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        blocker.setDirection(Servo.Direction.FORWARD);


        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheel = new Flywheel(hardwareMap);

        hood = new HoodPollen(hardwareMap);

        turret = new TurretPollen(hardwareMap);

        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);

        drive = new Drive(hardwareMap, true);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        loopCount++;
        loopCount %= 5;
        if (gamepad1.aWasPressed()) {
            if (blockerPosition == blockerOpen) {
                blockerPosition = blockerClosed;
            } else {
                blockerPosition = blockerOpen;
            }
        }
        blocker.setPosition(blockerPosition);


        outtaking = intaking * -1;
        semiTransferring = intaking * transferMult;


        if (gamepad1.right_trigger > 0.1) {
            intakePower = intaking;
            if (!transferFull){
                if (loopCount == 1) {
                    if (transferMotor.getCurrent(CurrentUnit.MILLIAMPS) > intakeStallDraw) {
                        transferPower = 0;
                        transferFull = true;
                    } else {
                        transferPower = semiTransferring;
                    }
                } else {
                    transferPower = semiTransferring;
                }
            } else {
                transferPower = 0;
            }
        } else if (gamepad1.left_trigger > 0.1) {
            transferPower = outtaking;
            intakePower = outtaking;
            transferFull = false;
        } else {
            intakePower = 0;
            transferPower = 0;
            transferFull = false;
        }

        transferMotor.setPower(transferPower);
        intakeMotor.setPower(intakePower);


        flywheelActual = flywheel.getVelocity();

        isAtTarget = flywheelActual >= flywheelTarget;

        if (!isAtTarget) {
            power = 1;
        } else {
            power = 0;
        }

        flywheelActual = flywheel.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV);
        shooterpid = shooterController.calculate(flywheelActual, flywheelTarget) + (Math.signum(flywheelTarget) * kS);
        flywheel.update(Range.clip(shooterpid, -1.0, 1.0));


        hood.update(hoodPosition);



        turret.update(turretPosition);

        drive.update(gamepad1);



        telemetry.addLine("VALUES:");
        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Transfer power: ", transferPower);
        telemetry.addData("Transfer current: ", transferMotor.getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.update();
    }
}
