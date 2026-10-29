package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars.isRed;

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
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Intake {
    // -------------- Hardware --------------

    //public DigitalChannel laser;
    public DcMotorEx intake, transferMotor;
    public Servo transferPollenBlocker;
    //public Servo transferNectarBlocker;
    public Servo flowerWedgeLeft, flowerWedgeRight;

    // -------------- Limelight Stuff -------------- (commented out, we don't use a limelight)

    //public double limelightMountAngleDegrees = 10;

    //public double limelightLensHeightInches = 16.930377953; // distance from the center of the Limelight lens to the floor

    //public boolean lowLevel = false; // Color Sensor Values and Logic

    // distance from the target to the floor
    //public double goalHeightInches = 65.6;


    // -------------- Power Values --------------

    //MOTOR POWER
    public double intakePower, transferPower;

    public double blockerPosition;

    //ELAPSED TIMES
    public ElapsedTime matchTimer;

    //USEFUL STATES

    //MOTOR POWERS
    public static final double dormant = 0;
    public static double intaking = 0.9; // TEMPORARY VALUE
    public static double outtaking = -0.9;
    public static double semiTransferring = 0.18;

    //SERVO STATES
    public static double blockerOpen = 0.6;
    public static double blockerClosed = 0.85;

    //FLOWER WEDGE POSITIONS
    public static double flowerWedgeDown = 0; // TEMPORARY VALUE
    public static double flowerWedgeUp = .5; // TEMPORARY VALUE
    public static double flowerWedgeDebounceSeconds = 0.5;
    public boolean flowerWedgeIsDown;

    // Flower wedge is disabled for now. Set to true to bring it back (needs the
    // flowerWedgeLeft / flowerWedgeRight servos in the robot config).
    public static final boolean flowerWedgeEnabled = false;
    public ElapsedTime flowerWedgeWait;

    //MOTOR DIRECTIONS (matching IntakeTest, known working). Applied at init.
    public static boolean intakeReversed = false;
    public static boolean transferReversed = true;





    public Intake(HardwareMap hardwareMap) {


        matchTimer = new ElapsedTime();
        matchTimer.reset();

        flowerWedgeWait = new ElapsedTime();
        flowerWedgeWait.reset();
        flowerWedgeIsDown = false;

        // SET UP HARDWARE
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake.setDirection(intakeReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(transferReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transferPollenBlocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        transferPollenBlocker.setDirection(Servo.Direction.FORWARD);
        //transferNectar = hardwareMap.get(Servo.class, "transferNectar");
        //transferNectar.setDirection(Servo.Direction.FORWARD);

        if (flowerWedgeEnabled) {
            flowerWedgeLeft = hardwareMap.get(Servo.class, "flowerWedgeLeft");
            //flowerWedgeLeft.setDirection(Servo.Direction.REVERSE);
            flowerWedgeRight = hardwareMap.get(Servo.class, "flowerWedgeRight");
            flowerWedgeRight.setDirection(Servo.Direction.REVERSE);
        }

        //led = hardwareMap.get(ServoImplEx.class, "led");

        if (inAuto) {
            intakePower = dormant;
            transferPollenBlocker.setPosition(blockerClosed);
            //transferNectar.setPosition(blockerClosed);
            if (flowerWedgeEnabled) {
                flowerWedgeLeft.setPosition(flowerWedgeUp);
                flowerWedgeRight.setPosition(flowerWedgeUp);
            }
            //led.setPosition(0);
        } else {
            intakePower = dormant;
            transferPollenBlocker.setPosition(blockerClosed);
            //transferNectar.setPosition(blockerClosed);
            if (flowerWedgeEnabled) {
                flowerWedgeLeft.setPosition(flowerWedgeUp);
                flowerWedgeRight.setPosition(flowerWedgeUp);
            }
            //led.setPosition(.6);
        }
    }


    // -------------- AUTONOMOUS UPDATE METHOD --------------
    public void update(boolean flowerWedgeIsActive) {
        if (flowerWedgeEnabled) {
            flowerWedgeLeft.setPosition(flowerWedgeIsActive ? flowerWedgeDown : flowerWedgeUp);
            flowerWedgeRight.setPosition(flowerWedgeIsActive ? flowerWedgeDown : flowerWedgeUp);
        }


        switch (Robot.transportState) {
            case HOME:
                intakePower = dormant;
                transferPower = dormant;
                blockerPosition = blockerClosed;
                break;
            case INTAKE:
                intakePower = intaking;
                transferPower = semiTransferring;
                blockerPosition = blockerClosed;
                break;
            case OUTTAKE:
                intakePower = outtaking;
                transferPower = outtaking;
                blockerPosition = blockerClosed;
                break;
            case POWER_SHOOTER:
                intakePower = intaking;
                transferPower = semiTransferring;
                blockerPosition = blockerClosed;
                break;
            case SHOOT:
                intakePower = intaking;
                transferPower = intaking;
                blockerPosition = blockerOpen;
                break;
        }

        // Write after the switch so this loop's state is applied this loop
        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        transferPollenBlocker.setPosition(blockerPosition);
        //transferNectar.setPosition(blockerPosition);
    }

    // -------------- TELEOP UPDATE METHOD --------------
    public void update(Gamepad gamepad1, Gamepad gamepad2) {

//        if (Vision.tX < 2 && Vision.tX > -2 && Vision.validResult) {
//            led.setPosition(.5);
//        } else if (!isRed && Vision.tX < 6 && Vision.tX > 2.5 && Vision.validResult) {
//            led.setPosition(.63);
//        } else if (isRed && Vision.tX > -6 && Vision.tX < -2.5 && Vision.validResult) {
//            led.setPosition(.63);
        // if (!Vision.isConnected) {
        //     led.setPosition(0);
        // } else {
        //     led.setPosition(.3);
        // }

        if (flowerWedgeEnabled) {
            // call setFlowerWedge once: it consumes the bumper press, so a second call would miss it
            double wedgePosition = setFlowerWedge(gamepad1);
            flowerWedgeLeft.setPosition(wedgePosition);
            flowerWedgeRight.setPosition(wedgePosition);
        }
        //transferNectar.setPosition(blockerPosition);


        switch (Robot.transportState) {
            case HOME:
                intakePower = dormant;
                transferPower = dormant;
                blockerPosition = blockerClosed;
                break;
            case OUTTAKE:
                intakePower = outtaking;
                transferPower = outtaking;
                blockerPosition = blockerClosed;
                break;
            case INTAKE:
                intakePower = intaking;
                transferPower = semiTransferring;
                blockerPosition = blockerClosed;
                break;
            case POWER_SHOOTER:
                intakePower = semiTransferring;
                transferPower = semiTransferring;
                blockerPosition = blockerClosed;
                break;
            case SHOOT:
                intakePower = intaking;
                transferPower = intaking;
                blockerPosition = blockerOpen;
                break;
        }

        // Write after the switch so this loop's state is applied this loop
        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        transferPollenBlocker.setPosition(blockerPosition);
    }

    public double setFlowerWedge(Gamepad gamepad1) {
        // Raise and lower the flower wedge with the left bumper,
        // but only if flowerWedgeDebounceSeconds have passed since the last press to prevent rapid toggling
        if (gamepad1.leftBumperWasPressed() && flowerWedgeWait.seconds() > flowerWedgeDebounceSeconds) {
            flowerWedgeWait.reset();
            if (flowerWedgeIsDown) {
                flowerWedgeIsDown = false;
                return flowerWedgeUp;
            } else {
                flowerWedgeIsDown = true;
                return flowerWedgeDown;
            }
        }
        else {
            if (flowerWedgeIsDown) {
                return flowerWedgeDown;
            } else {
                return flowerWedgeUp;
            }
        }
    }
}