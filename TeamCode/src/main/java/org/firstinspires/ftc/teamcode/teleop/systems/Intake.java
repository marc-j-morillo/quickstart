
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

public class Intake {
    // ----- Hardware -----

    //public DigitalChannel laser;
    public DcMotorEx intake, transferMotor;
    public Servo transferPollenBlocker;
    //public Servo transferNectarBlocker;
    public Servo flowerWedgeLeft, flowerWedgeRight;

    // ----- Limelight Stuff -------
    //public double limelightMountAngleDegrees = 10;
    
    //public double limelightLensHeightInches = 16.930377953; // distance from the center of the Limelight lens to the floor
    
    //public boolean lowLevel = false; // Color Sensor Values and Logic
    
    // distance from the target to the floor
    public double goalHeightInches = 65.6;

    //MOTOR POWER
    public double intakePower, transferPower;

    public double blockerPosition;

    //ELAPSED TIMES
    public ElapsedTime matchTimer;

    //USEFUL STATES

    //MOTOR POWERS
    public static final double dormant = 0;
    public static final double intaking = 0.9; // TEMPORARY VALUE
    public static final double outtaking = intaking * -1;
    public static final double semiTransferring = intaking * 0.2; // 0.18 power

    //SERVO STATES
    public static final double blockerOpen = .8; // TEMPORARY VALUE
    public static final double blockerClosed = 0; // TEMPORARY VALUE

    //FLOWER WEDGE POSITIONS
    public static final double flowerWedgeDown = 0; // TEMPORARY VALUE
    public static final double flowerWedgeUp = .5; // TEMPORARY VALUE
    public boolean flowerWedgeIsDown;
    public ElapsedTime flowerWedgeWait;

    public static double distanceFromLimelightToTagInches;



    

    public Intake(HardwareMap hardwareMap) {
        

        matchTimer = new ElapsedTime();
        matchTimer.reset();

        flowerWedgeWait = new ElapsedTime();
        flowerWedgeWait.reset();
        flowerWedgeIsDown = false;

        // SET UP HARDWARE
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        //transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transferPollenBlocker = hardwareMap.get(Servo.class, "transferPollenBlocker");
        transferPollenBlocker.setDirection(Servo.Direction.FORWARD);
        //transferNectar = hardwareMap.get(Servo.class, "transferNectar");
        //transferNectar.setDirection(Servo.Direction.FORWARD);

        flowerWedgeLeft = hardwareMap.get(Servo.class, "flowerWedgeLeft");
        //flowerWedgeLeft.setDirection(Servo.Direction.REVERSE);
        flowerWedgeRight = hardwareMap.get(Servo.class, "flowerWedgeRight");
        flowerWedgeRight.setDirection(Servo.Direction.REVERSE);

        //led = hardwareMap.get(ServoImplEx.class, "led");

        if (inAuto) {
            intakePower = dormant;
            transferPollenBlocker.setPosition(blockerClosed);
            //transferNectar.setPosition(blockerClosed);
            flowerWedgeLeft.setPosition(flowerWedgeUp);
            flowerWedgeRight.setPosition(flowerWedgeUp);
            //led.setPosition(0);
        } else {
            intakePower = dormant;
            transferPollenBlocker.setPosition(blockerClosed);
            //transferNectar.setPosition(blockerClosed);
            flowerWedgeLeft.setPosition(flowerWedgeUp);
            flowerWedgeRight.setPosition(flowerWedgeUp);
            //led.setPosition(.6);
        }
    }

    public void update(boolean flowerWedgeIsActive) {
        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        transferPollenBlocker.setPosition(blockerPosition);
        //transferNectar.setPosition(blockerPosition);
        flowerWedgeLeft.setPosition(flowerWedgeIsActive ? flowerWedgeDown : flowerWedgeUp);
        flowerWedgeRight.setPosition(flowerWedgeIsActive ? flowerWedgeDown : flowerWedgeUp);


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
    }


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

        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        transferPollenBlocker.setPosition(blockerPosition);
        flowerWedgeLeft.setPosition(setFlowerWedge(gamepad1));
        flowerWedgeRight.setPosition(setFlowerWedge(gamepad1));
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
    }

    public double setFlowerWedge(Gamepad gamepad1) {
        // Raise and lower the flower wedge with the left bumper, 
        // but only if 0.5 seconds have passed since the last press to prevent rapid toggling
        if (gamepad1.left_bumper && flowerWedgeWait.seconds() > 0.5) {
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