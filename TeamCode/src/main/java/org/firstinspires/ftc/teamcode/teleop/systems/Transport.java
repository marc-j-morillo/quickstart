// TODO: 
// Change case function to include all possible cases and changes

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

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class Transport {
    public ElapsedTime shootWait;

    // ----- Hardware -----

    //public DigitalChannel laser;
    public Toggle intakeToggle, redToggle, manualMode;
    public static DcMotorEx intake, transferMotor;
    public static MotorEx leftShooter, rightShooter;
    public static MotorGroup shooter;
    public static Servo transferPollenBlocker, hoodPollen;
    //public static Servo transferNectar, hoodNectar;
    public static Servo flowerWedgeLeft, flowerWedgeRight;

    // ----- Limelight Stuff -------
    //public double limelightMountAngleDegrees = 10;
    
    //public double limelightLensHeightInches = 16.930377953; // distance from the center of the Limelight lens to the floor
    
    //public boolean lowLevel = false; // Color Sensor Values and Logic
    
    // distance from the target to the floor
    public double goalHeightInches = 65.6;

    //Shooter PID
    private PIDFController shooterController;

    public double shooterp = 0.007, shooteri = 0, shooterd = 0, kV = .000400186335403727, kS = .001;
    public double shooterpid;
    //MOTOR POWER
    public static double intakePower, transferPower;

    //MOTOR VELOCITY
    public static double shooterVelocity;

    //MOTOR TARGETS
    public static double shooterVelocityTarget;

    //SERVO STATES
    public static double transferPosition, hoodPollenPosition, hoodNectarPosition;

    //ELAPSED TIMES
    public static ElapsedTime matchTimer;

    //USEFUL STATES

    //MOTOR STATES
    public final double dormant = 0;
    public final double intaking = 1;
    public final double outtaking = -1;
    public final double semiTransferring = intaking * 0.2;
    //SERVO STATES
    public final double transferOpen = .8;
    public final double transferClosed = 0;

    //SHOOTER VELOCITY TARGETS
    public final double shootingLong = 800;
    public final double shootingMed = 600;
    public final double shootingShort = 510;
    public final double emergencyEject = 0;

    //HOOD POSITIONS
    public final double hoodShort = 0;
    public final double hoodMed = .15;
    public final double hoodLong = .18;

    //FLOWER WEDGE POSITIONS
    public final double flowerWedgeDown = 0; // TEMPORARY VALUE
    public final double flowerWedgeUp = .5; // TEMPORARY VALUE
    public boolean flowerWedgeIsDown = true;
    public ElapsedTime flowerWedgeWait;

    //Init the Look up table
    public InterpLUT lut = new InterpLUT();

    public static double distanceFromLimelightToTagInches;

    //FSMs:
    public enum TransportState {
        HOME,
        INTAKE,
        OUTTAKE,

        POWER_SHOOTER,

        POWER_SHOOTER_SHORT,

        POWER_SHOOTER_MED,

        POWER_SHOOTER_LONG,

        AUTO_POWER,
        SHOOT

    }

    public TransportState transportState = TransportState.HOME;
    public TransportState returnCase = TransportState.HOME;

    //Fire Ready?
    public static double fireTolerance = 40;
    public boolean inRange(double currentVelocity, double targetVelocity) {
        if (Math.abs(targetVelocity - currentVelocity) < fireTolerance) {
            return true;
        } else {
            return false;
        }
    }

    public double calcVelocity() {
        double targetGoalX;
        double targetGoalY;

        // Determine target goal coordinates
        if (isRed) targetGoalX = 58.0;
        else targetGoalX = 84.0;

        if (Localization.botY < 51.0) targetGoalY = 61.0;
        else if (Localization.botY > 93.0) targetGoalY = 83.0;
        else targetGoalY = Localization.botY;


        // Calculate Distance Vector
        double deltaX = targetGoalX - Localization.botX;
        double deltaY = targetGoalY - Localization.botY;
        distanceFromLimelightToTagInches = Math.hypot(deltaX, deltaY);

        // Lookup Velocity & Round to Nearest 10 RPM
        double rawVelocity = lut.get(distanceFromLimelightToTagInches);
        double velocityTarget = Math.round(rawVelocity / 10.0) * 10.0;

        return velocityTarget;
    }

    public static void setTransportState() {
        if (gamepad1.dpad_down && transportState != TransportState.HOME) {
            transportState = TransportState.HOME;
        }
        if (gamepad1.left_trigger > 0) {
            transportState = TransportState.INTAKE;
        }
        if (gamepad1.x) {
            transportState = TransportState.POWER_SHOOTER_SHORT;
        }
        if (gamepad1.right_trigger > 0) {
            transportState = TransportState.AUTO_POWER;
        }
        if (gamepad1.y) {
            transportState = TransportState.POWER_SHOOTER_MED;
        }
        if (gamepad1.b) {
            returnCase = transportState;
            transportState = TransportState.OUTTAKE;
        }
        if (gamepad1.dpad_up) {
            transportState = TransportState.POWER_SHOOTER_LONG;
        }
        // Automatically change from POWER_SHOOTER to SHOOT when the shooter is at the target velocity
        if (transportState == TransportState.POWER_SHOOTER || 
            transportState == TransportState.POWER_SHOOTER_SHORT || 
            transportState == TransportState.POWER_SHOOTER_MED || 
            transportState == TransportState.POWER_SHOOTER_LONG || 
            transportState == TransportState.AUTO_POWER) {
            if (inRange(shooterVelocity, shooterVelocityTarget)){
                shootWait.reset();
                transportState = TransportState.SHOOT;
            }
        }
        // Raise and lower the flower wedge with the left bumper, 
        // but only if 0.5 seconds have passed since the last press to prevent rapid toggling
        if (gamepad1.left_bumper && flowerWedgeWait.seconds() > 0.5) {
            flowerWedgeWait.reset();
            if (flowerWedgeIsDown) {
                flowerWedgeLeft.setPosition(flowerWedgeUp);
                flowerWedgeRight.setPosition(flowerWedgeUp);
                flowerWedgeIsDown = false;
            } else {
                flowerWedgeLeft.setPosition(flowerWedgeDown);
                flowerWedgeRight.setPosition(flowerWedgeDown);
                flowerWedgeIsDown = true;
            }
        }
        
    }
    public Transport(HardwareMap hardwareMap) {
        //DISTANCE, VELOCITY
        lut.add(0, 0);
        /*
        lut.add(20, 450);
        lut.add(30, 460);
        lut.add(35, 470);
        lut.add(40, 480);
        lut.add(45, 490);
        lut.add(55, 500);
        lut.add(60, 510);
        lut.add(65, 520);
        lut.add(70, 530);
        lut.add(75, 550);
        lut.add(80, 570);
        lut.add(85, 580);
        lut.add(90, 600);
        lut.add(95, 620);
        lut.add(100, 640);
        lut.add(105, 660);
        lut.add(115, 680);
        lut.add(125, 740);
        lut.add(147, 850);
        lut.add(1000, 850);
        */
        lut.createLUT();

        manualMode = new Toggle(false);

        if (isRed) {
            redToggle = new Toggle(true);
        } else {
            redToggle = new Toggle(false);
        }

        matchTimer = new ElapsedTime();
        matchTimer.reset();

        shootWait = new ElapsedTime();
        shootWait.reset();

        flowerWedgeWait = new ElapsedTime();
        flowerWedgeWait.reset();

        //laser = hardwareMap.get(DigitalChannel.class, "laser");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        //transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDFController(shooterp, shooteri, shooterd, kV);
        leftShooter = new Motor(hardwareMap, "leftShooter", Motor.GoBILDA.RPM_6000);
        rightShooter = new Motor(hardwareMap, "rightShooter", Motor.GoBILDA.RPM_6000);
        rightShooter.setInverted(true); // May change to leftShooter.setInverted(true); depending on which should be inverted
        shooter = new MotorGroup(leftShooter, rightShooter);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        transferPollenBlocker = hardwareMap.get(ServoImplEx.class, "transferPollenBlocker");
        transferPollenBlocker.setDirection(Servo.Direction.FORWARD);
        //transferNectar = hardwareMap.get(ServoImplEx.class, "transferNectar");
        //transferNectar.setDirection(Servo.Direction.FORWARD);

        hoodPollen = hardwareMap.get(ServoImplEx.class, "hoodPollen");
        hoodPollen.setDirection(Servo.Direction.REVERSE);
        //hoodNectar = hardwareMap.get(ServoImplEx.class, "hoodNectar");
        //hoodNectar.setDirection(Servo.Direction.REVERSE);

        flowerWedgeLeft = hardwareMap.get(ServoImplEx.class, "flowerWedgeLeft");
        //flowerWedgeLeft.setDirection(Servo.Direction.REVERSE);
        flowerWedgeRight = hardwareMap.get(ServoImplEx.class, "flowerWedgeRight");
        flowerWedgeRight.setDirection(Servo.Direction.REVERSE);

        //led = hardwareMap.get(ServoImplEx.class, "led");

        if (inAuto) {
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            transferPollenBlocker.setPosition(transferClosed);
            //transferNectar.setPosition(transferClosed);
            flowerWedgeLeft.setPosition(flowerWedgeDown);
            flowerWedgeRight.setPosition(flowerWedgeDown);
            hoodPollen.setPosition(hoodShort);
            //hoodNectar.setPosition(hoodShort);
            //led.setPosition(0);
        } else {
            intakePower = dormant;
            shooterVelocity = dormant;
            shooterVelocityTarget = dormant;
            transferPollenBlocker.setPosition(transferClosed);
            //transferNectar.setPosition(transferClosed);
            flowerWedgeLeft.setPosition(flowerWedgeDown);
            flowerWedgeRight.setPosition(flowerWedgeDown);
            hoodPollen.setPosition(hoodShort);
            //hoodNectar.setPosition(hoodShort);
            //led.setPosition(.6);
        }
    }

    public void update(double voltageMultiplier) {
        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            kV = 0;
        } else {
            shooterp = .025; // Change these values
            kV = .000400186335403727; // Change these values
        }

        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        transferPollenBlocker.setPosition(transferPosition);
        //transferNectar.setPosition(transferPosition);
        hoodPollen.setPosition(hoodPollenPosition);
        //hoodNectar.setPosition(hoodNectarPosition);

        shooterVelocity = shooter.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + Math.abs((Math.signum(shooterVelocityTarget) * kS * voltageMultiplier));
        shooter.set(Range.clip(shooterpid, -1, 1));

        switch (transportState) {
            case HOME:
                intakePower = dormant;
                transferPower = dormant;
                shooterVelocityTarget = dormant;
                transferPosition = transferClosed;
                flowerWedgeLeft.setPosition(flowerWedgeUp);
                flowerWedgeRight.setPosition(flowerWedgeUp);
                break;
            case OUTTAKE:
                intakePower = outtaking;
                transferPower = outtaking;
                shooterVelocityTarget = emergencyEject;
                transferPosition = transferClosed;
                break;
            case INTAKE:
                intakePower = intaking;
                transferPower = semiTransferring;
                shooterVelocityTarget = dormant;
                transferPosition = transferClosed;
                break;
            case POWER_SHOOTER_SHORT:
                intakePower = intaking;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingShort - 20;
                transferPosition = transferClosed;
                fireTolerance = 30;
                break;
            case POWER_SHOOTER_MED:
                intakePower = intaking;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingMed - 20;
                transferPosition = transferClosed;
                fireTolerance = 30;
                break;
            case POWER_SHOOTER_LONG:
                intakePower = dormant;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingLong - 20;
                transferPosition = transferClosed;
                fireTolerance = 30;
                break;
            case SHOOT:
                // if (shooterVelocityTarget <= 550) {
                //     fireTolerance = 150;
                // } else if (shooterVelocityTarget <= 660) {
                //     fireTolerance = 150;
                // } else {
                //     fireTolerance = 100;
                // }

                // if (shooterVelocityTarget >= 660 || shooterVelocityTarget <= 540) {
                //     transferPosition = transferOpen;
                //     intakePower = intaking;
                // } else {
                //     transferPosition = transferClosed;
                //     intakePower = intaking;
                // }
                intakePower = intaking;
                transferPower = intaking;
                //shooterVelocityTarget = shooting;
                transferPosition = transferOpen;
                fireTolerance = 150;
                break;
            default:
                transportState = TransportState.HOME;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2, double voltageMultiplier) {

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

        if (shooterVelocityTarget == 0) {
            shooterp = 0;
            kV = 0;
        } else {
            shooterp = .025; // Change these values
            kV = .000400186335403727; // Change these values
        }

        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
        transferPollenBlocker.setPosition(transferPosition);
        //transferNectar.setPosition(transferPosition);
        hoodPollen.setPosition(hoodPollenPosition);
        //hoodNectar.setPosition(hoodNectarPosition);

        shooterVelocity = shooter.getVelocity();
        shooterController.setP(shooterp);
        shooterController.setF(kV * voltageMultiplier);
        shooterpid = shooterController.calculate(shooterVelocity, shooterVelocityTarget) + Math.abs((Math.signum(shooterVelocityTarget) * kS * voltageMultiplier));
        shooter.set(Range.clip(shooterpid, -1, 1));

        switch (transportState) {
            case HOME:
                intakePower = dormant;
                transferPower = dormant;
                shooterVelocityTarget = dormant;
                transferPosition = dormant;
                setTransportState();
                break;
            case INTAKE:
                intakePower = intaking;
                transferPower = semiTransferring;
                shooterVelocityTarget = dormant;
                transferPosition = transferClosed;
                setTransportState();
                break;
            case OUTTAKE:
                intakePower = outtaking;
                transferPower = outtaking;
                transferPosition = transferClosed;
                shooterVelocityTarget = emergencyEject;
                if (!gamepad1.circle) {
                    TransportState = returnCase;
                }
                break;
            case POWER_SHOOTER:
                intakePower = dormant;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingShort;
                transferPosition = transferClosed;
                fireTolerance = 30;
                setTransportState();
                break;
            case POWER_SHOOTER_SHORT:
                intakePower = dormant;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingShort;
                transferPosition = transferClosed;
                hoodPollenPosition = hoodShort;
                fireTolerance = 30;
                setTransportState();
                break;
            case POWER_SHOOTER_MED:
                intakePower = dormant;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingMed;
                transferPosition = transferClosed;
                hoodPollenPosition = hoodMed;
                fireTolerance = 30;
                setTransportState();
                break;
            case POWER_SHOOTER_LONG:
                intakePower = dormant;
                transferPower = semiTransferring;
                shooterVelocityTarget = shootingLong;
                transferPosition = transferClosed;
                hoodPollenPosition = hoodLong;
                fireTolerance = 30;
                setTransportState();
                break;
            case AUTO_POWER:
                intakePower = dormant;
                transferPower = semiTransferring;
                // if (calcVelocity() < 860) {
                //     shooterVelocityTarget = calcVelocity();
                // } else {
                //     shooterVelocityTarget = 860;
                // }
                shooterVelocityTarget = calcVelocity();
                transferPosition = transferClosed;
                hoodPollenPosition = hoodMed;
                fireTolerance = 30;
                setTransportState();
                break;
            case SHOOT:
                if (shooterVelocityTarget <= 550) {
                    fireTolerance = 150;
                } else if (shooterVelocityTarget <= 660) {
                    fireTolerance = 150;
                } else {
                    fireTolerance = 100;
                }
                if ((Math.abs(Drive.rx) < .2) && (inRange(shooterVelocity, shooterVelocityTarget))) {
                    if (shooterVelocityTarget <= 660 && shooterVelocityTarget >= 560) {
                        transferPosition = transferOpen;
                        intakePower = intaking;
                        transferPower = intaking;
                    } else {
                        transferPosition = transferOpen;
                        intakePower = intaking;
                        transferPower = intaking;
                    }
                } else {
                    if (shooterVelocityTarget >= 660) {
                        transferPosition = transferClosed; //TODO: maybe dormant?
                    } else {
                        transferPosition = transferClosed;
                    }
                    intakePower = dormant;
                    transferPower = dormant;
                }
                setTransportState();
                break;
            default:
                TransportState = TransportState.HOME;
        }

        if (gamepad1.square && TransportState != TransportState.HOME) {
        TransportState = TransportState.HOME;
//                intakeToggle.value = false;
        }
    }
}
