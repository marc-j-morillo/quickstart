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
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    public static double semiTransferring = 0.70;

    //SERVO STATES
    public static double blockerOpen = 0.6;
    public static double blockerClosed = 0.85;

    //FLOWER WEDGE POSITIONS (each servo has its own; find them with FlowerWedgeTest, which edits these)
    public static double leftWedgeDown = 0.79;
    public static double leftWedgeUp = 0.1;
    public static double rightWedgeDown = 0.6;  // TEMPORARY VALUE
    public static double rightWedgeUp = 0;   // TEMPORARY VALUE
    public static double flowerWedgeDebounceSeconds = 0.5;

    /** Puts both wedge servos at their own up or down position. Used here and by the teleops/tests. */
    public static void setFlowerWedge(Servo left, Servo right, boolean down) {
        left.setPosition(down ? leftWedgeDown : leftWedgeUp);
        right.setPosition(down ? rightWedgeDown : rightWedgeUp);
    }
    public boolean flowerWedgeIsDown;

    // Flower wedge on/off. Needs the flowerWedgeLeft / flowerWedgeRight servos in the robot config.
    public static final boolean flowerWedgeEnabled = true;
    public ElapsedTime flowerWedgeWait;

    //MOTOR DIRECTIONS (checked on the robot with the motors in the correct ports). Applied at init.
    public static boolean intakeReversed = true;
    public static boolean transferReversed = false;

    // -------------- Direct control (competition teleop) --------------
    // runIntake / runFeed / runOuttake / stopRollers / setGateOpen / setFlowerWedgeDown, called by the opmode's
    // own state machine instead of update(). Separate powers from the transport-state ones above, which autos use.
    public static double directIntakePower = .9;
    public static double directIntakeTransferPower = .8; // transfer while intaking (stops when it stalls)
    public static double directFeedTransferPower = .57;     // transfer while shooting (no stall check)
    public static double directOuttakePower = -1;
    public static double transferStallMilliamps = 5000;   // transfer current above this while intaking = it's full
    public static int stallCheckEveryLoops = 5;           // current isn't in the bulk read, so don't read it every loop

    // -------------- Auto stall check (update(boolean)) --------------
    // In INTAKE / POWER_SHOOTER the transfer runs at semiTransferring (0.18), so it can't reach the teleop 5000 mA:
    // a stalled motor at 0.18 power draws roughly 0.18 x its full stall current. Robot.update(Pose) clears the
    // "full" flag after every path (resetTransferStall), and SHOOT / OUTTAKE clear it too.
    public static double autoTransferStallMilliamps = 4000; // TEMPORARY VALUE: check "Transfer current" telemetry
    public static double stallIgnoreMs = 250; // ignore the current spike when the transfer (re)starts

    @IgnoreConfigurable public static double lastTransferMilliamps = 0; // last current read by the auto stall check

    private boolean transferFull = false;
    private int stallCheckCount = 0;
    private final ElapsedTime transferRunTime = new ElapsedTime(); // since the transfer last started running
    private boolean transferWasRunning = false;





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
            if (flowerWedgeEnabled) setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, false);
            //led.setPosition(0);
        } else {
            intakePower = dormant;
            transferPollenBlocker.setPosition(blockerClosed);
            //transferNectar.setPosition(blockerClosed);
            if (flowerWedgeEnabled) setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, false);
            //led.setPosition(.6);
        }
    }


    // -------------- AUTONOMOUS UPDATE METHOD --------------
    public void update(boolean flowerWedgeIsActive) {
        if (flowerWedgeEnabled) setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, flowerWedgeIsActive);


        switch (Robot.transportState) {
            case HOME:
                transferFull = false;
                intakePower = dormant;
                transferPower = dormant;
                blockerPosition = blockerClosed;
                break;
            case INTAKE:
                intakePower = intaking;
                transferPower = autoTransferWithStallCheck(semiTransferring);
                blockerPosition = blockerClosed;
                break;
            case OUTTAKE:
                transferFull = false;
                intakePower = outtaking;
                transferPower = outtaking;
                blockerPosition = blockerClosed;
                break;
            case POWER_SHOOTER:
                // Intake and transfer off while the flywheel first spins up at the start of auto (preload shot)
                intakePower = Robot.preloadSpinUp ? dormant : intaking;
                transferPower = Robot.preloadSpinUp ? dormant : autoTransferWithStallCheck(semiTransferring);
                blockerPosition = blockerClosed;
                break;
            case SHOOT:
                transferFull = false; // emptying, so no stall check
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
        transferWasRunning = transferPower > 0.1;
    }

    /**
     * Auto transfer power with the stall check: runPower until the transfer current (read every
     * stallCheckEveryLoops calls, not in the first stallIgnoreMs after it starts) goes over
     * autoTransferStallMilliamps, then 0 until resetTransferStall() / SHOOT / OUTTAKE.
     */
    private double autoTransferWithStallCheck(double runPower) {
        if (transferFull) return 0;
        if (!transferWasRunning) transferRunTime.reset(); // starting up this loop

        stallCheckCount = (stallCheckCount + 1) % stallCheckEveryLoops;
        if (stallCheckCount == 0 && transferRunTime.milliseconds() > stallIgnoreMs) {
            lastTransferMilliamps = transferMotor.getCurrent(CurrentUnit.MILLIAMPS);
            if (lastTransferMilliamps > autoTransferStallMilliamps) transferFull = true;
        }
        return transferFull ? 0 : runPower;
    }

    /** Clears the "transfer full" flag, so the transfer runs again (and is re-checked). Robot calls this after every auto path. */
    public void resetTransferStall() {
        transferFull = false;
        stallCheckCount = 0;
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
            // call toggleFlowerWedge once: it consumes the bumper press, so a second call would miss it
            setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, toggleFlowerWedge(gamepad1));
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

    // -------------- DIRECT CONTROL METHODS --------------

    /** Intake + transfer. Stops the transfer once its current shows it's full (checked every stallCheckEveryLoops calls). */
    public void runIntake() {
        stallCheckCount = (stallCheckCount + 1) % stallCheckEveryLoops;
        if (!transferFull && stallCheckCount == 0
                && transferMotor.getCurrent(CurrentUnit.MILLIAMPS) > transferStallMilliamps) {
            transferFull = true;
        }
        setRollers(directIntakePower, transferFull ? 0 : directIntakeTransferPower);
    }

    /** Shooting: feed everything into the flywheel, no stall check (the transfer is emptying). */
    public void runFeed() {
        transferFull = false;
        setRollers(directIntakePower, directFeedTransferPower);
    }

    /** Push everything back out. */
    public void runOuttake() {
        transferFull = false;
        setRollers(directOuttakePower, directOuttakePower);
    }

    /** Intake and transfer off (keeps the "transfer full" flag). */
    public void stopRollers() {
        setRollers(dormant, dormant);
    }

    public void setGateOpen(boolean open) {
        blockerPosition = open ? blockerOpen : blockerClosed;
        transferPollenBlocker.setPosition(blockerPosition);
    }

    public void setFlowerWedgeDown(boolean down) {
        flowerWedgeIsDown = down;
        if (flowerWedgeEnabled) setFlowerWedge(flowerWedgeLeft, flowerWedgeRight, down);
    }

    public boolean isTransferFull() {
        return transferFull;
    }

    private void setRollers(double intakePower, double transferPower) {
        this.intakePower = intakePower;
        this.transferPower = transferPower;
        intake.setPower(intakePower);
        transferMotor.setPower(transferPower);
    }

    /** Returns whether the wedge should be down, toggled with the left bumper. */
    public boolean toggleFlowerWedge(Gamepad gamepad1) {
        // Raise and lower the flower wedge with the left bumper,
        // but only if flowerWedgeDebounceSeconds have passed since the last press to prevent rapid toggling
        if (gamepad1.leftBumperWasPressed() && flowerWedgeWait.seconds() > flowerWedgeDebounceSeconds) {
            flowerWedgeWait.reset();
            flowerWedgeIsDown = !flowerWedgeIsDown;
        }
        return flowerWedgeIsDown;
    }
}