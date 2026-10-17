package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.systems.Localization;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.TurretPollen;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.startPose;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

@TeleOp(name = "TransferTest", group = "TeleOp")
public class TransferTest extends OpMode {

    //  ------------- TURRET -------------
    //TurretPollen turret;
    private double turretPosition;
    public static Servo turretPollenLeft, turretPollenRight;


    //  ------------- BLOCKER -------------

    public Servo blocker;

    //SERVO STATES
    public static final double blockerOpen = 0.6; // TEMPORARY VALUE
    public static final double blockerClosed = 0.85; // TEMPORARY VALUE
    public double blockerPosition = blockerOpen;
    private ElapsedTime matchTimer = new ElapsedTime();
    private double oldTime = 0;

    //  ------------- INTAKE -------------

    public DcMotorEx transferMotor;


    //MOTOR POWERS
    public static final double dormant = 0;
    public static final double intaking = 0.5; // TEMPORARY VALUE
    public static final double outtaking = intaking * -1;
    public static final double semiTransferring = intaking * 0.2; // 0.1 power

    private double transferPower = dormant;


    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setDirection(Servo.Direction.FORWARD);


        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        //transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretPollenLeft = hardwareMap.get(Servo.class, "turretPollenLeft");
        turretPollenRight = hardwareMap.get(Servo.class, "turretPollenRight");
        // turretPollenLeft.setDirection(Servo.Direction.REVERSE);
        // turretPollenRight.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void start() {
        matchTimer.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            blockerPosition = blockerClosed;
        } else {
            blockerPosition = blockerOpen;
        }
        blocker.setPosition(blockerPosition);


        if (gamepad1.right_trigger > 0.1) {
            transferPower = gamepad1.right_trigger;
        } else {
            transferPower = gamepad1.left_trigger * -1;
        }
        transferMotor.setPower(transferPower);

        // 5 - right
        // 4 - left
        turretPosition = gamepad1.left_stick_x;
        turretPollenLeft.setPosition(turretPosition);
        turretPollenRight.setPosition(turretPosition);

        telemetry.addData("Blocker position: ", blockerPosition);
        telemetry.addData("Blocker is open (t/f): ", blockerPosition == blockerOpen);
        telemetry.addData("Transfer power: ", transferPower);
        telemetry.addData("Turret position: ", turretPosition);

        telemetry.update();
    }
}
