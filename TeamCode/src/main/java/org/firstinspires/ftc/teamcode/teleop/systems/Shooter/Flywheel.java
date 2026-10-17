package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.inAuto;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

public class Flywheel {
    public static MotorEx leftShooter, rightShooter;
    public static MotorGroup shooter;

    public Flywheel(HardwareMap hardwareMap) {
        leftShooter = new MotorEx(hardwareMap, "leftShooter", Motor.GoBILDA.BARE);
        rightShooter = new MotorEx(hardwareMap, "rightShooter", Motor.GoBILDA.BARE);

        rightShooter.setInverted(true); // May change to leftShooter.setInverted(true)

        shooter = new MotorGroup(leftShooter, rightShooter);
    }

    public void update(double power) {
        shooter.set(power);
    }
}