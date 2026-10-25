package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


public class HoodPollen {
    public static Servo hoodPollen;

    public HoodPollen(HardwareMap hardwareMap) {
        hoodPollen = hardwareMap.get(Servo.class, "hoodPollen");
        hoodPollen.setDirection(Servo.Direction.REVERSE);
    }

    public void update(double hoodTargetPosition) {
        hoodPollen.setPosition(hoodTargetPosition);
    }
}
