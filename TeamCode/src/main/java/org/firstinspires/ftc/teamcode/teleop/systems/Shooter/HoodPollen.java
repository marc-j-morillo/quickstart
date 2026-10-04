package org.firstinspires.ftc.teamcode.teleop.systems;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class HoodPollen {
    public static Servo hoodPollen;

    public HoodPollen(HardwareMap hardwareMap) {
        hoodPollen = hardwareMap.get(Servo.class, "hoodPollen");
    }

    public void update(double hoodTargetPosition) {
        hoodPollen.setPosition(hoodTargetPosition);
    }
}
