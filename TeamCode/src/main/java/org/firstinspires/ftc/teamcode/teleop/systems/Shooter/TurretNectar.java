
/*
package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.isRed;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;


public class TurretNectar {
    public static Servo turretNectarLeft, turretNectarRight;

    public TurretNectar(HardwareMap hardwareMap) {
        turretNectarLeft = hardwareMap.get(Servo.class, "turretNectarLeft");
        turretNectarRight = hardwareMap.get(Servo.class, "turretNectarRight");
        turretNectarLeft.setDirection(Servo.Direction.REVERSE);
        // turretNectarRight.setDirection(Servo.Direction.REVERSE);
    }

    public void update(double turretTargetPosition) {
        turretNectarLeft.setPosition(turretTargetPosition);
        turretNectarRight.setPosition(turretTargetPosition);
    }
}*/