package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretPollen {
    public static Servo turretPollenLeft, turretPollenRight;

    public TurretPollen(HardwareMap hardwareMap) {
        turretPollenLeft = hardwareMap.get(Servo.class, "turretPollenLeft");
        turretPollenRight = hardwareMap.get(Servo.class, "turretPollenRight");
        // turretPollenLeft.setDirection(Servo.Direction.REVERSE);
        // turretPollenRight.setDirection(Servo.Direction.REVERSE);
    }

    public void update(double turretTargetPosition) {
        turretPollenLeft.setPosition(turretTargetPosition + 0.02);
        turretPollenRight.setPosition(turretTargetPosition - 0.02);
    }
}