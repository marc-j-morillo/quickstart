package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class HoodPollen {
    // Hood travel limits (servo position), every command is clamped to these.
    // Same limits as ActuallyEverythingManualTest / ActuallyEverythingTest (HoodTest uses 0.3, double check on the robot).
    public static double hoodMin = 0;
    public static double hoodMax = 0.81;

    public Servo hoodPollen;

    public HoodPollen(HardwareMap hardwareMap) {
        hoodPollen = hardwareMap.get(Servo.class, "hoodPollen");
        hoodPollen.setDirection(Servo.Direction.REVERSE);
    }

    public void update(double hoodTargetPosition) {
        hoodPollen.setPosition(clamp(hoodTargetPosition));
    }

    public static double clamp(double hoodPosition) {
        return Range.clip(hoodPosition, hoodMin, hoodMax);
    }
}
