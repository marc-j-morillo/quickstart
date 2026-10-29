package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Two positional (servo mode) servos driving the pollen turret together.
 * Find every value below with TurretServoTuner, then copy them in here.
 *
 * Angles are turret degrees: 0 = facing robot-forward, counterclockwise positive (same as the pose heading).
 */
@Configurable
public class TurretPollen {
    // -------------- Calibration (tick -> angle) --------------
    public static double centerPosition = 0.5;      // servo position with the turret facing robot-forward (0 deg)
    public static double degreesPerPosition = 240;  // turret degrees per 1.0 of servo position, negative if the turret turns the other way

    // -------------- Travel limits ("start" and "end" of the turret) --------------
    public static double minPosition = 0.02;
    public static double maxPosition = 0.98;

    // -------------- Per-servo trim, so the two servos don't fight each other --------------
    public static double leftTrim = 0.02;
    public static double rightTrim = -0.02;

    // -------------- Hardware --------------
    public static boolean leftReversed = false;
    public static boolean rightReversed = false;
    // FTC hubs default to 600-2400 us. Axon servos accept 500-2500 us, which unlocks their full programmed range.
    public static double pwmLowerUs = 600;
    public static double pwmUpperUs = 2400;

    public ServoImplEx turretPollenLeft, turretPollenRight;
    private double commandedPosition = 0.5;

    public TurretPollen(HardwareMap hardwareMap) {
        turretPollenLeft = hardwareMap.get(ServoImplEx.class, "turretPollenLeft");
        turretPollenRight = hardwareMap.get(ServoImplEx.class, "turretPollenRight");
        applyHardwareConfig();
    }

    /** Applies the direction and PWM range settings. Call again after changing them live. */
    public void applyHardwareConfig() {
        turretPollenLeft.setDirection(leftReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        turretPollenRight.setDirection(rightReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        PwmControl.PwmRange range = new PwmControl.PwmRange(pwmLowerUs, pwmUpperUs);
        turretPollenLeft.setPwmRange(range);
        turretPollenRight.setPwmRange(range);
    }

    /** Points the turret at a turret angle in degrees, clamped to the travel limits. */
    public void setAngle(double degrees) {
        update(angleToPosition(degrees));
    }

    /** Sets the turret servo position, clamped to the travel limits. */
    public void update(double turretTargetPosition) {
        setRawPosition(Range.clip(turretTargetPosition, Math.min(minPosition, maxPosition), Math.max(minPosition, maxPosition)));
    }

    /** Sets the servo position ignoring the travel limits (only clipped to 0..1). For finding the limits. */
    public void setRawPosition(double position) {
        commandedPosition = Range.clip(position, 0, 1);
        turretPollenLeft.setPosition(Range.clip(commandedPosition + leftTrim, 0, 1));
        turretPollenRight.setPosition(Range.clip(commandedPosition + rightTrim, 0, 1));
    }

    public double getCommandedPosition() {
        return commandedPosition;
    }

    public double getCommandedAngle() {
        return positionToAngle(commandedPosition);
    }

    public static double angleToPosition(double degrees) {
        if (degreesPerPosition == 0) return centerPosition;
        return centerPosition + degrees / degreesPerPosition;
    }

    public static double positionToAngle(double position) {
        return (position - centerPosition) * degreesPerPosition;
    }
}
