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
    // Turret degrees per 1.0 of servo position, negative = higher position turns the turret clockwise (right).
    // Measured at the PWM range below. If you change the PWM range, scale this by (new span / old span),
    // or re-measure with TurretServoTuner.
    public static double degreesPerPosition = -345;

    // -------------- Travel limits ("start" and "end" of the turret) --------------
    public static double minPosition = 0;
    public static double maxPosition = 1;

    // -------------- Per-servo trim, so the two servos don't fight each other --------------
    public static double leftTrim = 0;
    public static double rightTrim = 0;

    // -------------- Hardware --------------
    public static boolean leftReversed = false;
    public static boolean rightReversed = false;
    // FTC hubs default to 600-2400 us. Axon servos accept 500-2500 us, which unlocks their full programmed range.
    // In servo mode the pulse width is the POSITION command, not power: narrowing it shrinks the travel.
    public static double pwmLowerUs = 500;
    public static double pwmUpperUs = 2500;

    // -------------- Output smoothing --------------
    // The aim math is unchanged: only the position sent to the servos is smoothed (a low-pass filter), so a
    // sudden big change in the target is spread over a short time instead of sent all at once, and quick
    // back-and-forth jumps mostly cancel out. smoothingTimeMs = time to cover about 63% of a jump (about 3x this
    // to get within 5% of it). Bigger = smoother but laggier. 0 = no smoothing (send the target as-is).
    public static double smoothingTimeMs = 60;

    // -------------- Power reduction (no Axon programmer needed) --------------
    // false = drive only the left servo and leave the right one unpowered (PWM off): half the torque, and the
    // two servos can't fight each other. Takes effect on the next applyHardwareConfig (opmode init).
    public static boolean rightServoEnabled = true;

    public ServoImplEx turretPollenLeft, turretPollenRight;
    private double commandedPosition = 0.5;
    private long lastUpdateNanos = 0;
    private boolean outputEnabled = true;

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
        if (rightServoEnabled) turretPollenRight.setPwmEnable();
        else turretPollenRight.setPwmDisable();
    }

    /** Points the turret at a turret angle in degrees, clamped to the travel limits. */
    public void setAngle(double degrees) {
        update(angleToPosition(degrees));
    }

    /**
     * Points the turret at a direction (any angle, +/- 360 doesn't matter), for auto-aim. Uses whichever
     * equivalent angle is inside the travel limits. If none is (the target is in the dead zone past the limits),
     * it holds the limit the turret is already closest to, so localizer noise can't flip it between both ends.
     * The turret only swings across once the target is reachable again on the other side.
     */
    public void aimAt(double degrees) {
        double lo = Math.min(positionToAngle(lowestPosition()), positionToAngle(highestPosition()));
        double hi = Math.max(positionToAngle(lowestPosition()), positionToAngle(highestPosition()));
        double current = getCommandedAngle();
        double wrapped = ((degrees % 360) + 540) % 360 - 180; // -180..180

        double best = Double.NaN;
        for (int turns = -1; turns <= 1; turns++) {
            double candidate = wrapped + 360 * turns;
            if (candidate < lo || candidate > hi) continue;
            if (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current)) best = candidate;
        }
        if (Double.isNaN(best)) best = Math.abs(current - lo) < Math.abs(current - hi) ? lo : hi;

        update(angleToPosition(best));
    }

    /**
     * Points the turret at a servo position, clamped to the travel limits. Call every loop: the position sent
     * moves smoothly toward the target (see smoothingTimeMs). The first call goes straight to the target.
     */
    public void update(double turretTargetPosition) {
        double target = Range.clip(turretTargetPosition, lowestPosition(), highestPosition());

        long now = System.nanoTime();
        double output = target;
        if (smoothingTimeMs > 0 && lastUpdateNanos != 0) {
            // Time-based, so it smooths the same at any loop time; dt is capped so a long pause can't cause a jump
            double dtMs = Math.min((now - lastUpdateNanos) / 1e6, 100);
            double alpha = 1 - Math.exp(-dtMs / smoothingTimeMs);
            output = commandedPosition + alpha * (target - commandedPosition);
        }
        lastUpdateNanos = now;

        setRawPosition(output);
    }

    /**
     * Lowest / highest position the turret may be commanded to: the travel limits, narrowed so that position + trim
     * stays inside 0..1 for every powered servo. Otherwise a trimmed servo stops at 0 or 1 while the other keeps
     * going, and the two fight at that end of travel only.
     */
    private static double lowestPosition() {
        double low = Math.max(Math.min(minPosition, maxPosition), -leftTrim);
        if (rightServoEnabled) low = Math.max(low, -rightTrim);
        return low;
    }

    private static double highestPosition() {
        double high = Math.min(Math.max(minPosition, maxPosition), 1 - leftTrim);
        if (rightServoEnabled) high = Math.min(high, 1 - rightTrim);
        return high;
    }

    /** Sets the servo position right away, ignoring the travel limits and smoothing (only clipped to 0..1). For tuning. */
    public void setRawPosition(double position) {
        commandedPosition = Range.clip(position, 0, 1);
        if (outputEnabled) sendCommandedPosition();
    }

    private void sendCommandedPosition() {
        turretPollenLeft.setPosition(Range.clip(commandedPosition + leftTrim, 0, 1));
        // Setting a position turns the PWM back on, so skip the right servo while it's meant to be off
        if (rightServoEnabled) turretPollenRight.setPosition(Range.clip(commandedPosition + rightTrim, 0, 1));
    }

    /**
     * Turns the servo signals off (the turret goes limp) or back on. Only the hardware output changes:
     * aiming, the speed limit and the commanded position keep updating, and turning it back on sends the
     * current command.
     */
    public void setOutputEnabled(boolean enabled) {
        if (enabled == outputEnabled) return;
        outputEnabled = enabled;
        if (enabled) {
            turretPollenLeft.setPwmEnable();
            if (rightServoEnabled) turretPollenRight.setPwmEnable();
            sendCommandedPosition();
        } else {
            turretPollenLeft.setPwmDisable();
            turretPollenRight.setPwmDisable();
        }
    }

    public boolean isOutputEnabled() {
        return outputEnabled;
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
