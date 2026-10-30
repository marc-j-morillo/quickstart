package org.firstinspires.ftc.teamcode.teleop.systems.Shooter;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Pollen turret driven by two servos in CONTINUOUS ROTATION mode (Axons switched to CR mode with the
 * Axon programmer), closed loop on an absolute analog encoder (e.g. the Axon's 4th feedback wire).
 * Find every value below with TurretCRTuner, then copy them in here.
 *
 * Robot config: set turretPollenLeft / turretPollenRight to "Continuous Rotation Servo", and add the encoder
 * as an "Analog Input" named turretEncoder.
 *
 * Angles are turret degrees: 0 = facing robot-forward, counterclockwise positive (same as the pose heading).
 */
@Configurable
public class TurretPollenCR {
    // -------------- Hardware --------------
    public static boolean leftReversed = false;
    public static boolean rightReversed = false;

    // -------------- Absolute encoder --------------
    public static double encoderMinVoltage = 0.0;      // voltage at 0 deg of the encoder (watch "min seen" in TurretCRTuner)
    public static double encoderMaxVoltage = 3.3;      // voltage at 360 deg of the encoder (watch "max seen")
    public static boolean encoderReversed = false;     // flip so positive power reads as a positive angle
    public static double encoderOffsetDegrees = 0;     // encoder reading when the turret faces robot-forward
    // Turret degrees per encoder degree: 1 if the encoder turns with the turret, servo teeth / turret teeth if
    // it reads the servo. If it isn't 1, the turret must start within +/-180 encoder degrees of forward, since
    // the encoder is only absolute for one turn and the rest is counted while running.
    public static double turretDegreesPerEncoderDegree = 1.0;

    // -------------- Travel limits (turret degrees) --------------
    public static double minAngle = -120;
    public static double maxAngle = 120;

    // -------------- Position controller --------------
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0005;
    public static double kS = 0.05;              // minimum power that makes the turret move (find it in KS mode)
    public static double maxPower = 1.0;
    public static double toleranceDegrees = 1.0; // stop pushing inside this error

    public CRServo turretPollenLeft, turretPollenRight;
    public AnalogInput encoder;
    private final PIDFController controller = new PIDFController(kP, kI, kD, 0);
    private final ElapsedTime velocityTimer = new ElapsedTime();

    private boolean encoderInitialized = false;
    private double lastRawDegrees;
    private int encoderTurns;

    private double angle, velocity, targetAngle, power;

    public TurretPollenCR(HardwareMap hardwareMap) {
        turretPollenLeft = hardwareMap.get(CRServo.class, "turretPollenLeft");
        turretPollenRight = hardwareMap.get(CRServo.class, "turretPollenRight");
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        applyHardwareConfig();
        readAngle();
        targetAngle = angle;
    }

    /** Applies the servo directions. Call again after changing them live. */
    public void applyHardwareConfig() {
        turretPollenLeft.setDirection(leftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turretPollenRight.setDirection(rightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /** Sets the target turret angle, clamped to the travel limits. Call update() every loop to drive there. */
    public void setAngle(double degrees) {
        targetAngle = Range.clip(degrees, Math.min(minAngle, maxAngle), Math.max(minAngle, maxAngle));
    }

    /** Reads the encoder and runs the position controller toward the target angle. */
    public void update() {
        readAngle();

        double error = targetAngle - angle;
        if (Math.abs(error) < toleranceDegrees) {
            controller.reset();
            setPower(0);
            return;
        }

        controller.setPIDF(kP, kI, kD, 0);
        double output = controller.calculate(angle, targetAngle) + Math.signum(error) * kS;
        setPower(Range.clip(output, -maxPower, maxPower));
    }

    /** Reads the encoder without driving (for manual power). */
    public void readAngle() {
        double raw = rawEncoderDegrees();
        boolean justInitialized = !encoderInitialized;

        if (!encoderInitialized) {
            // Pick the turn that puts the turret closest to forward
            encoderTurns = (int) Math.round((encoderOffsetDegrees - raw) / 360.0);
            encoderInitialized = true;
            velocityTimer.reset();
        } else {
            // Count wraps between loops so the angle keeps going past one encoder turn
            double delta = raw - lastRawDegrees;
            if (delta > 180) encoderTurns--;
            else if (delta < -180) encoderTurns++;
        }
        lastRawDegrees = raw;

        double newAngle = (raw + 360.0 * encoderTurns - encoderOffsetDegrees) * turretDegreesPerEncoderDegree;
        double dt = velocityTimer.seconds();
        velocityTimer.reset();
        velocity = (justInitialized || dt <= 0) ? 0 : (newAngle - angle) / dt;
        angle = newAngle;
    }

    /**
     * Re-reads the absolute position from scratch. Call after changing the encoder voltages or direction,
     * since those make the raw reading jump and the turn counter would miscount it as a wrap.
     */
    public void resyncEncoder() {
        encoderInitialized = false;
        readAngle();
    }

    /** Raw encoder angle, 0..360, before the offset and gear ratio. */
    public double rawEncoderDegrees() {
        double span = encoderMaxVoltage - encoderMinVoltage;
        if (span <= 0) span = encoder.getMaxVoltage();
        double degrees = (encoder.getVoltage() - encoderMinVoltage) / span * 360.0;
        degrees = ((degrees % 360) + 360) % 360;
        return encoderReversed ? (360 - degrees) % 360 : degrees;
    }

    /** Makes the current position read as 0 deg (turret facing forward). */
    public void zeroHere() {
        encoderOffsetDegrees = rawEncoderDegrees();
        encoderTurns = 0;
        lastRawDegrees = encoderOffsetDegrees;
        angle = 0;
        targetAngle = 0;
    }

    /**
     * Drives both servos at a power. Power that would push further past a travel limit is cut,
     * unless ignoreLimits is true (for finding the limits).
     */
    public void setPower(double power, boolean ignoreLimits) {
        if (!ignoreLimits) {
            if (angle >= Math.max(minAngle, maxAngle) && power > 0) power = 0;
            if (angle <= Math.min(minAngle, maxAngle) && power < 0) power = 0;
        }
        this.power = power;
        turretPollenLeft.setPower(power);
        turretPollenRight.setPower(power);
    }

    public void setPower(double power) {
        setPower(power, false);
    }

    public double getAngle() {
        return angle;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getPower() {
        return power;
    }
}
