package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.TurretPollenCR;

/**
 * Turret tuner for Axons in CONTINUOUS ROTATION mode with an absolute analog encoder (TurretPollenCR).
 * Everything it finds is written straight into TurretPollenCR's configurables. Panels values reset when the
 * app restarts, so copy the "COPY INTO TurretPollenCR" lines into the code.
 *
 * In CR mode the servo position command becomes a speed command (0.5 = stop), so the servo no longer knows
 * where it is: the encoder closes the loop. Robot config: turretPollenLeft / turretPollenRight as
 * "Continuous Rotation Servo", the encoder as "Analog Input" named turretEncoder.
 *
 * Steps (MANUAL mode first):
 *   1. Push the stick right. If the turret turns clockwise (seen from above), flip leftReversed / rightReversed
 *      in TurretPollenCR so positive power = counterclockwise. If the two servos fight, flip just one of them.
 *   2. If positive power makes the encoder angle go DOWN, flip encoderReversed.
 *   3. Encoder voltage range: if the encoder can turn a full turn, spin it once and copy "min / max seen" into
 *      encoderMinVoltage / encoderMaxVoltage (y resets them). Otherwise leave the defaults (0 - 3.3 V).
 *   4. Set turretDegreesPerEncoderDegree (1 if the encoder turns with the turret). Check it by turning the
 *      turret a measured 90 degrees and comparing with how much "Turret angle" changed.
 *   5. Point the turret forward and press a to zero it.
 *   6. Drive to each end of the travel, back off a little and record with dpad down / dpad up.
 *   7. KS mode: dpad right / left ramps the power until the turret moves, and sets kS to the larger of the two.
 *   8. HOLD and STEP modes: tune kP first (raise until it slightly overshoots), then kD to kill the overshoot,
 *      kI only if it stalls short of the target. STEP shows overshoot and settle time for each step.
 *
 * Controls (gamepad1):
 *   x ........................ next mode (MANUAL -> KS -> HOLD -> STEP)
 *   b ........................ stop (back to MANUAL with no power)
 *   MANUAL: right stick x = power, a = zero here, dpad down / up = record min / max angle, y = reset min/max voltage
 *   KS:     dpad right / left = ramp positive / negative power until the turret moves
 *   HOLD:   dpad left / right = -/+ angleStep, bumpers = -/+ 90 deg, y = 0 deg
 */
@TeleOp(name = "TurretCRTuner", group = "Tuning")
@Configurable
public class TurretCRTuner extends OpMode {
    public enum Mode { MANUAL, KS, HOLD, STEP }

    public static Mode mode = Mode.MANUAL;

    // MANUAL
    public static double manualMaxPower = 0.4;
    public static boolean manualIgnoresLimits = true; // so you can drive past the old limits to find new ones

    // KS
    public static double ksRampPerSecond = 0.05;  // how fast the power climbs
    public static double ksMoveDegrees = 2.0;     // movement that counts as "it moved"
    public static double ksMaxPower = 0.5;        // give up past this

    // HOLD
    public static double targetAngle = 0;
    public static double angleStep = 15;

    // STEP
    public static double stepAngleA = -45;
    public static double stepAngleB = 45;
    public static double stepPeriodSeconds = 2.0;
    public static double settleBandDegrees = 2.0;

    private TurretPollenCR turret;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    // Encoder voltage seen while running
    private double minVoltageSeen = Double.MAX_VALUE, maxVoltageSeen = -Double.MAX_VALUE;

    // KS ramp
    private int ksDirection = 0; // 0 = not ramping
    private double ksPower, ksStartAngle;
    private double ksPositive = Double.NaN, ksNegative = Double.NaN;
    private String ksStatus = "dpad right / left to ramp";

    // STEP results
    private boolean stepAtB = false;
    private double stepTarget, stepDirection, stepOvershoot, stepLastOutside;
    private double lastOvershoot = Double.NaN, lastSettleSeconds = Double.NaN, lastFinalError = Double.NaN;

    // Last applied settings, so changes in Panels get re-applied
    private boolean appliedLeftReversed, appliedRightReversed, appliedEncoderReversed;
    private double appliedMinVoltage, appliedMaxVoltage;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new TurretPollenCR(hardwareMap);
        rememberConfig();
        targetAngle = turret.getAngle();
    }

    @Override
    public void init_loop() {
        turret.readAngle();
        trackVoltage();
        telemetry.addData("Turret angle (deg)", "%.1f", turret.getAngle());
        telemetry.addData("Encoder voltage", "%.3f", turret.encoder.getVoltage());
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        stepTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        reapplyConfigIfChanged();

        if (gamepad1.xWasPressed()) {
            switchTo(Mode.values()[(mode.ordinal() + 1) % Mode.values().length]);
        }
        if (gamepad1.bWasPressed()) {
            switchTo(Mode.MANUAL);
        }

        boolean left = gamepad1.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed();
        boolean down = gamepad1.dpadDownWasPressed();
        boolean up = gamepad1.dpadUpWasPressed();
        boolean a = gamepad1.aWasPressed();
        boolean y = gamepad1.yWasPressed();
        boolean leftBumper = gamepad1.leftBumperWasPressed();
        boolean rightBumper = gamepad1.rightBumperWasPressed();

        switch (mode) {
            case MANUAL:
                turret.readAngle();
                turret.setPower(gamepad1.right_stick_x * manualMaxPower, manualIgnoresLimits);
                if (a) turret.zeroHere();
                if (down) TurretPollenCR.minAngle = turret.getAngle();
                if (up) TurretPollenCR.maxAngle = turret.getAngle();
                if (y) {
                    minVoltageSeen = Double.MAX_VALUE;
                    maxVoltageSeen = -Double.MAX_VALUE;
                }
                break;
            case KS:
                turret.readAngle();
                if (ksDirection == 0 && (right || left)) {
                    ksDirection = right ? 1 : -1;
                    ksPower = 0;
                    ksStartAngle = turret.getAngle();
                    ksStatus = "ramping " + (ksDirection > 0 ? "positive" : "negative");
                }
                runKsRamp(dt);
                break;
            case HOLD:
                if (left) targetAngle -= angleStep;
                if (right) targetAngle += angleStep;
                if (leftBumper) targetAngle -= 90;
                if (rightBumper) targetAngle += 90;
                if (y) targetAngle = 0;
                turret.setAngle(targetAngle);
                turret.update();
                break;
            case STEP:
                runStep();
                break;
        }
        trackVoltage();

        // ---------- Telemetry ----------
        telemetry.addData("Mode (x)", mode);
        telemetry.addData("Turret angle (deg)", "%.2f", turret.getAngle());
        telemetry.addData("Turret velocity (deg/s)", "%.1f", turret.getVelocity());
        telemetry.addData("Target angle (deg)", "%.2f", turret.getTargetAngle());
        telemetry.addData("Error (deg)", "%.2f", turret.getTargetAngle() - turret.getAngle());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addLine();

        telemetry.addLine("ENCODER:");
        telemetry.addData("Voltage", "%.3f", turret.encoder.getVoltage());
        telemetry.addData("Min / max seen (y resets)", "%.3f / %.3f", minVoltageSeen, maxVoltageSeen);
        telemetry.addData("Raw encoder (deg)", "%.1f", turret.rawEncoderDegrees());
        telemetry.addData("Travel (deg)", "%.1f to %.1f", TurretPollenCR.minAngle, TurretPollenCR.maxAngle);
        telemetry.addLine();

        if (mode == Mode.KS) {
            telemetry.addLine("KS:");
            telemetry.addData("Status", ksStatus);
            telemetry.addData("Positive / negative", "%.3f / %.3f", ksPositive, ksNegative);
            telemetry.addLine();
        }
        if (mode == Mode.STEP) {
            telemetry.addLine("LAST STEP:");
            telemetry.addData("Overshoot (deg)", "%.2f", lastOvershoot);
            telemetry.addData("Settle time (s, = period means it never settled)", "%.2f", lastSettleSeconds);
            telemetry.addData("Error at end of step (deg)", "%.2f", lastFinalError);
            telemetry.addLine();
        }

        telemetry.addLine("COPY INTO TurretPollenCR:");
        telemetry.addLine(String.format("leftReversed = %b; rightReversed = %b;", TurretPollenCR.leftReversed, TurretPollenCR.rightReversed));
        telemetry.addLine(String.format("encoderMinVoltage = %.3f; encoderMaxVoltage = %.3f;", TurretPollenCR.encoderMinVoltage, TurretPollenCR.encoderMaxVoltage));
        telemetry.addLine(String.format("encoderReversed = %b; encoderOffsetDegrees = %.2f;", TurretPollenCR.encoderReversed, TurretPollenCR.encoderOffsetDegrees));
        telemetry.addLine(String.format("turretDegreesPerEncoderDegree = %.4f;", TurretPollenCR.turretDegreesPerEncoderDegree));
        telemetry.addLine(String.format("minAngle = %.1f; maxAngle = %.1f;", TurretPollenCR.minAngle, TurretPollenCR.maxAngle));
        telemetry.addLine(String.format("kP = %.5f; kI = %.5f; kD = %.5f; kS = %.3f;", TurretPollenCR.kP, TurretPollenCR.kI, TurretPollenCR.kD, TurretPollenCR.kS));
        telemetry.update();
    }

    private void switchTo(Mode newMode) {
        mode = newMode;
        ksDirection = 0;
        turret.setPower(0);
        targetAngle = turret.getAngle(); // HOLD starts where the turret is, so it doesn't jump
        stepTimer.reset();
        stepAtB = false;
        startStep(stepAngleA);
    }

    // ------------------------------------------------------------------
    // KS: ramp the power up slowly until the turret moves
    // ------------------------------------------------------------------

    private void runKsRamp(double dt) {
        if (ksDirection == 0) {
            turret.setPower(0);
            return;
        }

        ksPower += ksRampPerSecond * dt;
        boolean moved = Math.abs(turret.getAngle() - ksStartAngle) > ksMoveDegrees;

        if (moved) {
            if (ksDirection > 0) ksPositive = ksPower;
            else ksNegative = ksPower;
            if (!Double.isNaN(ksPositive) && !Double.isNaN(ksNegative)) {
                TurretPollenCR.kS = Math.max(ksPositive, ksNegative);
                ksStatus = "kS set to the larger direction";
            } else {
                ksStatus = "now ramp the other direction";
            }
            ksDirection = 0;
            turret.setPower(0);
        } else if (ksPower > ksMaxPower) {
            ksStatus = "didn't move by ksMaxPower (at a travel limit?)";
            ksDirection = 0;
            turret.setPower(0);
        } else {
            turret.setPower(ksDirection * ksPower);
            if (ksPower > 0 && turret.getPower() == 0) {
                ksStatus = "at a travel limit, ramp the other way or move away first";
                ksDirection = 0;
            }
        }
    }

    // ------------------------------------------------------------------
    // STEP: alternate between two angles and measure each step
    // ------------------------------------------------------------------

    private void runStep() {
        if (stepTimer.seconds() > Math.max(stepPeriodSeconds, 0.2)) {
            // Score the step that just finished, then start the next one
            lastOvershoot = stepOvershoot;
            lastSettleSeconds = stepLastOutside;
            lastFinalError = stepTarget - turret.getAngle();
            stepAtB = !stepAtB;
            startStep(stepAtB ? stepAngleB : stepAngleA);
        }

        turret.setAngle(stepTarget);
        turret.update();

        double error = turret.getAngle() - stepTarget;
        stepOvershoot = Math.max(stepOvershoot, stepDirection * error);
        if (Math.abs(error) > settleBandDegrees) stepLastOutside = stepTimer.seconds();
    }

    private void startStep(double target) {
        stepDirection = Math.signum(target - turret.getAngle());
        stepTarget = target;
        stepOvershoot = 0;
        stepLastOutside = 0;
        stepTimer.reset();
    }

    // ------------------------------------------------------------------

    private void trackVoltage() {
        double voltage = turret.encoder.getVoltage();
        minVoltageSeen = Math.min(minVoltageSeen, voltage);
        maxVoltageSeen = Math.max(maxVoltageSeen, voltage);
    }

    private void rememberConfig() {
        appliedLeftReversed = TurretPollenCR.leftReversed;
        appliedRightReversed = TurretPollenCR.rightReversed;
        appliedEncoderReversed = TurretPollenCR.encoderReversed;
        appliedMinVoltage = TurretPollenCR.encoderMinVoltage;
        appliedMaxVoltage = TurretPollenCR.encoderMaxVoltage;
    }

    private void reapplyConfigIfChanged() {
        if (appliedLeftReversed != TurretPollenCR.leftReversed || appliedRightReversed != TurretPollenCR.rightReversed) {
            turret.applyHardwareConfig();
        }
        if (appliedEncoderReversed != TurretPollenCR.encoderReversed
                || appliedMinVoltage != TurretPollenCR.encoderMinVoltage
                || appliedMaxVoltage != TurretPollenCR.encoderMaxVoltage) {
            turret.resyncEncoder();
        }
        rememberConfig();
    }
}
