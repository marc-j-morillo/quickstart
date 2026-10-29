package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.TurretPollen;

/**
 * Turret tuner for SERVO MODE (positional) servos, e.g. Axons programmed in Servo mode.
 * Everything it finds is written straight into TurretPollen's configurables, so the real robot code uses it
 * right away. Panels values reset when the app restarts, so copy the "COPY INTO TurretPollen" lines into the code.
 *
 * Servo mode vs continuous mode: in servo mode the servo holds whatever 0..1 position you command, and its
 * total travel is set by the Axon programmer plus the hub's PWM range (pwmLowerUs..pwmUpperUs in TurretPollen,
 * FTC default 600-2400 us, Axon full range 500-2500 us). Change the PWM range and the tick -> angle
 * calibration changes too, so set it first. For continuous rotation + encoder, use TurretCRTuner instead.
 *
 * Steps:
 *   1. JOG mode: jog the turret to each hard stop, back off a little, and record it as start / end
 *      (dpad down / dpad up). Don't hold the servos against a hard stop, they stall.
 *   2. Point the turret at a known angle (e.g. -90 = robot-right, use a square) and press a,
 *      then at a second known angle (e.g. +90 = robot-left) and press b. This sets centerPosition and
 *      degreesPerPosition (tick -> angle). Set calAngleA / calAngleB in Panels if you use other angles.
 *   3. ANGLE mode: command angles with the dpad and check the turret actually points there.
 *   4. SWEEP mode: runs start <-> end so you can check the full range and that the two servos don't fight
 *      (buzzing / heat = adjust leftTrim / rightTrim in TurretPollen).
 *
 * Controls (gamepad1):
 *   x ........................ next mode (JOG -> ANGLE -> SWEEP)
 *   dpad left / right ........ JOG: -/+ fineStep     ANGLE: -/+ angleStep
 *   left / right bumper ...... JOG: -/+ coarseStep   ANGLE: -/+ 90 deg
 *   y ........................ go to center (0 deg)
 *   dpad down / dpad up ...... record the current position as the start / end of travel
 *   a / b .................... record calibration point A / B at the current position
 */
@TeleOp(name = "TurretServoTuner", group = "Tuning")
@Configurable
public class TurretServoTuner extends OpMode {
    public enum Mode { JOG, ANGLE, SWEEP }

    public static Mode mode = Mode.JOG;

    // JOG mode (servo position, ignores the travel limits so you can find them)
    public static double jogPosition = 0.5;
    public static double fineStep = 0.005;
    public static double coarseStep = 0.05;

    // ANGLE mode (turret degrees, uses the calibration and the travel limits)
    public static double targetAngle = 0;
    public static double angleStep = 15;

    // SWEEP mode
    public static double sweepPeriodSeconds = 2.0;

    // Calibration: the real turret angles you point at before pressing a / b
    public static double calAngleA = -90;
    public static double calAngleB = 90;

    // Display only: turret degrees per servo degree (servo gear teeth / turret gear teeth),
    // used to show the servo's own range so you can compare it with what the Axon is programmed to
    public static double turretDegreesPerServoDegree = 1.0;

    private TurretPollen turret;
    private final ElapsedTime sweepTimer = new ElapsedTime();

    private double calPositionA = Double.NaN, calPositionB = Double.NaN;
    private String calStatus = "not calibrated yet";

    // Last applied hardware settings, so changes in Panels get re-applied
    private boolean appliedLeftReversed, appliedRightReversed;
    private double appliedPwmLower, appliedPwmUpper;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new TurretPollen(hardwareMap);
        rememberHardwareConfig();
        jogPosition = TurretPollen.centerPosition;
    }

    @Override
    public void start() {
        sweepTimer.reset();
    }

    @Override
    public void loop() {
        reapplyHardwareConfigIfChanged();

        if (gamepad1.xWasPressed()) {
            mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            // Start the new mode where the turret already is, so it doesn't jump
            jogPosition = turret.getCommandedPosition();
            targetAngle = turret.getCommandedAngle();
            sweepTimer.reset();
        }

        boolean left = gamepad1.dpadLeftWasPressed();
        boolean right = gamepad1.dpadRightWasPressed();
        boolean leftBumper = gamepad1.leftBumperWasPressed();
        boolean rightBumper = gamepad1.rightBumperWasPressed();
        boolean center = gamepad1.yWasPressed();

        switch (mode) {
            case JOG:
                if (left) jogPosition -= fineStep;
                if (right) jogPosition += fineStep;
                if (leftBumper) jogPosition -= coarseStep;
                if (rightBumper) jogPosition += coarseStep;
                if (center) jogPosition = TurretPollen.centerPosition;
                jogPosition = Range.clip(jogPosition, 0, 1);
                turret.setRawPosition(jogPosition);
                break;
            case ANGLE:
                if (left) targetAngle -= angleStep;
                if (right) targetAngle += angleStep;
                if (leftBumper) targetAngle -= 90;
                if (rightBumper) targetAngle += 90;
                if (center) targetAngle = 0;
                turret.setAngle(targetAngle);
                break;
            case SWEEP:
                boolean atEnd = (int) (sweepTimer.seconds() / Math.max(sweepPeriodSeconds, 0.1)) % 2 == 1;
                turret.update(atEnd ? TurretPollen.maxPosition : TurretPollen.minPosition);
                break;
        }

        // ---------- Travel limits ----------
        double position = turret.getCommandedPosition();
        if (gamepad1.dpadDownWasPressed()) TurretPollen.minPosition = position;
        if (gamepad1.dpadUpWasPressed()) TurretPollen.maxPosition = position;

        // ---------- Calibration (tick -> angle) ----------
        if (gamepad1.aWasPressed()) {
            calPositionA = position;
            calibrate();
        }
        if (gamepad1.bWasPressed()) {
            calPositionB = position;
            calibrate();
        }

        // ---------- Telemetry ----------
        double minAngle = TurretPollen.positionToAngle(TurretPollen.minPosition);
        double maxAngle = TurretPollen.positionToAngle(TurretPollen.maxPosition);

        telemetry.addData("Mode (x)", mode);
        telemetry.addData("Commanded position", "%.4f", position);
        telemetry.addData("Commanded angle (deg)", "%.1f", turret.getCommandedAngle());
        telemetry.addData("Left / right servo", "%.4f / %.4f",
                turret.turretPollenLeft.getPosition(), turret.turretPollenRight.getPosition());
        if (mode == Mode.ANGLE && (targetAngle < Math.min(minAngle, maxAngle) || targetAngle > Math.max(minAngle, maxAngle))) {
            telemetry.addLine("Target angle is past the travel limits, clamped");
        }
        telemetry.addLine();

        telemetry.addLine("CALIBRATION:");
        telemetry.addData("Point A", Double.isNaN(calPositionA) ? "-" : String.format("%.4f @ %.1f deg", calPositionA, calAngleA));
        telemetry.addData("Point B", Double.isNaN(calPositionB) ? "-" : String.format("%.4f @ %.1f deg", calPositionB, calAngleB));
        telemetry.addData("Status", calStatus);
        telemetry.addData("Travel (deg)", "%.1f to %.1f", Math.min(minAngle, maxAngle), Math.max(minAngle, maxAngle));
        telemetry.addData("Turret deg per full servo range", "%.1f", Math.abs(TurretPollen.degreesPerPosition));
        if (turretDegreesPerServoDegree != 0) {
            telemetry.addData("Servo output range (deg)", "%.1f", Math.abs(TurretPollen.degreesPerPosition / turretDegreesPerServoDegree));
        }
        telemetry.addLine();

        telemetry.addLine("COPY INTO TurretPollen:");
        telemetry.addLine(String.format("centerPosition = %.4f;", TurretPollen.centerPosition));
        telemetry.addLine(String.format("degreesPerPosition = %.2f;", TurretPollen.degreesPerPosition));
        telemetry.addLine(String.format("minPosition = %.4f;", TurretPollen.minPosition));
        telemetry.addLine(String.format("maxPosition = %.4f;", TurretPollen.maxPosition));
        telemetry.addLine(String.format("leftTrim = %.4f; rightTrim = %.4f;", TurretPollen.leftTrim, TurretPollen.rightTrim));
        telemetry.addLine(String.format("pwmLowerUs = %.0f; pwmUpperUs = %.0f;", TurretPollen.pwmLowerUs, TurretPollen.pwmUpperUs));
        telemetry.update();
    }

    /** With both points recorded, solves angle = (position - centerPosition) * degreesPerPosition. */
    private void calibrate() {
        if (Double.isNaN(calPositionA) || Double.isNaN(calPositionB)) {
            calStatus = "record the other point";
            return;
        }
        if (Math.abs(calPositionB - calPositionA) < 1e-4 || calAngleA == calAngleB) {
            calStatus = "points are the same, move further apart";
            return;
        }
        TurretPollen.degreesPerPosition = (calAngleB - calAngleA) / (calPositionB - calPositionA);
        TurretPollen.centerPosition = calPositionA - calAngleA / TurretPollen.degreesPerPosition;
        calStatus = "applied";
    }

    private void rememberHardwareConfig() {
        appliedLeftReversed = TurretPollen.leftReversed;
        appliedRightReversed = TurretPollen.rightReversed;
        appliedPwmLower = TurretPollen.pwmLowerUs;
        appliedPwmUpper = TurretPollen.pwmUpperUs;
    }

    private void reapplyHardwareConfigIfChanged() {
        if (appliedLeftReversed != TurretPollen.leftReversed
                || appliedRightReversed != TurretPollen.rightReversed
                || appliedPwmLower != TurretPollen.pwmLowerUs
                || appliedPwmUpper != TurretPollen.pwmUpperUs) {
            turret.applyHardwareConfig();
            rememberHardwareConfig();
        }
    }
}
