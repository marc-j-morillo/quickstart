package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.teleop.systems.Robot;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.HoodPollen;
import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.teleop.systems.utils.LoopTimer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Builds the shot table (Shooter.config().lutDistances / lutVelocities / lutHoods) live, on the real robot code.
 * Shooter rebuilds its LUTs whenever those arrays change, so every edit here (or in Panels, under Shooter)
 * takes effect on the next loop. The arrays are static, so they survive between opmode runs but NOT an app
 * restart: copy the "COPY INTO Shooter" lines into Shooter.java when you're done.
 *
 * Workflow: drive somewhere, shoot (real controls on gamepad1), adjust the velocity / hood trims on gamepad2
 * until the shots go in, then press a to save that distance as a point. The trims reset to 0 because the new
 * point now gives the same shot. Repeat at other distances, then drive between them to check the interpolation.
 *
 * Controls:
 *   gamepad1 ................. the real TeleOp controls (testCode), drive + intake + shoot
 *   gamepad2 dpad up / down .. velocity trim +/- velocityStep     bumpers: +/- velocityCoarseStep
 *   gamepad2 dpad right / left hood trim +/- hoodStep
 *   gamepad2 a ............... save a point at the current distance (overwrites one within mergeDistanceInches)
 *   gamepad2 b ............... delete the point nearest the current distance (within deleteDistanceInches)
 *   gamepad2 x ............... reset the trims
 *   gamepad2 y ............... write the table to the robot log (logcat tag "LutTuner")
 */
@TeleOp(name = "LutTuner", group = "Tuning")
@Configurable
public class LutTuner extends OpMode {
    public static boolean isRedAlliance = true;

    public static double velocityStep = 20;
    public static double velocityCoarseStep = 100;
    public static double hoodStep = 0.01;

    public static double mergeDistanceInches = 2;
    public static double deleteDistanceInches = 6;

    private Robot robot;
    private String lastAction = "-";
    private final LoopTimer loopTimer = new LoopTimer();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GlobalVars.inAuto = false;
        GlobalVars.isRed = isRedAlliance;
        // transportState is static, so it survives between opmode runs - start from a known state
        Robot.transportState = Robot.TransportState.HOME;
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        loopTimer.update();
        GlobalVars.isRed = isRedAlliance;

        // Edit before robot.update so this loop's shot uses the change
        if (gamepad2.dpadUpWasPressed()) Shooter.config().velocityTrim += velocityStep;
        if (gamepad2.dpadDownWasPressed()) Shooter.config().velocityTrim -= velocityStep;
        if (gamepad2.rightBumperWasPressed()) Shooter.config().velocityTrim += velocityCoarseStep;
        if (gamepad2.leftBumperWasPressed()) Shooter.config().velocityTrim -= velocityCoarseStep;
        if (gamepad2.dpadRightWasPressed()) Shooter.config().hoodTrim += hoodStep;
        if (gamepad2.dpadLeftWasPressed()) Shooter.config().hoodTrim -= hoodStep;
        if (gamepad2.xWasPressed()) resetTrims();
        if (gamepad2.aWasPressed()) savePoint();
        if (gamepad2.bWasPressed()) deleteNearestPoint();
        if (gamepad2.yWasPressed()) {
            for (String line : tableLines()) RobotLog.ii("LutTuner", line);
            lastAction = "table written to the robot log";
        }

        robot.update(gamepad1, gamepad2);

        // ---------- Telemetry ----------
        double distance = Shooter.distanceFromLimelightToTagInches;
        List<Integer> points = sortedPoints();

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Aiming at", Shooter.useTargetOverride ? "override point" : "goal");
        telemetry.addData("Distance (in)", "%.1f", distance);
        if (points.size() >= 2) {
            double min = Shooter.config().lutDistances[points.get(0)];
            double max = Shooter.config().lutDistances[points.get(points.size() - 1)];
            if (distance < min || distance > max) telemetry.addLine("Outside the table, using the nearest end");
        }
        telemetry.addLine();

        telemetry.addLine("SHOT:");
        telemetry.addData("LUT velocity + trim", "%.0f + %.0f", Shooter.lutVelocity, Shooter.config().velocityTrim);
        telemetry.addData("LUT hood + trim", "%.3f + %.3f", Shooter.lutHood, Shooter.config().hoodTrim);
        telemetry.addData("Flywheel velocity / target", "%.0f / %.0f", Shooter.shooterVelocity, Shooter.shooterVelocityTarget);
        telemetry.addData("Hood commanded", "%.3f", Shooter.hoodPollenPosition);
        telemetry.addData("Ready to fire", Math.abs(Shooter.shooterVelocity - Shooter.shooterVelocityTarget) < Shooter.fireTolerance);
        telemetry.addData("Transport state", Robot.transportState);
        telemetry.addData("Last action", lastAction);
        loopTimer.addTelemetry(telemetry);
        telemetry.addLine();

        telemetry.addLine("TABLE (distance: velocity, hood):");
        for (int i : points) {
            boolean near = Math.abs(Shooter.config().lutDistances[i] - distance) <= mergeDistanceInches;
            telemetry.addLine(String.format("%s%.1f: %.0f, %.3f", near ? "> " : "  ",
                    Shooter.config().lutDistances[i], Shooter.config().lutVelocities[i], Shooter.config().lutHoods[i]));
        }
        telemetry.addLine();

        telemetry.addLine("COPY INTO Shooter:");
        for (String line : tableLines()) telemetry.addLine(line);
        telemetry.update();
    }

    private void resetTrims() {
        Shooter.config().velocityTrim = 0;
        Shooter.config().hoodTrim = 0;
    }

    /** Saves the current shot (LUT + trims) as a point at the current distance. */
    private void savePoint() {
        double distance = Shooter.distanceFromLimelightToTagInches;
        double velocity = Shooter.lutVelocity + Shooter.config().velocityTrim;
        double hood = HoodPollen.clamp(Shooter.lutHood + Shooter.config().hoodTrim);
        if (distance <= 0) {
            lastAction = "distance is 0, not saved";
            return;
        }

        int index = nearestPoint(distance, mergeDistanceInches);
        if (index < 0) index = emptySlot();

        Shooter.config().lutDistances[index] = distance;
        Shooter.config().lutVelocities[index] = velocity;
        Shooter.config().lutHoods[index] = hood;
        resetTrims();
        lastAction = String.format("saved %.1f in: %.0f, %.3f", distance, velocity, hood);
    }

    private void deleteNearestPoint() {
        int index = nearestPoint(Shooter.distanceFromLimelightToTagInches, deleteDistanceInches);
        if (index < 0) {
            lastAction = "no point close enough to delete";
            return;
        }
        lastAction = String.format("deleted %.1f in", Shooter.config().lutDistances[index]);
        Shooter.config().lutDistances[index] = 0;
        Shooter.config().lutVelocities[index] = 0;
        Shooter.config().lutHoods[index] = 0;
    }

    /** Index of the used point closest to distance, or -1 if none is within maxGap. */
    private int nearestPoint(double distance, double maxGap) {
        int best = -1;
        for (int i = 0; i < pointSlots(); i++) {
            if (Shooter.config().lutDistances[i] <= 0) continue;
            double gap = Math.abs(Shooter.config().lutDistances[i] - distance);
            if (gap <= maxGap && (best < 0 || gap < Math.abs(Shooter.config().lutDistances[best] - distance))) best = i;
        }
        return best;
    }

    /** Index of an empty slot, growing the arrays if they're full (or not the same length). */
    private int emptySlot() {
        for (int i = 0; i < pointSlots(); i++) {
            if (Shooter.config().lutDistances[i] <= 0) return i;
        }
        int used = pointSlots();
        int size = Math.max(Shooter.config().lutDistances.length, Math.max(Shooter.config().lutVelocities.length, Shooter.config().lutHoods.length)) + 4;
        Shooter.config().lutDistances = Arrays.copyOf(Shooter.config().lutDistances, size);
        Shooter.config().lutVelocities = Arrays.copyOf(Shooter.config().lutVelocities, size);
        Shooter.config().lutHoods = Arrays.copyOf(Shooter.config().lutHoods, size);
        return used;
    }

    /** Slots that exist in all three arrays (Shooter ignores the rest). */
    private int pointSlots() {
        return Math.min(Shooter.config().lutDistances.length, Math.min(Shooter.config().lutVelocities.length, Shooter.config().lutHoods.length));
    }

    private List<Integer> sortedPoints() {
        List<Integer> points = new ArrayList<>();
        for (int i = 0; i < pointSlots(); i++) {
            if (Shooter.config().lutDistances[i] > 0) points.add(i);
        }
        points.sort((a, b) -> Double.compare(Shooter.config().lutDistances[a], Shooter.config().lutDistances[b]));
        return points;
    }

    /**
     * The table as Java array initializers, sorted by distance, in the same layout as Shooter:
     * distances on one line, then one velocity / hood per line with its distance as a comment.
     */
    private List<String> tableLines() {
        List<Integer> points = sortedPoints();
        StringBuilder distances = new StringBuilder();
        for (int i : points) {
            if (distances.length() > 0) distances.append(", ");
            distances.append(String.format("%.1f", Shooter.config().lutDistances[i]));
        }

        List<String> lines = new ArrayList<>();
        lines.add("public static double[] lutDistances  = {");
        lines.add("        " + distances);
        lines.add("};");
        lines.add("public static double[] lutVelocities = {");
        for (int i : points) lines.add(String.format("        %.0f, // %.1f in", Shooter.config().lutVelocities[i], Shooter.config().lutDistances[i]));
        lines.add("};");
        lines.add("public static double[] lutHoods      = {");
        for (int i : points) lines.add(String.format("        %.3f, // %.1f in", Shooter.config().lutHoods[i], Shooter.config().lutDistances[i]));
        lines.add("};");
        return lines;
    }
}
