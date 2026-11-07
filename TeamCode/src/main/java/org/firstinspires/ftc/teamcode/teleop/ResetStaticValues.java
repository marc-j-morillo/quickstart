package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.systems.utils.StaticDefaults;

import java.util.List;

/**
 * Puts every static field in TeamCode back to the value it had when the Robot Controller app started,
 * as soon as you press INIT. Nothing to do after that: stop the opmode.
 *
 * Resets everything, including Panels edits (trims, LUT points, tolerances, goal points, ...),
 * Robot.transportState, the goal side, and FollowerPose.startPose (so a teleop run after this won't get the
 * last auto's end pose). Same as restarting the Robot Controller app, without the restart.
 * See StaticDefaults for how it works.
 */
@TeleOp(name = "Reset Static Values", group = "Utility")
public class ResetStaticValues extends OpMode {
    private static final int MAX_LISTED = 25; // the Driver Station screen only fits so many lines

    @Override
    public void init() {
        if (!StaticDefaults.hasSnapshot()) {
            telemetry.addLine("NOTHING RESET: the starting values weren't recorded when the app started.");
            telemetry.addLine("Restart the Robot Controller app instead, and check the robot log for StaticDefaults.");
            telemetry.update();
            return;
        }

        List<String> changed = StaticDefaults.resetAll();

        telemetry.addLine("Reset " + StaticDefaults.fieldCount() + " static fields to their starting values.");
        if (StaticDefaults.classesSkipped() > 0) {
            telemetry.addLine(StaticDefaults.classesSkipped() + " classes couldn't be loaded and were skipped (see the robot log).");
        }
        telemetry.addLine();

        if (changed.isEmpty()) {
            telemetry.addLine("Nothing had changed.");
        } else {
            telemetry.addLine("Changed back (" + changed.size() + "):");
            for (int i = 0; i < Math.min(changed.size(), MAX_LISTED); i++) telemetry.addLine("  " + changed.get(i));
            if (changed.size() > MAX_LISTED) telemetry.addLine("  ... and " + (changed.size() - MAX_LISTED) + " more");
        }
        telemetry.addLine();
        telemetry.addLine("Done, you can stop this opmode.");
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
