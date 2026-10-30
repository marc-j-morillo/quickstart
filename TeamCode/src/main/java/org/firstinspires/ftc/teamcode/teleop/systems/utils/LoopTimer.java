package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Measures loop time. Call update() once per loop, then addTelemetry(). */
public class LoopTimer {
    private static final double SMOOTHING = 0.1;    // weight of the newest loop in the average
    private static final double PEAK_WINDOW_S = 1.0; // slowest loop is tracked over this window

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime peakTimer = new ElapsedTime();
    private boolean started = false;

    private double lastMs, averageMs, peakMs, peakInWindowMs;

    public void update() {
        double ms = loopTimer.milliseconds();
        loopTimer.reset();
        if (!started) { // first call only starts the clock
            started = true;
            peakTimer.reset();
            return;
        }

        lastMs = ms;
        averageMs = averageMs == 0 ? ms : averageMs + SMOOTHING * (ms - averageMs);

        peakInWindowMs = Math.max(peakInWindowMs, ms);
        if (peakTimer.seconds() > PEAK_WINDOW_S) {
            peakMs = peakInWindowMs;
            peakInWindowMs = 0;
            peakTimer.reset();
        }
    }

    public double getLastMs() { return lastMs; }
    public double getAverageMs() { return averageMs; }
    public double getPeakMs() { return peakMs; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Loop time (ms)", "%.1f avg, %.1f now, %.1f worst (1s)", averageMs, lastMs, peakMs);
        telemetry.addData("Loop rate (Hz)", "%.0f", averageMs > 0 ? 1000.0 / averageMs : 0);
    }
}
