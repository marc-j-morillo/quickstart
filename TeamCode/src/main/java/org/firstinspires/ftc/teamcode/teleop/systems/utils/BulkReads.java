package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Bulk reads in MANUAL mode: each hub reads all its motor encoders / velocities / digital + analog inputs in
 * ONE command the first time anything is read, and every other read that loop comes from that cache.
 * Call clear() once at the very start of every loop, before any hardware read, or the values go stale.
 *
 * Not covered by bulk reads (each is its own slow command, keep them out of the loop or throttle them):
 * motor current (getCurrent), the voltage sensor, I2C devices (Pinpoint, color sensors, IMU).
 */
public final class BulkReads {
    private BulkReads() {}

    /** Puts every hub in MANUAL bulk caching. Returns the hubs to pass to clear(). */
    public static List<LynxModule> enableManual(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        return hubs;
    }

    /** Throws away last loop's cached reads. Call once at the start of every loop. */
    public static void clear(List<LynxModule> hubs) {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
