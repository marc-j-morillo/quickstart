package org.firstinspires.ftc.teamcode.teleop.systems.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.systems.Robot;

/**
 * Battery voltage for feedforward scaling. The voltage sensor isn't in the bulk read (each read is its own
 * ~1-2 ms command) and battery voltage changes slowly, so it's only read every Robot.voltageReadIntervalMs.
 */
public class BatteryVoltage {
    private final VoltageSensor sensor;
    private final ElapsedTime timer = new ElapsedTime();
    private double voltage;

    public BatteryVoltage(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        read();
    }

    /** Battery volts, re-read at most every Robot.voltageReadIntervalMs. */
    public double volts() {
        if (timer.milliseconds() >= Robot.voltageReadIntervalMs) read();
        return voltage;
    }

    /** Robot.nominalVoltage / battery volts: multiply feedforward terms by this so they act the same as the battery drains. */
    public double multiplier() {
        return Robot.nominalVoltage / volts();
    }

    private void read() {
        voltage = sensor.getVoltage();
        timer.reset();
    }
}
