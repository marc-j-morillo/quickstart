package org.firstinspires.ftc.teamcode.teleop.hardwaretests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.HoodPollen;

/**
 * Sets the hood servo from Panels: change hoodPosition (Panels, under HoodPanelsTest) and the hood moves to it.
 * The hood only moves after START. Positions are clamped to HoodPollen.hoodMin / hoodMax (0 - 0.81).
 */
@Configurable
@TeleOp(name = "HoodPanelsTest", group = "TeleOp")
public class HoodPanelsTest extends OpMode {
    public static double hoodPosition = 0; // servo position, 0 - 1 (clamped to HoodPollen's limits)

    private HoodPollen hood;

    @Override
    public void init() {
        hood = new HoodPollen(hardwareMap);
    }

    @Override
    public void loop() {
        hood.update(hoodPosition);

        telemetry.addData("Hood position (Panels)", hoodPosition);
        telemetry.addData("Hood position sent (clamped)", HoodPollen.clamp(hoodPosition));
        telemetry.addData("Limits", "%.2f - %.2f", HoodPollen.hoodMin, HoodPollen.hoodMax);
        telemetry.update();
    }
}
