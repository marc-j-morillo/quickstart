package org.firstinspires.ftc.teamcode.teleop.hardwaretests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Hold a button to spin the shooter motors at low power.
 *   dpad left  = shooter1 only
 *   dpad right = shooter2 only
 *   dpad up    = both
 *
 * Directions match Flywheel (both REVERSE). Each motor alone should spin the flywheel the shooting way.
 * If one spins it backward with the same code direction, that motor's power wires are swapped.
 * Both velocities are shown, so you can also see which motor has its encoder plugged in.
 */
@TeleOp(name = "ShooterMotorTest", group = "TeleOp")
public class ShooterMotorTest extends OpMode {
    private static final double POWER = 0.3;

    private DcMotorEx shooter1, shooter2;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Same directions as Flywheel (setInverted(true) = REVERSE)
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        boolean run1 = gamepad1.dpad_left || gamepad1.dpad_up;
        boolean run2 = gamepad1.dpad_right || gamepad1.dpad_up;
        shooter1.setPower(run1 ? POWER : 0);
        shooter2.setPower(run2 ? POWER : 0);

        telemetry.addLine("dpad left = shooter1, dpad right = shooter2, dpad up = both");
        telemetry.addData("shooter1 on / velocity", "%b / %.0f", run1, shooter1.getVelocity());
        telemetry.addData("shooter2 on / velocity", "%b / %.0f", run2, shooter2.getVelocity());
        telemetry.update();
    }
}
