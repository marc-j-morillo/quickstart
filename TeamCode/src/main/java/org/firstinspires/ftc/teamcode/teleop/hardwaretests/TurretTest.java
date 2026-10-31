package org.firstinspires.ftc.teamcode.teleop.hardwaretests;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp(name = "TurretTest", group = "TeleOp")
public class TurretTest extends OpMode {

    //  ------------- TURRET -------------
    //TurretPollen turret;
    private double turretPosition;
    @IgnoreConfigurable public static Servo turretPollenLeft, turretPollenRight;

    public static int rotLoopCount = 0;
    public static double turretOffsets = 0;




    @Override
    public void init() {
        turretPollenLeft = hardwareMap.get(Servo.class, "turretPollenLeft");
        turretPollenRight = hardwareMap.get(Servo.class, "turretPollenRight");
        // turretPollenLeft.setDirection(Servo.Direction.REVERSE);
        // turretPollenRight.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        // 5 - right
        // 4 - left

        if (gamepad1.dpadLeftWasPressed()) {
            turretPosition += 0.1;
        } else if (gamepad1.dpadRightWasPressed()) {
            turretPosition -= 0.1;
        } else if (gamepad1.dpadDownWasPressed()) {
            turretPosition += 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            turretPosition -= 0.01;
        } else if (gamepad1.right_trigger > 0.1) {
            turretPosition += 0.01;
        } else if (gamepad1.left_trigger > 0.1) {
            turretPosition -= 0.01;
        }

        if (turretPosition > 1) {
            turretPosition = 1;
        } else if (turretPosition < 0) {
            turretPosition = 0;
        }

        turretPollenLeft.setPosition(turretPosition + turretOffsets);
        turretPollenRight.setPosition(turretPosition - turretOffsets);
        telemetry.addData("Turret position: ", turretPosition);

        telemetry.update();
    }
}
