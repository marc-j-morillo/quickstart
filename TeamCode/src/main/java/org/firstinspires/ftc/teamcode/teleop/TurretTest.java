package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TurretTest", group = "TeleOp")
public class TurretTest extends OpMode {

    //  ------------- TURRET -------------
    //TurretPollen turret;
    private double turretPosition;
    public static Servo turretPollenLeft, turretPollenRight;




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
        }

        if (turretPosition > 0.98) {
            turretPosition = 0.98;
        } else if (turretPosition < 0.02) {
            turretPosition = 0.02;
        }

        turretPollenLeft.setPosition(turretPosition + 0.02);
        turretPollenRight.setPosition(turretPosition - 0.02);
        telemetry.addData("Turret position: ", turretPosition);

        telemetry.update();
    }
}
