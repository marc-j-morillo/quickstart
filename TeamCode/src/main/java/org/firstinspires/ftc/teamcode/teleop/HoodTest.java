package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HoodTest", group = "TeleOp")
public class HoodTest extends OpMode {

    //  ------------- TURRET -------------
    //TurretPollen turret;
    public static Servo hoodPollen;
    public static double hoodPosition = 0;




    @Override
    public void init() {
        hoodPollen = hardwareMap.get(Servo.class, "hoodPollen");
        // hoodPollen.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start() {
        hoodPollen.setPosition(hoodPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed()) {
            hoodPosition -= 0.1;
        } else if (gamepad1.dpadRightWasPressed()) {
            hoodPosition += 0.1;
        } else if (gamepad1.dpadDownWasPressed()) {
            hoodPosition -= 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            hoodPosition += 0.01;
        }

        if (hoodPosition < 0){
            hoodPosition = 0;
        }

        hoodPollen.setPosition(hoodPosition);
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("right - increase by 0.1");
        telemetry.addLine("left - decrease by 0.1");
        telemetry.addLine("up - increase by 0.01");
        telemetry.addLine("down - decrease by 0.01");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Hood position: ", hoodPosition);

        telemetry.update();
    }
}
