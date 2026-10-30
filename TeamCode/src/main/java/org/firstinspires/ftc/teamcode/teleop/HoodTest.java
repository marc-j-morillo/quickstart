package org.firstinspires.ftc.teamcode.teleop;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.systems.Shooter.HoodPollen;

@TeleOp(name = "HoodTest", group = "TeleOp")
@Configurable
public class HoodTest extends OpMode {

    //  ------------- HOOD -------------
    //TurretPollen turret;
    HoodPollen hood;

    public static double hoodPosition = 0;

    public static double hoodMin = 0;

    public static double hoodMax = 0.3;




    @Override
    public void init() {
        hood = new HoodPollen(hardwareMap);
    }

    @Override
    public void start() {

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

        if (hoodPosition < hoodMin){
            hoodPosition = hoodMin;
        } else if (hoodPosition > hoodMax) {
            hoodPosition = hoodMax;
        }

        hood.update(hoodPosition);
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
