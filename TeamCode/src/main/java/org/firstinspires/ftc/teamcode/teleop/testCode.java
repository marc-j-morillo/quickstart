package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionBotX;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionBotY;
import static org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars.transitionHeading;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Localization;
import org.firstinspires.ftc.teamcode.teleop.systems.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

import java.util.List;

@TeleOp(name = "testCode", group = "TeleOp")
public class testCode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drive drive;
    private Localization localization;
    private Transport transport;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        localization = new Localization(hardwareMap);
        //transport = new Transport(hardwareMap);

        // Set bulk caching mode for all Lynx modules
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }