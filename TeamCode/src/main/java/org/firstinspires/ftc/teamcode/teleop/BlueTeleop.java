package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Competition teleop for the blue alliance. Everything is in CompetitionTeleop. */
@TeleOp(name = "BlueTeleop", group = "Competition")
public class BlueTeleop extends CompetitionTeleop {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
