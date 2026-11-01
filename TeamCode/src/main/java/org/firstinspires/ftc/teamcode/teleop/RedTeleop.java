package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Competition teleop for the red alliance. Everything is in CompetitionTeleop. */
@TeleOp(name = "RedTeleop", group = "Competition")
public class RedTeleop extends CompetitionTeleop {
    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
