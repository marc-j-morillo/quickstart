package org.firstinspires.ftc.teamcode.pedro.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue copy of RedSolo2Flower: same routine, every pose mirrored (x -> fieldWidth - x, heading -> 180 - heading). Edit RedSolo2Flower. */
@Autonomous(name = "BlueSolo2Flower", group = "Autonomous")
public class BlueSolo2Flower extends RedSolo2Flower {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
