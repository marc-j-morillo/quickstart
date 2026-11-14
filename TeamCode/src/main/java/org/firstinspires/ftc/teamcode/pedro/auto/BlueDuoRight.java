package org.firstinspires.ftc.teamcode.pedro.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue copy of RedDuoRight: same routine, every pose mirrored (x -> fieldWidth - x, heading -> 180 - heading). Edit RedDuoRight. */
@Autonomous(name = "BlueDuoRight", group = "Autonomous")
public class BlueDuoRight extends RedDuoRight {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
