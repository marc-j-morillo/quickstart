package org.firstinspires.ftc.teamcode.pedro.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue copy of RedDuoLeft: same routine, every pose mirrored (x -> fieldWidth - x, heading -> 180 - heading). Edit RedDuoLeft. */
@Autonomous(name = "BlueDuoLeft", group = "Autonomous")
public class BlueDuoLeft extends RedDuoLeft {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
