package org.firstinspires.ftc.teamcode.pedro.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue copy of ShootPreloadRed_Reset: same routine, every pose mirrored (x -> fieldWidth - x, heading -> 180 - heading). Edit ShootPreloadRed_Reset. */
@Autonomous(name = "ShootPreloadBlue_Reset", group = "Autonomous")
public class ShootPreloadBlue_Reset extends ShootPreloadRed_Reset {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
