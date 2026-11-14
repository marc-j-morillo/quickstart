package org.firstinspires.ftc.teamcode.pedro.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue copy of ShootPreloadRed: same routine, every pose mirrored (x -> fieldWidth - x, heading -> 180 - heading). Edit ShootPreloadRed. */
@Autonomous(name = "ShootPreloadBlue", group = "Autonomous")
public class ShootPreloadBlue extends ShootPreloadRed {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
