package org.firstinspires.ftc.teamcode.pedro.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Blue copy of ShootPreload_Left: same routine, every pose mirrored (x -> fieldWidth - x, heading -> 180 - heading). Edit ShootPreload_Left. */
@Autonomous(name = "ShootPreload_Left_Blue", group = "Autonomous")
public class ShootPreload_Left_Blue extends ShootPreload_Left {
    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}
