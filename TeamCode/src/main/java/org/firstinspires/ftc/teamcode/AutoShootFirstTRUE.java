package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.config.GardenConfig.GARDEN_AROUND_Y;

/*
 * Garden auto (see GardenAutoBase), then park right away at 14s instead of going to the
 * second pose to shoot again.
 */
@Autonomous(name = "SingleShot", group = "OpMode")
public class AutoShootFirstTRUE extends GardenAutoBase {
    private boolean kaboom = false;

    // Straight from the garden to park would cut through the middle garden, so first drive
    // to (x - 30, GARDEN_AROUND_Y), then over to park.
    private Command gardenToPark() {
        Pose here = follower.pose();
        Pose from = p.of(here.x(), here.y(), Math.toDegrees(here.heading()));
        Pose around = p.of(here.x() - 30, GARDEN_AROUND_Y, 0);
        return sequential(
                follow(follower, line(from, around).linear(from, around)),
                follow(follower, line(around, park).linear(around, park)));
    }

    @Override
    protected void afterGarden(long elapsed) {
        if (!kaboom && elapsed > 14_000) {
            kaboom = true;
            phase = "parking";
            switchTo(gardenToPark());
        }
    }
}
