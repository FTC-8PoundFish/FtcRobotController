package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.config.LauncherConfig.autoLauncherMinVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.autoLauncherTargetVelocity;

/*
 * Garden auto (see GardenAutoBase), then:
 *   14s     drive from the garden to the second pose
 *   18-26s  shoot again
 *   26s     park
 */
@Autonomous(name = "AutoShootGarden", group = "OpMode")
public class AutoShootGarden extends GardenAutoBase {
    private final Pose seccondPose = p.of(32,34,216);

    private boolean boom = false;
    private boolean kaboom = false;

    private Path GardenToSecond() {
        return line(gardenPose, seccondPose).linear(gardenPose, seccondPose);
    }
    private Path park() {
        return line(seccondPose, park).linear(seccondPose, park);
    }

    @Override
    protected void afterGarden(long elapsed) {
        if (!boom && elapsed > 14_000) {
            boom = true;
            phase = "garden to seccondPose";
            launcher.setVelocity(autoLauncherTargetVelocity());
            switchTo(follow(follower, GardenToSecond()));
        }
        if (elapsed > 18_000 && elapsed < 26_000) {
            launcher.setVelocity(autoLauncherTargetVelocity());
            if (launcher.getVelocity() >= autoLauncherMinVelocity()) {
                windmillFeeder.feed();
                neoWindmill.setPower(1);
                intakePower += 0.5;
            } else {
                windmillFeeder.stop();
            }
        }
        if (!kaboom && elapsed > 26_000) {
            kaboom = true;
            phase = "parking";
            switchTo(follow(follower, park()));
        }
    }
}
