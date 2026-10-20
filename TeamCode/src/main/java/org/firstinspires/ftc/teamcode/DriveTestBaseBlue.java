package org.firstinspires.ftc.teamcode;

/* DriveTestBase that aims at the blue AprilTags. Subclasses pick the launcher target velocity. */
public abstract class DriveTestBaseBlue extends DriveTestBase {
    protected DriveTestBaseBlue(int launcherTargetVelocity) {
        super(launcherTargetVelocity, false);
    }
}
