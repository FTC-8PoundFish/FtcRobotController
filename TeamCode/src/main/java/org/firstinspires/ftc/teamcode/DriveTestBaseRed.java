package org.firstinspires.ftc.teamcode;

/* DriveTestBase that aims at the red AprilTags. Subclasses pick the launcher target velocity. */
public abstract class DriveTestBaseRed extends DriveTestBase {
    protected DriveTestBaseRed(int launcherTargetVelocity) {
        super(launcherTargetVelocity, true);
    }
}
