package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/*
 * Launcher speeds shared by DriveTest, AutoShootFirst and AutoShootSecond. Tune them live
 * from FTC Dashboard (http://192.168.43.1:8080/dash) under "LauncherConfig".
 *
 * The OpModes read these every time they use them, so a change on the dashboard takes
 * effect right away. The feed threshold always follows the target: target - gap.
 */
@Config
public class LauncherConfig {
    // Launcher speed to shoot at.
    public static int launcherTargetVelocity = 1225;
    // Feed only once the launcher is within this much of launcherTargetVelocity.
    public static int launcherGap = 25;

    public static int launcherMinVelocity() {
        return launcherTargetVelocity - launcherGap;
    }
}
