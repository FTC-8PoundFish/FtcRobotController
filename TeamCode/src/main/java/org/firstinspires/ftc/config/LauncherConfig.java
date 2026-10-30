package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Launcher speeds shared by DriveTest, AutoShootFirst and AutoShootSecond. Tune them live
 * from FTC Dashboard (http://192.168.43.1:8080/dash) under "LauncherConfig".
 *
 * The OpModes read these every time they use them, so a change on the dashboard takes
 * effect right away. The feed threshold always follows the target: target - gap.
 *
 * Read the values through the methods below, not the fields: if a field comes back as 0
 * (e.g. a cleared dashboard box), the method falls back to the default.
 */
@Config
public class LauncherConfig {
    private static final int DEFAULT_LAUNCHER_TARGET_VELOCITY = 1175;
    private static final int DEFAULT_LAUNCHER_GAP = 25;

    private static final int DEFAULT_AUTO_LAUNCHER_TARGET_VELOCITY = 1187;
    private static final int DEFAULT_AUTO_LAUNCHER_GAP = 25;

    // Launcher speed to shoot at.
    public static int launcherTargetVelocity = DEFAULT_LAUNCHER_TARGET_VELOCITY;
    // Feed only once the launcher is within this much of launcherTargetVelocity.
    public static int launcherGap = DEFAULT_LAUNCHER_GAP;

    public static int autoLauncherTargetVelocity = DEFAULT_AUTO_LAUNCHER_TARGET_VELOCITY;
    // Feed only once the launcher is within this much of launcherTargetVelocity.
    public static int autoLauncherGap = DEFAULT_AUTO_LAUNCHER_GAP;

    public static int launcherTargetVelocity() {
        return launcherTargetVelocity != 0 ? launcherTargetVelocity : DEFAULT_LAUNCHER_TARGET_VELOCITY;
    }

    public static int launcherGap() {
        return launcherGap != 0 ? launcherGap : DEFAULT_LAUNCHER_GAP;
    }

    public static int launcherMinVelocity() {
        return launcherTargetVelocity() - launcherGap();
    }

    public static int autoLauncherTargetVelocity() {
        return autoLauncherTargetVelocity != 0 ? autoLauncherTargetVelocity : DEFAULT_AUTO_LAUNCHER_TARGET_VELOCITY;
    }

    public static int autoLauncherGap() {
        return autoLauncherGap != 0 ? autoLauncherGap : DEFAULT_AUTO_LAUNCHER_GAP;
    }

    public static int autoLauncherMinVelocity() {
        return autoLauncherTargetVelocity() - autoLauncherGap();
    }
}
