package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Tunables for AutoShootSecond. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "SecondConfig". Its launcher speeds are in
 * LauncherConfig (secondLauncherTargetVelocity, secondLauncherGap).
 *
 * Lives outside the teamcode package, like the other config classes.
 */
@Config
public class SecondConfig {
    // Whether to shoot during the 10s at shootPose. When false the robot still waits there
    // for the same 10s (so the timing doesn't change), but leaves the launcher off.
    // Read when the robot reaches shootPose.
    public static boolean SECOND_SHOOT = true;
    // AutoShootSecond's own launcher speed and feed gap.
    private static final int DEFAULT_SECOND_LAUNCHER_TARGET_VELOCITY = 1187;
    private static final int DEFAULT_SECOND_LAUNCHER_GAP = 25;

    public static int secondLauncherTargetVelocity = DEFAULT_SECOND_LAUNCHER_TARGET_VELOCITY;
    public static int secondLauncherGap = DEFAULT_SECOND_LAUNCHER_GAP;

    public static int secondLauncherTargetVelocity() {
        return secondLauncherTargetVelocity != 0 ? secondLauncherTargetVelocity : DEFAULT_SECOND_LAUNCHER_TARGET_VELOCITY;
    }

    public static int secondLauncherGap() {
        return secondLauncherGap != 0 ? secondLauncherGap : DEFAULT_SECOND_LAUNCHER_GAP;
    }

    public static int secondLauncherMinVelocity() {
        return secondLauncherTargetVelocity() - secondLauncherGap();
    }

}
