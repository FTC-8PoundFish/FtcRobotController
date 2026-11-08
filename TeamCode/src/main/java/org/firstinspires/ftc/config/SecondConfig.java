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

    // After reaching the flower, rock back and forth to shake balls into the intake (like
    // the garden jiggle in AutoShootGarden). Each rock backs up SECOND_JIGGLE_INCHES and
    // returns. Read when the jiggle starts.
    public static double SECOND_JIGGLE_INCHES = 3;
    public static int SECOND_JIGGLE_TIMES = 3;
    // Max time for each half of a rock, in case the robot can't quite reach the end.
    public static int SECOND_JIGGLE_HALF_MS = 400;
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
