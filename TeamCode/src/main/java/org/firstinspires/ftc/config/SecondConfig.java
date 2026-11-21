package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Tunables for AutoShootRED / AutoShootBLUE (AutoShootBASE). Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "SecondConfig". Its launcher speeds are in
 * LauncherConfig (secondLauncherTargetVelocity, secondLauncherGap).
 *
 * Lives outside the teamcode package, like the other config classes.
 */
@Config
public class SecondConfig {
    // Whether to shoot during the SHOOT_TIME at shootPose. When false the robot still waits
    // there for the same time, but leaves the launcher off and then parks straight from
    // shootPose without going to the flower. Read when the robot reaches shootPose.
    public static boolean SECOND_SHOOT = true;

    // How long (ms) the robot stays at shootPose shooting before moving on. Read when the
    // robot reaches shootPose.
    public static int SHOOT_TIME = 4_000;
    // Second shot (at twiceTheFun): shoot for at least SECOND_SHOOT_TIME, and keep shooting
    // until only SECOND_PARK_RESERVE_MS of the 30s auto is left to drive to park. Read when
    // the second shot starts.
    public static int SECOND_SHOOT_TIME = 5_500;
    public static int SECOND_PARK_RESERVE_MS = 5_000;

    // If the AprilTag is seen within EARLY_TAG_MS (ms after start), there's time for the
    // two-shot route after the first shot: flower, to twiceTheFun, shoot again, park.
    // Seen later: flower, then park. Read after the first shot.
    public static int EARLY_TAG_MS = 11_000;

    // y (inches) of flowerPose, where the robot drives to pick up at the flower (x is 127).
    // Read when INIT is pressed.
    public static double SECOND_FLOWER_Y = 97;

    // At the flower, how long to wait after lowering the flower intake before the jiggle
    // starts, so it's fully down first. Read when the flower starts lowering.
    public static int SECOND_FLOWER_DOWN_MS = 500;

    // After reaching the flower, rock once to shake balls into the intake: drive forward
    // SECOND_JIGGLE_INCHES, stop there for SECOND_JIGGLE_STOP_MS, then drive back.
    // Read when the jiggle starts.
    public static double SECOND_JIGGLE_INCHES = 15;
    public static int SECOND_JIGGLE_STOP_MS = 3000;
    // Max time for each drive (forward or back), in case the robot can't quite reach the end.
    public static int SECOND_JIGGLE_HALF_MS = 400;
    // AutoShootBASE's own launcher speed and feed gap.
    private static final int DEFAULT_SECOND_LAUNCHER_TARGET_VELOCITY = 1155;
    private static final int DEFAULT_SECOND_LAUNCHER_GAP = 35;

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
