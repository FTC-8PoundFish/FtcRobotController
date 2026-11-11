package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Drive tunables for DriveTest and RobotRocker. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "DriveConfig".
 *
 * Read the values through the methods below, not the fields: if a field comes back as 0
 * (e.g. a cleared dashboard box), the method falls back to the default.
 */
@Config
public class DriveConfig {
    private static final double DEFAULT_DRIVE_JIGGLE_POWER = 0.5;
    private static final double DEFAULT_DRIVE_JIGGLE_CYCLE_MS = 300;
    private static final double DEFAULT_DRIVE_JIGGLE_FORWARD_RATIO = 0.6;

    // Holding y with both sticks centered keeps knocking the robot into the flower base: each
    // DRIVE_JIGGLE_CYCLE_MS it drives forward at DRIVE_JIGGLE_POWER for
    // DRIVE_JIGGLE_FORWARD_RATIO of the cycle (0.6 = 60%), then back for the rest. Since it
    // goes forward longer than back, it creeps forward and stays pressed against the base.
    public static double DRIVE_JIGGLE_POWER = DEFAULT_DRIVE_JIGGLE_POWER;
    public static double DRIVE_JIGGLE_CYCLE_MS = DEFAULT_DRIVE_JIGGLE_CYCLE_MS;
    public static double DRIVE_JIGGLE_FORWARD_RATIO = DEFAULT_DRIVE_JIGGLE_FORWARD_RATIO;

    public static double driveJigglePower() {
        return orDefault(DRIVE_JIGGLE_POWER, DEFAULT_DRIVE_JIGGLE_POWER);
    }

    public static double driveJiggleCycleMs() {
        return orDefault(DRIVE_JIGGLE_CYCLE_MS, DEFAULT_DRIVE_JIGGLE_CYCLE_MS);
    }

    // Clamped to 0..1.
    public static double driveJiggleForwardRatio() {
        double ratio = orDefault(DRIVE_JIGGLE_FORWARD_RATIO, DEFAULT_DRIVE_JIGGLE_FORWARD_RATIO);
        return Math.max(0.0, Math.min(1.0, ratio));
    }

    private static double orDefault(double value, double fallback) {
        return value != 0.0 ? value : fallback;
    }
}
