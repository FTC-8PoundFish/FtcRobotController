package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Drive tunables for DriveTest. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "DriveConfig".
 *
 * Read the values through the methods below, not the fields: if a field comes back as 0
 * (e.g. a cleared dashboard box), the method falls back to the default.
 */
@Config
public class DriveConfig {
    private static final double DEFAULT_DRIVE_JIGGLE_POWER = 0.3;
    private static final double DEFAULT_DRIVE_JIGGLE_HALF_MS = 100;

    // Holding y with both sticks centered rocks the whole robot forward and back: forward at
    // DRIVE_JIGGLE_POWER for DRIVE_JIGGLE_HALF_MS, then back for the same time, repeating.
    // Raise either one for a bigger rock.
    public static double DRIVE_JIGGLE_POWER = DEFAULT_DRIVE_JIGGLE_POWER;
    public static double DRIVE_JIGGLE_HALF_MS = DEFAULT_DRIVE_JIGGLE_HALF_MS;

    public static double driveJigglePower() {
        return orDefault(DRIVE_JIGGLE_POWER, DEFAULT_DRIVE_JIGGLE_POWER);
    }

    public static double driveJiggleHalfMs() {
        return orDefault(DRIVE_JIGGLE_HALF_MS, DEFAULT_DRIVE_JIGGLE_HALF_MS);
    }

    private static double orDefault(double value, double fallback) {
        return value != 0.0 ? value : fallback;
    }
}
