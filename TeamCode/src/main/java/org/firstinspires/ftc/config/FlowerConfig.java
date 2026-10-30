package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Flower servo positions used by DriveTest. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "FlowerConfig".
 *
 * The OpMode reads these every time it uses them, so a change on the dashboard takes
 * effect right away.
 *
 * Read the values through the methods below, not the fields: if a field comes back as 0
 * (e.g. a cleared dashboard box), the method falls back to the default.
 */
@Config
public class FlowerConfig {
    private static final double DEFAULT_FLOWER_HOME = 0.5;
    private static final double DEFAULT_FLOWER_90_DEG = 0.44;
    private static final double DEFAULT_FLOWER_NECTAR_BLOCK = 0.36;
    private static final double DEFAULT_FLOWER_NUDGE_PER_SEC = 0.1;

    // Flower servos are mirrored: the left one is set to REVERSE, so sending the same
    // position to both turns one clockwise and the other counter-clockwise by the same angle.
    // FLOWER_HOME is the servo's center, leaving room to rotate either way.
    // FLOWER_90_DEG = 90 / (servo's full range in degrees): 0.3 for a 300-degree goBILDA
    // servo, 0.333 for a 270-degree REV Smart Servo, 0.5 for a 180-degree servo.
    public static double FLOWER_HOME = DEFAULT_FLOWER_HOME;
    public static double FLOWER_90_DEG = DEFAULT_FLOWER_90_DEG;
    public static double FLOWER_NECTAR_BLOCK = DEFAULT_FLOWER_NECTAR_BLOCK;

    // Holding dpad_left / dpad_right moves the flower this much per second, so the speed
    // doesn't depend on how fast loop() runs.
    public static double FLOWER_NUDGE_PER_SEC = DEFAULT_FLOWER_NUDGE_PER_SEC;

    public static double flowerHome() {
        return orDefault(FLOWER_HOME, DEFAULT_FLOWER_HOME);
    }

    public static double flower90Deg() {
        return orDefault(FLOWER_90_DEG, DEFAULT_FLOWER_90_DEG);
    }

    public static double flowerNectarBlock() {
        return orDefault(FLOWER_NECTAR_BLOCK, DEFAULT_FLOWER_NECTAR_BLOCK);
    }

    public static double flowerNudgePerSec() {
        return orDefault(FLOWER_NUDGE_PER_SEC, DEFAULT_FLOWER_NUDGE_PER_SEC);
    }

    private static double orDefault(double value, double fallback) {
        return value != 0.0 ? value : fallback;
    }
}
