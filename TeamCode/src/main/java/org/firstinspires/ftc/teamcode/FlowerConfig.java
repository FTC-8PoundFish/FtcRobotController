package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/*
 * Flower servo positions used by DriveTest. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "FlowerConfig".
 *
 * The OpMode reads these every time it uses them, so a change on the dashboard takes
 * effect right away.
 */
@Config
public class FlowerConfig {
    // Flower servos are mirrored: the left one is set to REVERSE, so sending the same
    // position to both turns one clockwise and the other counter-clockwise by the same angle.
    // FLOWER_HOME is the servo's center, leaving room to rotate either way.
    // FLOWER_90_DEG = 90 / (servo's full range in degrees): 0.3 for a 300-degree goBILDA
    // servo, 0.333 for a 270-degree REV Smart Servo, 0.5 for a 180-degree servo.
    public static double FLOWER_HOME = 0.5;
    public static double FLOWER_90_DEG = 0.44;
    public static double FLOWER_NECTAR_BLOCK = 0.36;

    // Holding dpad_left / dpad_right moves the flower this much per second, so the speed
    // doesn't depend on how fast loop() runs.
    public static double FLOWER_NUDGE_PER_SEC = 0.1;
}
