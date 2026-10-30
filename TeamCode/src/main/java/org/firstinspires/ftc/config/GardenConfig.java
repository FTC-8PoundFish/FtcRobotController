package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Tunables for AutoShootGarden. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "GardenConfig".
 *
 * Lives outside the teamcode package so Sloth doesn't load a second copy of it; otherwise
 * dashboard edits land in a copy the running OpMode never reads.
 */
@Config
public class GardenConfig {
    // Heading (degrees) at shootPose. Read when PLAY is pressed.
    public static int SHOOT_HEADING = 0;

    // After driving into the garden, rock back and forth to shake balls into the intake
    // (like holding x in DriveTest). Each rock backs up GARDEN_JIGGLE_INCHES and returns.
    public static double GARDEN_JIGGLE_INCHES = 3;
    public static int GARDEN_JIGGLE_TIMES = 3;
    // Max time for each half of a rock, in case the robot can't quite reach the end.
    public static int GARDEN_JIGGLE_HALF_MS = 400;
    // gardenPose is past the wall, so the robot may never "finish" toGarden; start the
    // jiggle after this long regardless.
    public static int GARDEN_DRIVE_MAX_MS = 2500;
}
