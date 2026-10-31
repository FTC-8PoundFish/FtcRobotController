package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Tunables for AutoShootFirst. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "FirstConfig". Its forward distance and launcher
 * speeds are in LauncherConfig.
 *
 * Lives outside the teamcode package, like the other config classes.
 */
@Config
public class FirstConfig {
    // Where AutoShootFirst parks (inches). Read when INIT is pressed.
    public static double FIRST_PARK_X = 64;
    public static double FIRST_PARK_Y = 20;
}
