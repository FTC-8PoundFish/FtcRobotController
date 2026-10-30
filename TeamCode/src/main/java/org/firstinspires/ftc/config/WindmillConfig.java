package org.firstinspires.ftc.config;

import com.acmerobotics.dashboard.config.Config;

/*
 * Windmill feed timings used by WindmillFeeder. Tune them live from FTC Dashboard
 * (http://192.168.43.1:8080/dash) under "WindmillConfig".
 *
 * Lives outside the teamcode package so Sloth doesn't load a second copy of it; otherwise
 * dashboard edits land in a copy the running OpMode never reads.
 *
 * Read the values through the methods below, not the fields: if a field comes back as 0
 * (e.g. a cleared dashboard box), the method falls back to the default.
 */
@Config
public class WindmillConfig {
    private static final long DEFAULT_ON_MILLS = 240;
    private static final long DEFAULT_OFF_MILLS = 500;

    // Each feed cycle spins the windmill forward ON_MILLS, then pauses OFF_MILLS.
    public static long ON_MILLS = DEFAULT_ON_MILLS;
    public static long OFF_MILLS = DEFAULT_OFF_MILLS;

    public static long onMillis() {
        return ON_MILLS != 0 ? ON_MILLS : DEFAULT_ON_MILLS;
    }

    public static long offMillis() {
        return OFF_MILLS != 0 ? OFF_MILLS : DEFAULT_OFF_MILLS;
    }
}
