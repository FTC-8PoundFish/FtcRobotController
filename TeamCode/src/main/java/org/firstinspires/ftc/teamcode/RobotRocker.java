package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.config.DriveConfig.driveJiggleCycleMs;
import static org.firstinspires.ftc.config.DriveConfig.driveJiggleForwardRatio;
import static org.firstinspires.ftc.config.DriveConfig.driveJigglePower;

/*
 * Rocks the whole robot forward and back (e.g. to shake balls into the intake), shared by
 * TeleOp and Auto. Power and timing come from DriveConfig (tunable on FTC Dashboard).
 *
 * Call drive() every loop() iteration. While rocking is true it returns a forward drive
 * power for the first DRIVE_JIGGLE_FORWARD_RATIO of each DRIVE_JIGGLE_CYCLE_MS and a
 * backward one for the rest, starting forward; otherwise it returns 0.
 * Feed the result into the mecanum formula as the "drive" input. It doesn't touch the
 * motors itself, so it works with plain motors or a Pedro follower.
 */
public class RobotRocker {

    // When the current rock started (ms), or -1 when not rocking. Starting from the first
    // rocking call means every rock begins with a forward half.
    private long startMs = -1;

    public double drive(boolean rocking) {
        if (!rocking) {
            startMs = -1;
            return 0;
        }
        long now = System.currentTimeMillis();
        if (startMs < 0) {
            startMs = now;
        }
        double cycleMs = driveJiggleCycleMs();
        double intoCycle = (now - startMs) % cycleMs;
        boolean forward = intoCycle < cycleMs * driveJiggleForwardRatio();
        return forward ? driveJigglePower() : -driveJigglePower();
    }
}
