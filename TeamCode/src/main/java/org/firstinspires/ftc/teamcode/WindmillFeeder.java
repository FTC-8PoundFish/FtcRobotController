package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Pulsed feeding for the windmill servo, shared by TeleOp and Auto.
 *
 * Call feed() every loop() iteration while you want to feed balls into the launcher. Each
 * cycle spins forward ON_MS, backs off REVERSE_MS to loosen any ball wedged in the windmill,
 * then pauses OFF_MS. Non-blocking, so the rest of loop() keeps running.
 *
 * Call stop() (or reverse()) whenever you're not feeding; the next feed() then starts a
 * fresh cycle with a full forward spin.
 */
@Config
public class WindmillFeeder {

    private static final long DEFAULT_ON_MILLS = 240;
    private static final long DEFAULT_OFF_MILLS = 500;

    // Tunable on FTC Dashboard. Read through onMillis() / offMillis(), which fall back to
    // the default if a field comes back as 0 (e.g. a cleared dashboard box).
    public static long ON_MILLS = DEFAULT_ON_MILLS;
    public static long OFF_MILLS = DEFAULT_OFF_MILLS;

    private static long onMillis() {
        return ON_MILLS != 0 ? ON_MILLS : DEFAULT_ON_MILLS;
    }

    private static long offMillis() {
        return OFF_MILLS != 0 ? OFF_MILLS : DEFAULT_OFF_MILLS;
    }

    private final CRServo servo;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean feeding = false;

    public WindmillFeeder(CRServo servo) {
        this.servo = servo;
    }

    public void feed() {
        if (!feeding) {
            feeding = true;
            timer.reset();
        }
        long onMillis = onMillis();
        double cycleNano = timer.milliseconds() % (onMillis + offMillis());
        if (cycleNano < onMillis) {
            servo.setPower(1);
        } else {
            servo.setPower(0);
        }
    }

    public void stop() {
        feeding = false;
        servo.setPower(0);
    }

    /** Manual full reverse (e.g. driver clearing the hopper). */
    public void reverse() {
        feeding = false;
        servo.setPower(-1);
    }
}
