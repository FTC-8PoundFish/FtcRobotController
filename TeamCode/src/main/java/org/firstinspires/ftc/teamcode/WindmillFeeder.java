package org.firstinspires.ftc.teamcode;

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
public class WindmillFeeder {

    public static long ON_MILLS = 240;
    public static long OFF_MILLS = 440; //160

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
        double cycleNano = timer.milliseconds() % (ON_MILLS + OFF_MILLS);
        if (cycleNano < ON_MILLS) {
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
