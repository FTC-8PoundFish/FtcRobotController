package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.MILLIS_IN_NANO;

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

    public static long ON_NANO = 240 * MILLIS_IN_NANO;
    public static long OFF_NANO = 440 * MILLIS_IN_NANO; //160

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
        double cycleNano = timer.nanoseconds() % (ON_NANO + OFF_NANO);
        if (cycleNano < ON_NANO) {
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
