package org.firstinspires.ftc.teamcode.NewGen;

import com.qualcomm.robotcore.hardware.CRServo;

/*
 * Feeding for the windmill CR servos that push balls into the launchers, shared by TeleOp and
 * Auto (Driving and the NewGen autos pass "nectarServo1" and "pollenServo2"). All the servos
 * always get the same power; set each servo's direction before passing it in if one is
 * mounted the other way round.
 *
 * Call feed() every loop() iteration while you want to feed balls into the launcher; the
 * servos spin forward continuously. Call stop() (or reverse()) whenever you're not feeding.
 */
public class WindmillFeeder {

    private final CRServo[] servos;

    public WindmillFeeder(CRServo... servos) {
        this.servos = servos;
    }

    public void feed() {
        setPower(1);
    }

    public void stop() {
        setPower(0);
    }

    /** Manual full reverse (e.g. driver clearing the hopper). */
    public void reverse() {
        setPower(-1);
    }

    private void setPower(double power) {
        for (CRServo servo : servos) {
            servo.setPower(power);
        }
    }
}
