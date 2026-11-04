package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Feeding for the windmill motor ("NEO"), shared by TeleOp and Auto.
 *
 * Call feed() every loop() iteration while you want to feed balls into the launcher; the
 * motor spins forward continuously. Call stop() (or reverse()) whenever you're not feeding.
 */
public class WindmillFeeder {

    private final DcMotor motor;

    public WindmillFeeder(DcMotor motor) {
        this.motor = motor;
    }

    public void feed() {
        motor.setPower(1);
    }

    public void stop() {
        motor.setPower(0);
    }

    /** Manual full reverse (e.g. driver clearing the hopper). */
    public void reverse() {
        motor.setPower(-1);
    }
}
