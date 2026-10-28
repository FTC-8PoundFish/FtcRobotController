package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/*
 * Shared TeleOp for LimelightRed / LimelightBlue. Regular mecanum drive (same as
 * DriveTest). While right_bumper is held, the robot looks for this color's AprilTags and
 * rotates so the camera faces the center between the 2 middle tags of a group of 4
 * (see TagAimer). The driver can still drive and strafe with the left stick while aiming.
 */
public abstract class LimelightAimBase extends OpMode {

    private final boolean red;

    protected LimelightAimBase(boolean red) {
        this.red = red;
    }

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private TagAimer tagAimer;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);

        tagAimer = new TagAimer(hardwareMap, telemetry, red);

        telemetry.addData("Status", "Initialized (%s)", tagAimer.colorName());
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        telemetry.addData("Color", tagAimer.colorName());

        if (gamepad1.right_bumper) {
            turn = tagAimer.aimTurn(turn);
        } else {
            telemetry.addData("Aim", "hold right_bumper to aim");
        }

        double[] speeds = {
                (drive + strafe + turn),
                (drive - strafe - turn),
                (drive - strafe + turn),
                (drive + strafe - turn)
        };

        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

        telemetry.update();
    }

    @Override
    public void stop() {
        tagAimer.stop();
    }
}
