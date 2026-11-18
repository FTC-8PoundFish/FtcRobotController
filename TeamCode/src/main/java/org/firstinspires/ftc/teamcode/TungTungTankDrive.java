package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TripleTDrive", group = "OpMode")
public class TungTungTankDrive extends OpMode {

    // Drive motors
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;

    // Mechanism motors & servos
    private DcMotorEx intake, pollenFlywheel;
    private Servo pollenServo1, pollenServo2;

    @Override
    public void init() {
        // --- 1. HARDWARE MAPPING ---
        // Drive Motors
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        // Mechanisms
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        pollenFlywheel = hardwareMap.get(DcMotorEx.class, "pollenFlywheel");
        pollenServo1 = hardwareMap.get(Servo.class, "servo1");
        pollenServo2 = hardwareMap.get(Servo.class, "servo2");

        // --- 2. MOTOR DIRECTIONS ---
        // Mecanum drive directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Mechanism directions
        intake.setDirection(DcMotor.Direction.FORWARD);
        pollenFlywheel.setDirection(DcMotor.Direction.FORWARD);

        // --- 3. RUN MODES & ENCODERS ---
        // Configure pollenFlywheel to use its encoder for velocity regulation
        pollenFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- 4. ZERO POWER BEHAVIORS ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- 1. MECANUM DRIVE CONTROLS ---
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // --- 2. MECHANISM CONTROLS ---
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else {
            intake.setPower(-1);
        }

        // --- TELEMETRY FEEDBACK ---
        telemetry.addData("Motors", "FL (%.2f), BL (%.2f), FR (%.2f), BR (%.2f)",
                frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        telemetry.update();
    }
}