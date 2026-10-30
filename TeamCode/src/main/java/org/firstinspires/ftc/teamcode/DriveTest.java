/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.config.FlowerConfig.flower90Deg;
import static org.firstinspires.ftc.config.FlowerConfig.flowerHome;
import static org.firstinspires.ftc.config.FlowerConfig.flowerNectarBlock;
import static org.firstinspires.ftc.config.FlowerConfig.flowerNudgePerSec;
import static org.firstinspires.ftc.config.LauncherConfig.launcherMinVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.launcherTargetVelocity;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.config.LauncherConfig;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * Launcher speeds come from LauncherConfig (tunable on FTC Dashboard).
 *
 * Limelight aiming (hold x to auto-rotate toward a color's AprilTags, see TagAimer) is
 * commented out for now. To bring it back, pass the alliance color (red = true/false)
 * into TagAimer.
 */
@TeleOp(name = "DriveTest", group = "OpMode")
public class DriveTest extends OpMode {

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    // Flower positions come from FlowerConfig (tunable on FTC Dashboard).
    double flowerPos;
    private double lastLoopTime = -1;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private Servo rightFlowerIntakeServo = null;
    private Servo leftflowerIntakeServo = null;
    private CRServo windmillServo = null;
    boolean stopIntake = false;
    private long timeNow;
    // When right_bumper started being held, or -1 when it isn't held.
    private long rightBumperHeldSince = -1;

    // Stick values smaller than this are treated as 0, so a stick that doesn't rest
    // exactly at center won't creep the robot.
    private static final double STICK_DEADBAND = 0.05;
    // Holding x with the sticks centered rocks the robot forward and back (e.g. to
    // settle artifacts in the hopper). Each half-cycle drives JIGGLE_HALF_MS at
    // JIGGLE_POWER, forward first, so it ends up roughly where it started.
    private static final double JIGGLE_POWER = 0.3;
    private static final long JIGGLE_HALF_MS = 150;
    // When x started being held for jiggling, or -1 when it isn't.
    private long jiggleSince = -1;
    private boolean wasJiggling = false;
    // True after left_trigger turns the intake on, until right_trigger is used.
    private boolean intakeLatchedOn = false;

    private GoBildaPinpointDriver odo;

    private WindmillFeeder windmillFeeder;
    private Follower follower;


    @Override
    public void start(){
    }


    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        intake.setZeroPowerBehavior(BRAKE);
        rightFlowerIntakeServo = hardwareMap.get(Servo.class, "flowery");
        leftflowerIntakeServo = hardwareMap.get(Servo.class, "flowey");
        rightFlowerIntakeServo.setDirection(Servo.Direction.REVERSE);
        leftflowerIntakeServo.setDirection(Servo.Direction.FORWARD);
        // Read flowerHome() here rather than at construction so a dashboard change applies
        // the next time this OpMode is initialized.
        flowerPos = flowerHome();
        rightFlowerIntakeServo.setPosition(flowerPos);
        leftflowerIntakeServo.setPosition(flowerPos);
//        follower = Constants.create(hardwareMap);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setPower(0);

        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillFeeder = new WindmillFeeder(windmillServo);
//        tagAimer = new TagAimer(hardwareMap, telemetry, red);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);


        odo.setOffsets(
                0,
                0,
                DistanceUnit.INCH
        );
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        telemetry.addData(
                "Speed",
                "target (%d), min (%d) flower (%.2f)",
                launcherTargetVelocity(),
                launcherMinVelocity(),
                flowerPos);
        telemetry.update();

    }

    // init() runs only once, so show the speed here too; a dashboard change made after
    // pressing INIT then shows up right away.
    @Override
    public void init_loop() {
        telemetry.addData(
                "Speed",
                "target (%d), min (%d) flower (%.2f)",
                launcherTargetVelocity(),
                launcherMinVelocity(),
                flowerPos);
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x);
        double turn = deadband(gamepad1.right_stick_x);
        double[] speeds = {
                (drive + strafe + turn),
                (drive - strafe - turn),
                (drive - strafe + turn),
                (drive + strafe - turn)
        };
        launch();
        if (gamepad1.right_trigger_pressed){
            intake.setPower(-1);
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(-1);
            stopIntake = true;
        }
        if (stopIntake&&!gamepad1.right_trigger_pressed){
            leftIntakeServo.setPower(0);
            rightIntakeServo.setPower(0);
            intake.setPower(0);
        }
        if (gamepad1.left_trigger_pressed){
            stopIntake = false;
            intake.setPower(1);
            leftIntakeServo.setPower(1);
            rightIntakeServo.setPower(1);
            intakeLatchedOn = true;
        }
        if (gamepad1.right_trigger_pressed) {
            intakeLatchedOn = false;
        }
        // dpad_down = home + 90 degrees, dpad_up = home (mirrored on each side).
        // Holding dpad_left / dpad_right moves it slowly and continuously.
        double now = getRuntime();
        double dt = lastLoopTime < 0 ? 0 : now - lastLoopTime;
        lastLoopTime = now;
        if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.x) {
            if (gamepad1.dpad_down){
                flowerPos = flowerHome() + flower90Deg();
            } else if (gamepad1.dpad_up){
                flowerPos = flowerHome();
            } else if (gamepad1.dpad_left){
                flowerPos = flowerPos + flowerNudgePerSec() * dt;
            } else if (gamepad1.dpad_right){
                flowerPos = flowerPos - flowerNudgePerSec() * dt;
            } else if (gamepad1.x) {
                flowerPos = flowerHome() + flowerNectarBlock();
            }
            flowerPos = Math.max(0.0, Math.min(1.0, flowerPos));
            rightFlowerIntakeServo.setPosition(flowerPos);
            leftflowerIntakeServo.setPosition(flowerPos);
        }



        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }

        // If and only if the maximum is outside the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);
//        telemetry.addData(
//                "Speed",
//                "target (%d), min (%d) flower (%f)",
//                launcherTargetVelocity(),
//                launcherMinVelocity(),
//                rightFlowerIntakeServo.getPosition());
        telemetry.addData(
                "Motors",
                "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)",
                speeds[0],
                speeds[1],
                speeds[2],
                speeds[3]);
        odo.update();
        Pose2D position = odo.getPosition();
        telemetry.addData(
                "Odo",
                "x (%f): y (%f) left (%f) right (%f)",
                position.getX(DistanceUnit.INCH),
                position.getY(DistanceUnit.INCH),
                leftflowerIntakeServo.getPosition(),
                rightFlowerIntakeServo.getPosition()
        );
        // Raw dashboard field vs. what the code actually uses (they differ only if the
        // field is 0 and the default kicks in).
        telemetry.addData(
                "Target velocity",
                "field (%d), method (%d)",
                LauncherConfig.launcherTargetVelocity,
                launcherTargetVelocity());
        telemetry.update();
    }


//    @Override
//    public void stop() {
//        tagAimer.stop();
//    }

    private static double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }

    void launch() {
        /*
         * Calling gamepad1.right_bumper returns a boolean whicfh will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        if (gamepad1.right_bumper) {
            if (rightBumperHeldSince < 0) {
                rightBumperHeldSince = System.currentTimeMillis();
            }
            launcher.setVelocity(launcherTargetVelocity());
            telemetry.addData("launcher get speed", "launcher get speed %f", launcher.getVelocity());
        } else {
            rightBumperHeldSince = -1;
            launcher.setVelocity(0);
        }
        // No telemetry.update() here: loop() calls it once at the end. Calling it here too
        // cleared everything added before it and could drop lines from the log.

        /*
         * Here we ask if the driver is currently pressing the right bumper, AND the launcher is
         * spinning fast enough to make a successful shot. If it is, then we will turn on the
         * windmill servo to start feeding the elements into the launcher motor. We also
         * add some power to the intake power. This can sometimes help dislodge stuck elements from
         * inside the hopper.
         */
        // After the bumper has been held for 10s, feed anyway even if the launcher never got
        // up to launcherMinVelocity.
        boolean heldLongEnough = rightBumperHeldSince >= 0
                && System.currentTimeMillis() - rightBumperHeldSince > 5_000;
        if (gamepad1.right_bumper
                && (launcher.getVelocity() > launcherMinVelocity() || heldLongEnough)) {
            windmillFeeder.feed();
        } else {
            if (gamepad1.left_bumper) {
                windmillFeeder.reverse();
            } else {
                windmillFeeder.stop();
            }
        }
    }
}
