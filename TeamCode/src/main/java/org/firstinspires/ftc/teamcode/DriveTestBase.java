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

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

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
 * Shared TeleOp for the DriveTestRed* / DriveTestBlue* OpModes. Each subclass picks the
 * launcher target velocity and the alliance color (via DriveTestBaseRed / DriveTestBaseBlue);
 * the minimum (feed) velocity is always target - 25. The speeds are per-instance, so
 * adjusting them here never changes Constants.
 *
 * Hold x to auto-rotate toward this color's AprilTags (see TagAimer).
 */
public abstract class DriveTestBase extends OpMode {

    private int launcherTargetVelocity;
    private int launcherMinVelocity;
    private final boolean red;
    private TagAimer tagAimer;

    protected DriveTestBase(int launcherTargetVelocity, boolean red) {
        this.launcherTargetVelocity = launcherTargetVelocity;
        this.launcherMinVelocity = launcherTargetVelocity - 25;
        this.red = red;
    }

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    boolean stopIntake = false;
    private long timeNow;
    // When right_bumper started being held, or -1 when it isn't held.
    private long rightBumperHeldSince = -1;

    private GoBildaPinpointDriver odo;

    private WindmillFeeder windmillFeeder;
    private Follower follower;


    @Override
    public void start(){
//        follower.setPose(OpModeStorage.autonomousEndPose);
//        follower.update();
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
        tagAimer = new TagAimer(hardwareMap, telemetry, red);

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
                "target (%d), min (%d)",
                launcherTargetVelocity,
                launcherMinVelocity);
        telemetry.update();
        if (gamepad1.right_bumper) {
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 500) {
            }
            launcherTargetVelocity += 50;
            launcherMinVelocity += 50;
            telemetry.addData(
                    "Speed",
                    "target (%d), min (%d)",
                    launcherTargetVelocity,
                    launcherMinVelocity);
            telemetry.update();
        }
        if (gamepad1.right_trigger_pressed) {
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 500) {
            }
            launcherTargetVelocity += 50;
            launcherMinVelocity += 50;
            telemetry.addData(
                    "Speed",
                    "target (%d), min (%d)",
                    launcherTargetVelocity,
                    launcherMinVelocity);
            telemetry.update();
        }
    }

    @Override
    public void loop() {
//        DrivePowers powers = ManualDrive.fieldCentric(
//                -gamepad1.left_stick_y,
//                gamepad1.left_stick_x,
//                gamepad1.right_stick_x,
//                follower.pose().heading()
//        );
        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        telemetry.addData("Color", tagAimer.colorName());
        if (gamepad1.x) {
            turn = tagAimer.aimTurn(turn);
        } else {
            telemetry.addData("Aim", "hold x to aim");
        }
//        follower.manual(powers);
//        ManualDrive.driveOrHold(follower,powers);
//        if (gamepad1.startWasPressed()) {
//            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
//            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
//            follower.setPose(cornerPose); // overrides our pose
//        }
//        follower.update();

        double[] speeds = {
                (drive + strafe + turn),
                (drive - strafe - turn),
                (drive - strafe + turn),
                (drive + strafe - turn)
        };
//        Pose robotPose = follower.pose(); // returns a Pose object
//        telemetry.addData("Robot X", robotPose.x());
//        telemetry.addData("Robot Y", robotPose.y());
//        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
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
        telemetry.addData(
                "Motors",
                "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)",
                speeds[0],
                speeds[1],
                speeds[2],
                speeds[3]);
        telemetry.addData(
                "Speed",
                "target (%d), min (%d)",
                launcherTargetVelocity,
                launcherMinVelocity);
        odo.update();
        Pose2D position = odo.getPosition();
        telemetry.addData(
                "Odo",
                "x (%f): y (%f) posx %f posy %f encoderx %d encodery %d",
                position.getX(DistanceUnit.INCH),
                position.getY(DistanceUnit.INCH),
                odo.getPosX(DistanceUnit.MM),
                odo.getPosY(DistanceUnit.MM),
                odo.getEncoderX(),
                odo.getEncoderY()
        );
        telemetry.update();
    }


    @Override
    public void stop() {
        tagAimer.stop();
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
            launcher.setVelocity(launcherTargetVelocity);
            telemetry.addData("launcher get speed", "launcher get speed %f", launcher.getVelocity());
        } else {
            telemetry.addData("launcher stopped", "launcher stopped");
            launcher.setVelocity(0);
            rightBumperHeldSince = -1;
        }
        telemetry.update();

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
                && (launcher.getVelocity() > launcherMinVelocity || heldLongEnough)) {
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
