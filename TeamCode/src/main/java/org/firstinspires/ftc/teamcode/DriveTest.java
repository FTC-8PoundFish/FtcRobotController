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

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.config.FlowerConfig.flower90Deg;
import static org.firstinspires.ftc.config.FlowerConfig.flowerHome;
import static org.firstinspires.ftc.config.FlowerConfig.flowerNectarBlock;
import static org.firstinspires.ftc.config.LauncherConfig.launcherMinVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.launcherTargetVelocity;
import static org.firstinspires.ftc.config.LauncherConfig.nectarTargetVelocity;
import static org.firstinspires.ftc.config.SecondConfig.secondLauncherMinVelocity;
import static org.firstinspires.ftc.config.SecondConfig.secondLauncherTargetVelocity;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.config.DriveConfig;
import org.firstinspires.ftc.config.LauncherConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

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
    private DcMotor neoWindmill = null;
    private DcMotor intake = null;
    // Flower positions come from FlowerConfig (tunable on FTC Dashboard).
    double flowerPos;
    protected final PoseFactory p = PoseFactory.degrees();
    // Where turn() aims. square toggles it between these two.
    private final Pose hiveA = p.of(82,82,0);
    private final Pose hiveB = p.of(0,0,0);
    private Pose Hive = hiveA;
    // For detecting a new square press (down this loop, up last loop).
    private boolean squareWasDown = false;
    // True from a circle press until turn() finishes (or it's cancelled).
    private boolean turningToHive = false;
    // For detecting a new circle press (down this loop, up last loop).
    private boolean circleWasDown = false;
    // When the current turn started; it gives up after TURN_TIMEOUT_MS.
    private long turnStartedAt;
    private static final long TURN_TIMEOUT_MS = 3_000;
    private final RobotRocker rocker = new RobotRocker();
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private Servo rightFlowerIntakeServo = null;
    private Servo leftflowerIntakeServo = null;
    boolean stopIntake = false;
    private long timeNow;
    // When right_bumper started being held, or -1 when it isn't held.
    private long rightBumperHeldSince = -1;

    // Stick values smaller than this are treated as 0, so a stick that doesn't rest
    // exactly at center won't creep the robot.
    private static final double STICK_DEADBAND = 0.05;
    // True after left_trigger turns the intake on, until right_trigger is used.
    private boolean intakeLatchedOn = false;

    private static final int APRILTAG_PIPELINE = 1;  // same pipeline as AutoShootBASE
    private Limelight3A limelight;

    // turn(): proportional rotate-in-place toward Hive. Flip the sign of TURN_KP if the robot
    // turns away from the Hive.
    private static final double TURN_KP = 0.02;          // turn power per degree of error
    private static final double TURN_MAX_POWER = 0.4;
    private static final double TURN_TOLERANCE_DEG = 1.0;
    // Heading (radians) that faces Hive, worked out when circle is pressed.
    private double hiveHeading;
    // Set once the Limelight sees a tag during this circle press; from then on turn() aims
    // with the camera instead of odometry.
    private boolean tagSeenWhileTurning = false;

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
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        neoWindmill = hardwareMap.get(DcMotor.class, "NEO");
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
        follower = Constants.create(hardwareMap);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setPower(0);

        rightIntakeServo.setPower(0);
        neoWindmill.setPower(0);
        neoWindmill.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillFeeder = new WindmillFeeder(neoWindmill);
//        tagAimer = new TagAimer(hardwareMap, telemetry, red);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontRight.setZeroPowerBehavior(FLOAT);
        frontLeft.setZeroPowerBehavior(FLOAT);
        backLeft.setZeroPowerBehavior(FLOAT);
        backRight.setZeroPowerBehavior(FLOAT);
        intake.setZeroPowerBehavior(BRAKE);


        // The Pinpoint ("odo") is set up by the follower's localizer in Constants.create(),
        // with the tuned pod offsets. Don't reconfigure it here: setting the offsets to 0
        // made turning in place read as x/y movement.

        // Limelight, same setup as AutoShootBASE. turn() stops as soon as it sees any tag.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        Pose cornerPose = new Pose(0, 0, Math.toRadians(0));
        // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
        follower.setPose(cornerPose);

    }

    @Override
    public void loop() {
        double drive = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x);
        double turn = deadband(gamepad1.right_stick_x);
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
        double[] speeds = {
                (drive + strafe + turn),
                (drive - strafe - turn),
                (drive - strafe + turn),
                (drive + strafe - turn)
        };
        if (gamepad1.dpad_left) {
            Pose cornerPose = new Pose(133.5, 133.5, Math.toRadians(0));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }
        follower.update();
        // Panic stop: while dpad_right is held, everything is stopped and the rest of loop()
        // is skipped, so nothing can start again until it's released.
        if (gamepad1.dpad_right) {
            panicStop();
            return;
        }



        // Holding y with both sticks centered rocks the robot forward and back (see
        // RobotRocker). Any stick input takes over, so normal driving always wins.
        boolean sticksCentered = drive == 0 && strafe == 0 && turn == 0
                && deadband(gamepad1.right_stick_y) == 0;
        boolean rocking = gamepad1.y && sticksCentered;
        double rock = rocker.drive(rocking);
        if (rocking) {
            drive = rock;
        }
        launch();
        if (gamepad1.right_trigger_pressed){
            intake.setPower(-1);
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(1);
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
            rightIntakeServo.setPower(-1);
            intakeLatchedOn = true;
        }
        if (gamepad1.right_trigger_pressed) {
            intakeLatchedOn = false;
        }
        // dpad_down = home + 90 degrees, dpad_up = home (mirrored on each side),
        // x = nectar block. (dpad_right is the panic stop.)
        if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.x) {
            if (gamepad1.dpad_down){
                flowerPos = flowerHome() + flower90Deg();
            } else if (gamepad1.dpad_up){
                flowerPos = flowerHome();
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

        // Each press of square switches the Hive between (82, 82) and (0, 0). It applies to the
        // next circle press; a turn already running keeps its target.
        boolean squarePressed = gamepad1.square && !squareWasDown;
        squareWasDown = gamepad1.square;
        if (squarePressed) {
            Hive = (Hive == hiveA) ? hiveB : hiveA;
        }
        telemetry.addData("Hive", "(%.0f, %.0f) - square to switch", Hive.x(), Hive.y());

        // One press of circle turns in place to face the Hive (see turn()) and finishes on its
        // own. Pressing circle again, moving a stick, or TURN_TIMEOUT_MS cancels it.
        boolean circlePressed = gamepad1.circle && !circleWasDown;
        circleWasDown = gamepad1.circle;
        if (circlePressed) {
            if (turningToHive) {
                turningToHive = false;
                telemetry.addData("Turn", "cancelled (circle)");
            } else {
                turningToHive = true;
                turnStartedAt = System.currentTimeMillis();
                tagSeenWhileTurning = false;
                Pose current = follower.pose();
                hiveHeading = Math.atan2(Hive.y() - current.y(), Hive.x() - current.x());
            }
        }
        if (turningToHive && (drive != 0 || strafe != 0 || turn != 0)) {
            turningToHive = false;
            telemetry.addData("Turn", "cancelled (stick)");
        }
        if (turningToHive && System.currentTimeMillis() - turnStartedAt > TURN_TIMEOUT_MS) {
            turningToHive = false;
            telemetry.addData("Turn", "timed out");
        }
        if (turningToHive) {
            turn();  // sets turningToHive = false once it's done
            return;
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
    }

    @Override
    public void stop() {
        limelight.stop();
    }


//    @Override
//    public void stop() {
//        tagAimer.stop();
//    }

    private static double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }
//    private Path turn(int x, int y, int z) {
//        final PoseFactory p = PoseFactory.degrees();
//        int X = x-82;
//        int Y = y-82;
//        Pose turning = p.of(x,y,Math.atan2(y,x));
//        Pose currentPose = p.of(x,y,z);
//        return line(currentPose,turning).linear(currentPose, turning);
//    }
    // Stop every motor and servo that moves: drivetrain (and any follower path or hold),
    // launcher, intake and windmill. The intake stays off after release until a trigger
    // turns it back on.
    private void panicStop() {
        follower.stop();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        launcher.setVelocity(0);
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillFeeder.stop();
        intakeLatchedOn = false;
        stopIntake = false;
        rightBumperHeldSince = -1;
        telemetry.addData("PANIC STOP", "release dpad_right to drive again");
        telemetry.update();
    }

    // Rotate in place. Call every loop while turningToHive is true. Only a turn power goes to
    // the wheels (no drive or strafe), so x and y stay put and only the heading changes.
    //   No tag seen yet: turn toward hiveHeading using odometry.
    //   Once the Limelight sees a tag: aim with the camera instead, turning until that tag is
    //   in the center of the image (tx within TURN_TOLERANCE_DEG). If the tag is lost after
    //   that, stop rather than swing back toward the odometry heading.
    // Finishes (turningToHive = false) once there's nothing left to turn: tag centered, tag
    // lost, or Hive heading reached.
    private void turn() {
        Double tx = tagTx();
        if (tx != null) {
            tagSeenWhileTurning = true;
        }
        double power = 0;
        if (tagSeenWhileTurning) {
            if (tx == null) {
                telemetry.addData("Turn", "tag lost, stopped");
            } else if (Math.abs(tx) <= TURN_TOLERANCE_DEG) {
                telemetry.addData("Turn", "tag centered (tx %.1f deg)", tx);
            } else {
                // tx > 0: tag is right of center, so turn clockwise (positive power).
                power = Math.max(-TURN_MAX_POWER, Math.min(TURN_MAX_POWER, TURN_KP * tx));
                telemetry.addData("Turn", "centering tag, tx %.1f deg, power %.2f", tx, power);
            }
        } else {
            double error = hiveHeading - follower.pose().heading();
            double errorDeg = Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error)));
            if (Math.abs(errorDeg) > TURN_TOLERANCE_DEG) {
                // Positive turn power rotates clockwise (heading goes down), like the right
                // stick, so a positive error (need to turn counter-clockwise) needs negative power.
                power = Math.max(-TURN_MAX_POWER, Math.min(TURN_MAX_POWER, -TURN_KP * errorDeg));
            }
            telemetry.addData("Turn", "facing Hive, error %.1f deg, power %.2f", errorDeg, power);
        }
        // Same mecanum formula as the sticks, with drive = strafe = 0.
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        if (power == 0) {
            turningToHive = false;  // done
        }
    }

    // Horizontal angle (tx, degrees) from the camera's center to the visible AprilTag closest
    // to center, or null if no tag is visible. Picking the closest one keeps the aim from
    // jumping between tags when several are in view.
    private Double tagTx() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        Double best = null;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            double tx = tag.getTargetXDegrees();
            if (best == null || Math.abs(tx) < Math.abs(best)) {
                best = tx;
            }
        }
        return best;
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
        } else {
            rightBumperHeldSince = -1;
            launcher.setVelocity(500);
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
