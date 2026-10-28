package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

@Autonomous(name = "AutoShootGarden", group = "OpMode")
public class AutoShootGarden extends OpMode {

    public static final int TARGET_SPEED = 1185;
    private long startTime;

    boolean launch_Done = false;
    boolean stopped = false;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(144, 48, 0);
    private final Pose seccondPose = p.of(140, 20, 0);
    private final Pose park = p.of(56, 5, 0);
    private final Pose gardenPose = p.of(144,0,0);
    private final Pose gardenAproach = p.of(130,0,0);
    private static final int forward = 4;
    private final Pose shootPose = p.of(144 - forward, 48, 10);

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    double intakePower;

    boolean kaboom = false;
    boolean boom = false;

    private GoBildaPinpointDriver odo;

    private WindmillFeeder windmillFeeder;


    public static int Launcher_Velocity = 0;

    //poses from before
    // other poses...
    private Path GardenToSecond() {

        return line(gardenPose, seccondPose).linear(gardenPose, seccondPose);
    }
    private Path toGardenAproach(){
        return line(shootPose,gardenAproach).linear(shootPose, gardenAproach);
    }
    private Path toGarden(){
        return line(gardenAproach,gardenPose).linear(gardenAproach,gardenPose);
    }
    private Path park() {
        return line(seccondPose, park).linear(seccondPose, park);
    }
    private Path startToShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, toGardenAproach())
                // Add mechanism commands here.
//                follow(follower, park())
        );
    }

//    private Path park() {
//        return curve(startPose, controlPose, park).linear(startPose, park);
//    }
// in your autonomous


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setPower(0);
        windmillFeeder = new WindmillFeeder(windmillServo);
        launcher =hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        leftIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        schedule(follow(follower, startToShoot()));
        launcher.setVelocity(TARGET_SPEED);
    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        if (System.currentTimeMillis() - startTime > 10_000) {
            launcher.setVelocity(0);
            windmillServo.setPower(0);
            launcher.setPower(0);
            intake.setPower(1);
            leftIntakeServo.setPower(1);
            rightIntakeServo.setPower(1);
            if (!launch_Done) {
                launch_Done = true;
                telemetry.addData("launch done", "set launch done");
                intake.setPower(1);
                schedule(autoRoutine());
                intake.setPower(1);
            }
            // Checked every loop (not just once above) so it actually fires at the 20s mark.
            if (!stopped && System.currentTimeMillis() - startTime > 12_000) {
                stopped = true;
                Constants.foresightConfig.maxPathSpeed.set(1.0);

                Scheduler.reset();  // cancel whatever command is still running
                follower.stop();    // and stop the drivetrain from following its path
                schedule(follow(follower, toGarden()));
                Constants.foresightConfig.maxPathSpeed.set(20.0);

            }
            if (!boom && System.currentTimeMillis() - startTime > 19_000) {
                boom = true;
                Scheduler.reset();  // cancel whatever command is still running
                follower.stop();    // and stop the drivetrain from following its path
                schedule(follow(follower, GardenToSecond()));
            }
            if (!kaboom && System.currentTimeMillis() - startTime > 21_000) {
                kaboom = true;
                Scheduler.reset();  // cancel whatever command is still running
                follower.stop();    // and stop the drivetrain from following its path
                schedule(follow(follower, park()));
                telemetry.addData("parking","now");
            }
            return;
        }

        if (!launch_Done) {
            if (launcher.getVelocity() >= TARGET_SPEED - 20) {
                telemetry.addData("launch done", "launch_speed_reached");
                windmillFeeder.feed();
                intakePower += 0.5;
            } else {
                telemetry.addData("launch done", "launch_speed_not_reached");
                windmillFeeder.stop();
            }
        }
        telemetry.update();
    }
}

