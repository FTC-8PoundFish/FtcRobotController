package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/*
 * Shared TeleOp for LimelightRed / LimelightBlue. Regular mecanum drive (same as
 * DriveTestBase). While right_bumper is held, the robot looks for this color's AprilTags and
 * rotates so the camera faces the center between the 2 middle tags of a group of 4:
 *   red:  30-33 (aim between 31 & 32) or 34-37 (between 35 & 36)
 *   blue: 38-41 (aim between 39 & 40) or 42-45 (between 43 & 44)
 * The driver can still drive and strafe with the left stick while aiming.
 */
public abstract class LimelightAimBase extends OpMode {

    private static final int APRILTAG_PIPELINE = 1;  // same pipeline as AutoShootSecondBase

    // Turn power per degree of error. Flip the sign if the robot turns away from the tags.
    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_TURN = 0.4;
    // Stop turning when we're this close (degrees).
    private static final double AIM_TOLERANCE_DEG = 1.0;

    private final boolean red;
    private final int[] groupFirstIds;  // first tag ID of each group of 4

    protected LimelightAimBase(boolean red) {
        this.red = red;
        this.groupFirstIds = red ? new int[]{30, 34} : new int[]{38, 42};
    }

    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private Limelight3A limelight;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addData("Status", "Initialized (%s)", red ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        telemetry.addData("Color", red ? "RED" : "BLUE");

        if (gamepad1.right_bumper) {
            Double aimTx = findAimAngle();
            if (aimTx == null) {
                telemetry.addData("Aim", "no %s tags found", red ? "red" : "blue");
            } else if (Math.abs(aimTx) <= AIM_TOLERANCE_DEG) {
                turn = 0;
                telemetry.addData("Aim", "ON TARGET (%.1f deg)", aimTx);
            } else {
                turn = clip(AIM_KP * aimTx, AIM_MAX_TURN);
                telemetry.addData("Aim", "turning, error %.1f deg, turn %.2f", aimTx, turn);
            }
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

    /*
     * Returns the horizontal angle (tx, degrees) of the point between the 2 middle tags of the
     * group with the most visible tags, or null if no tag of this color is visible.
     *   both middle tags seen -> average of their tx
     *   2+ tags of the group  -> straight-line fit of tx vs. position in group, read at 1.5
     *   only 1 tag            -> that tag's tx (turning toward it brings the others into view)
     */
    private Double findAimAngle() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        List<double[]> best = null;  // {position in group 0..3, tx}
        int bestFirstId = -1;
        for (int firstId : groupFirstIds) {
            List<double[]> seen = new ArrayList<>();
            for (LLResultTypes.FiducialResult tag : tags) {
                int pos = tag.getFiducialId() - firstId;
                if (pos >= 0 && pos <= 3) {
                    seen.add(new double[]{pos, tag.getTargetXDegrees()});
                }
            }
            if (!seen.isEmpty() && (best == null || seen.size() > best.size())) {
                best = seen;
                bestFirstId = firstId;
            }
        }
        if (best == null) {
            return null;
        }

        StringBuilder ids = new StringBuilder();
        for (double[] t : best) {
            ids.append(bestFirstId + (int) t[0]).append(' ');
        }
        telemetry.addData("Group", "%d-%d, aiming between %d & %d",
                bestFirstId, bestFirstId + 3, bestFirstId + 1, bestFirstId + 2);
        telemetry.addData("Tags seen", ids.toString().trim());

        Double middle1 = null, middle2 = null;
        for (double[] t : best) {
            if (t[0] == 1) middle1 = t[1];
            if (t[0] == 2) middle2 = t[1];
        }
        if (middle1 != null && middle2 != null) {
            return (middle1 + middle2) / 2;
        }
        if (best.size() == 1) {
            return best.get(0)[1];
        }

        // Least-squares line tx = a + b * pos, evaluated at pos = 1.5
        double n = best.size(), sumP = 0, sumT = 0, sumPP = 0, sumPT = 0;
        for (double[] t : best) {
            sumP += t[0];
            sumT += t[1];
            sumPP += t[0] * t[0];
            sumPT += t[0] * t[1];
        }
        double b = (n * sumPT - sumP * sumT) / (n * sumPP - sumP * sumP);
        double a = (sumT - b * sumP) / n;
        return a + b * 1.5;
    }

    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
