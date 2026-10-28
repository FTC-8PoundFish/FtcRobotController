package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/*
 * Limelight AprilTag aiming shared by LimelightAimBase and DriveTest. aimTurn() returns the
 * turn power that rotates the robot so the camera faces the center between the 2 middle tags
 * of a group of 4 of this color:
 *   red:  30-33 (aim between 31 & 32) or 34-37 (between 35 & 36)
 *   blue: 38-41 (aim between 39 & 40) or 42-45 (between 43 & 44)
 */
public class TagAimer {

    private static final int APRILTAG_PIPELINE = 1;  // same pipeline as AutoShootSecond

    // Turn power per degree of error. Flip the sign if the robot turns away from the tags.
    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_TURN = 0.4;
    // Stop turning when we're this close (degrees).
    private static final double AIM_TOLERANCE_DEG = 1.0;

    private final boolean red;
    private final int[] groupFirstIds;  // first tag ID of each group of 4
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public TagAimer(HardwareMap hardwareMap, Telemetry telemetry, boolean red) {
        this.red = red;
        this.groupFirstIds = red ? new int[]{30, 34} : new int[]{38, 42};
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    public String colorName() {
        return red ? "RED" : "BLUE";
    }

    /*
     * Turn power toward the aim point, or manualTurn if no tag of this color is visible.
     * Call every loop() while the aim button is held.
     */
    public double aimTurn(double manualTurn) {
        Double aimTx = findAimAngle();
        if (aimTx == null) {
            telemetry.addData("Aim", "no %s tags found", red ? "red" : "blue");
            return manualTurn;
        }
        if (Math.abs(aimTx) <= AIM_TOLERANCE_DEG) {
            telemetry.addData("Aim", "ON TARGET (%.1f deg)", aimTx);
            return 0;
        }
        double turn = clip(AIM_KP * aimTx, AIM_MAX_TURN);
        telemetry.addData("Aim", "turning, error %.1f deg, turn %.2f", aimTx, turn);
        return turn;
    }

    public void stop() {
        limelight.stop();
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
}
