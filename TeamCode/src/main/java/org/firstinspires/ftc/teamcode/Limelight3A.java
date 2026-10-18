package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/*
 * Limelight 3A AprilTag test. Hold an AprilTag in front of the camera and the Driver Station
 * shows whether a tag is detected, plus each tag's ID (code), angle and distance.
 *
 * Pipeline 0 on the Limelight must be an AprilTag (Fiducial) pipeline. Distance comes from the
 * tag's 3D pose, so "Full 3D" must be on in that pipeline and the tag size must be set
 * correctly. Otherwise distance reads 0.
 */
@TeleOp(name = "Limelight3A", group = "Test")
public class Limelight3A extends OpMode {

    private static final int APRILTAG_PIPELINE = 1;

    private com.qualcomm.hardware.limelightvision.Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addData("Status", "Initialized, press start");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", "%s connected=%b pipeline=%d (%s) fps=%.0f",
                status.getName(), limelight.isConnected(),
                status.getPipelineIndex(), status.getPipelineType(), status.getFps());

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> tags =
                (result != null && result.isValid()) ? result.getFiducialResults() : null;

        if (tags == null || tags.isEmpty()) {
            telemetry.addData("AprilTag", "NOT detected");
        } else {
            telemetry.addData("AprilTag", "DETECTED (%d)", tags.size());
            telemetry.addData("Staleness (ms)", result.getStaleness());
            for (LLResultTypes.FiducialResult tag : tags) {
                Pose3D tagPose = tag.getTargetPoseCameraSpace();
                Position pos = tagPose.getPosition().toUnit(DistanceUnit.INCH);
                // Straight-line distance from the camera to the tag
                double distance = Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
                double tagYaw = tagPose.getOrientation().getYaw(AngleUnit.DEGREES);

                telemetry.addLine("--- Tag ID (code): " + tag.getFiducialId()
                        + "  family: " + tag.getFamily());
                telemetry.addData("  Horizontal angle tx (deg)", "%.1f", tag.getTargetXDegrees());
                telemetry.addData("  Vertical angle ty (deg)", "%.1f", tag.getTargetYDegrees());
                telemetry.addData("  Tag yaw (deg)", "%.1f", tagYaw);
                telemetry.addData("  Distance (in)", "%.1f", distance);
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
