package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * AutoShootBASE, waiting for a red AprilTag (30-33) before shooting.
 */
@Autonomous(name = "AutoShootRED", group = "OpMode")
public class AutoShootRED extends AutoShootBASE {
    @Override
    protected String allianceName() {
        return "red";
    }

    @Override
    protected boolean isOurTag(int id) {
        return id >= RED_TAG_MIN && id <= RED_TAG_MAX;
    }
}
