package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * AutoShootBASE, waiting for a blue AprilTag (42-45) before shooting.
 */
@Autonomous(name = "AutoShootBLUE", group = "OpMode")
public class AutoShootBLUE extends AutoShootBASE {
    @Override
    protected String allianceName() {
        return "blue";
    }

    @Override
    protected boolean isOurTag(int id) {
        return id >= BLUE_TAG_MIN && id <= BLUE_TAG_MAX;
    }
}
