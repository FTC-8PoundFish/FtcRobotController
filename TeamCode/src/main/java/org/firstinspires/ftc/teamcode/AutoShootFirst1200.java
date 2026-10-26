package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoShootFirst1200", group = "OpMode")
@Config
public class AutoShootFirst1200 extends AutoShootFirstBase {
    // Tunable from FTC Dashboard; read when the OpMode is initialized.
    public static int TARGET_VELOCITY = 1200;

    public AutoShootFirst1200() {
        super(TARGET_VELOCITY);
    }
}
