package org.firstinspires.ftc.teamcode;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.procedures.Tests;
import org.firstinspires.ftc.teamcode.procedures.TwoWheelTuner;

public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

}
