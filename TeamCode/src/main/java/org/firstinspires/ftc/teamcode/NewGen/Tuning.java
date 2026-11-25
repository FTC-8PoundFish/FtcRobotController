package org.firstinspires.ftc.teamcode.NewGen;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.procedures.Tests;

public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, org.firstinspires.ftc.teamcode.Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, org.firstinspires.ftc.teamcode.Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}
