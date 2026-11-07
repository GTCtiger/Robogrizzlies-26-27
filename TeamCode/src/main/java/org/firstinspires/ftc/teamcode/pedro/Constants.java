package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.ticksPerUnit.set(OptionalDouble.of(1.0 / 0.00199746322));
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
        c.xPodOffset.set(5.549877497154897);
        c.yPodOffset.set(-1.4479571064626138);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4436194460300618);
                Controller secondaryTranslationalForward = Controller.proportional(0.16390553157819662);
                Controller primaryTranslationalLateral = Controller.proportional(0.7040190235561842);
                Controller secondaryTranslationalLateral = Controller.proportional(0.26011621746923996);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01821202015489118));
                c.brake.set(Controller.proportionalFeedforward(0.015480217131657502));

                c.headingFeedback.set(Controller.proportional(4.670415749430564));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05212330473233076, 0.00759235070414897));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09415413909442844, 0.07088409547931558));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001453053574329096, 0.002664272903788995));

                c.maxAchievableForwardVelocity.set(49.720049542999604);
                c.maxAchievableStrafeVelocity.set(28.228894383040448);
                c.naturalForwardDeceleration.set(23.14669709089582);
                c.naturalStrafeDeceleration.set(45.66168843544951);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}

