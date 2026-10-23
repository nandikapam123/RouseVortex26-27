package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pin");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.594277757359302);
        c.yPodOffset.set(-1.80451400636688);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20228656325890418);
                Controller secondaryTranslationalForward = Controller.proportional(0.07473947992764587);
                Controller primaryTranslationalLateral = Controller.proportional(0.26391263957180144);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09750866844617502);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016965946116500877));
                c.brake.set(Controller.proportionalFeedforward(0.014421054199025745));

                c.headingFeedback.set(Controller.proportional(2.8890086970167994));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.041140072440362024, 0.008864292260830733));

                c.linearBrakeCoefficients.set(Matrix.diag(0.050802650381545085, 0.06987661130590421));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002637908695788247, 0.0016348177417117066));

                c.maxAchievableForwardVelocity.set(60.06526006488197);
                c.maxAchievableStrafeVelocity.set(47.15719175839938);
                c.naturalForwardDeceleration.set(37.388211351134984);
                c.naturalStrafeDeceleration.set(52.03178591969806);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
    }
}