package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig driveConfig = new MecanumConfig(
        c -> {
            c.frontLeftName.set("LFD");
            c.backLeftName.set("LRD");
            c.frontRightName.set("RFD");
            c.backRightName.set("RRD");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.970879502183809);
        c.yPodOffset.set(-0.7523984984150084);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2688336554892009);
                Controller secondaryTranslationalForward = Controller.proportional(0.09932685233568707);
                Controller primaryTranslationalLateral = Controller.proportional(0.3020139583563443);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11158608765107533);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019884062396823787));
                c.brake.set(Controller.proportionalFeedforward(0.01690145303730022));

                c.headingFeedback.set(Controller.proportional(4.053368845258689));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03999549910266359, 0.009230873302517933));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05700428568617646, 0.0372649534872135));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002324074415689464, 0.002487002755907323));

                c.maxAchievableForwardVelocity.set(49.86219847798197);
                c.maxAchievableStrafeVelocity.set(40.37224201274695);
                c.naturalForwardDeceleration.set(37.44041421123742);
                c.naturalStrafeDeceleration.set(64.88197077176675);
            }
    );

    // DC Motors
    public static DcMotorEx shooter;
    public static DcMotorEx intake1;

    // Continuous Rotation Servos
    public static CRServo lsIntake;
    public static CRServo rsIntake;
    public static CRServo shooterServo1;

    public static Follower create(HardwareMap h) {
        Mecanum drivetrain = new Mecanum(h, driveConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(h, localizerConfig);

        // Initialize DC Motors safely
        try { shooter = h.get(DcMotorEx.class, "Shooter"); } catch (Exception ignored) { shooter = null; }
        try { intake1 = h.get(DcMotorEx.class, "Intake1"); } catch (Exception ignored) { intake1 = null; }

        // Initialize Continuous Rotation Servos safely
        try { lsIntake = h.get(CRServo.class, "LSIntake"); } catch (Exception ignored) { lsIntake = null; }
        try { rsIntake = h.get(CRServo.class, "RSIntake"); } catch (Exception ignored) { rsIntake = null; }
        try { shooterServo1 = h.get(CRServo.class, "ShooterServo1"); } catch (Exception ignored) { shooterServo1 = null; }

        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
    }
}
