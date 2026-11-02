package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Teleoptest1", group = "TeleOp")
public class test1 extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        // Creates the Follower instance (configured with your Pinpoint Localizer in Constants.java)
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        // Drivetrain control (Gamepad 1)
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Field-Centric rotation using current heading from follower.pose()
        double heading = follower.pose().heading();
        double rotatedForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotatedStrafe = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        follower.drivetrain.drive(new DrivePowers(rotatedForward, rotatedStrafe, turn), false);

        // Updates the follower (drives motors & updates Pinpoint localizer pose)
        follower.update();

        // Optional: Press 'A' on Gamepad 1 to reset field heading to 0 degrees
        if (gamepad1.a) {
            follower.setPose(new Pose(follower.pose().x(), follower.pose().y(), 0));
        }

        // --- Gamepad 2 Controls ---

        // 1. Intakes (LSIntake, RSIntake, and Intake1) controlled by Gamepad 2 X and O (B)
        double intakePower = 0.0;
        if (gamepad2.x) {
            intakePower = 1.0; // One direction
        } else if (gamepad2.b) { // B / Circle (O) on PlayStation controllers
            intakePower = -1.0; // Opposite direction
        }
        if (Constants.intake1 != null) Constants.intake1.setPower(intakePower);
        if (Constants.lsIntake != null) Constants.lsIntake.setPower(intakePower);
        if (Constants.rsIntake != null) Constants.rsIntake.setPower(intakePower);

        // 2. Shooter Servo 1 (ShooterServo1) controlled by Gamepad 2 Triggers
        double shooterServoPower = 0.0;
        if (gamepad2.left_trigger > 0.1) {
            shooterServoPower = gamepad2.left_trigger;
        } else if (gamepad2.right_trigger > 0.1) {
            shooterServoPower = -gamepad2.right_trigger;
        }
        if (Constants.shooterServo1 != null) Constants.shooterServo1.setPower(shooterServoPower);

        // 3. Shooter DC Motor controlled by Gamepad 2 Y button (Toggle/Hold)
        if (Constants.shooter != null) {
            if (gamepad2.y) {
                Constants.shooter.setPower(1.0);
            } else {
                Constants.shooter.setPower(0.0);
            }
        }

        // Telemetry readout
        telemetry.addData("X Position (in)", follower.pose().x());
        telemetry.addData("Y Position (in)", follower.pose().y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Shooter Servo Power", shooterServoPower);
        telemetry.update();
    }
}
