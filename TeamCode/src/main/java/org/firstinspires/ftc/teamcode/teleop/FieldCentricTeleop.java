package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedro.Constants.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(name = "Field Centric Teleop", group = "Linear OpMode")
public class FieldCentricTeleop extends LinearOpMode {

    // TUNING COEFFICIENTS FOR PEDRO PATHING 3.0 CONTROLLER (Tunable via FTC Dashboard)
    public static double Kp = 0.001;
    public static double Ki = 0.0001;
    public static double Kd = 0.00001;
    public static double Kf = 0.0003; // Proportional feedforward term
    public static double Ks = 0.03;   // Static friction feedforward term
    public static double TARGET_VELOCITY = 1200; // Ticks per second

    private Follower follower;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing Follower (drivetrain and localizer)
        follower = Constants.create(hardwareMap);

        // Instantiate Pedro Pathing 3.0 Controller combining PID, proportional feedforward, and static feedforward using .plus()
        Controller shooterController = Controller.pid(Kp, Ki, Kd)
                .plus(Controller.proportionalFeedforward(Kf))
                .plus(Controller.staticFeedforward(Ks));

        ElapsedTime velocityTimer = new ElapsedTime();
        double lastPosition = 0;

        telemetry.addData("Status", "Initialized (Field-Centric with Pedro Pathing 3.0 Controller)");
        telemetry.update();

        waitForStart();
        velocityTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update follower to process odometry and drive powers
            follower.update();

            // 1. Field-Centric Drivetrain Control (UNCHANGED)
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            follower.manual(powers);

            // Reset heading to 0 degrees if X button pressed
            if (gamepad1.x) {
                follower.setPose(new Pose(follower.pose().x(), follower.pose().y(), 0));
            }
            // 5. Shooter Motor Control (Runs continuously at full power after START)
            shooter.setPower(0.52);
            // 2. Intake Motor & Servos Control (Gamepad 2 A/B or X/B)
            double intakePowerControl = 0.0;
            double lsPower = 0.0;
            double rsPower = 0.0;

            if (gamepad2.a || gamepad2.x) {
                intakePowerControl = -1.0;
                lsPower = 1.0;
                rsPower = -1.0;
            } else if (gamepad2.b) {
                intakePowerControl = 1.0;
                lsPower = -1.0;
                rsPower = 1.0;
            }

            if (Constants.intake1 != null) Constants.intake1.setPower(intakePowerControl);
            if (Constants.lsIntake != null) Constants.lsIntake.setPower(lsPower);
            if (Constants.rsIntake != null) Constants.rsIntake.setPower(rsPower);

            // 3. Shooter Servo 1 Control (Gamepad 2 Left and Right Bumpers)
            double shooterServoPower = 0.0;
            if (gamepad2.left_bumper) {
                shooterServoPower = 1.0;
            } else if (gamepad2.right_bumper) {
                shooterServoPower = -1.0;
            }
            if (Constants.shooterServo1 != null) Constants.shooterServo1.setPower(shooterServoPower);

            // 4. Shooter Motor Control using Pedro Pathing 3.0 Controller API & Velocity Tracking
            if (shooter != null) {
                double currentPosition = shooter.getCurrentPosition();
                double secondsElapsed = velocityTimer.seconds();

                if (secondsElapsed > 0) {
                    double currentVelocity = (currentPosition - lastPosition) / secondsElapsed;
                    lastPosition = currentPosition;
                    velocityTimer.reset();

                    double shooterPower = 0;
                    if (gamepad2.y) { // Hold Gamepad 2 Y to activate shooter controller
                        shooterPower = shooterController.calculate(TARGET_VELOCITY, currentVelocity);
                    } else {
                        shooterController.reset(); // Clear controller memory when shooter is off
                    }

                    // Clip power between -1.0 and 1.0
                    shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
                    shooter.setPower(shooterPower);

                    // Telemetry for FTC Dashboard or Driver Hub tuning
                    telemetry.addData("Target Velocity (ticks/s)", TARGET_VELOCITY);
                    telemetry.addData("Current Velocity (ticks/s)", currentVelocity);
                    telemetry.addData("Shooter Power Out", shooterPower);
                }
            }

            // Telemetry readout for drivetrain & pose
            telemetry.addData("X Position (in)", follower.pose().x());
            telemetry.addData("Y Position (in)", follower.pose().y());
            telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Intake Power", intakePowerControl);
            telemetry.update();
        }
    }
}
