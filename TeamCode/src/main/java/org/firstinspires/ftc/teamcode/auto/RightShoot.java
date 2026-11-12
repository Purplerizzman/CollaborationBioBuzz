package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RightShoot", group = "Autonomous")
public class RightShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(24.75, 33.75, 270);
    private final Pose point2 = poseFactory.of(3.75, 85.75, -85.8381);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 1. Shooter starts when play is pressed
        if (Constants.shooter != null) {
            Constants.shooter.setPower(.54);
        }

        // 2. shooterServo1 spins then waits .5 seconds for 5 seconds total
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        while (opModeIsActive() && shootTimer.seconds() < 10.0) {
            double elapsed = shootTimer.seconds();
            // 1.0 second cycle: spin for first 0.5s, wait for next 0.5s
            if ((elapsed % 1.0) < 1.0) {
                if (Constants.shooterServo1 != null) {
                    Constants.shooterServo1.setPower(1.0);
                }
            } else {
                if (Constants.shooterServo1 != null) {
                    Constants.shooterServo1.setPower(0.0);
                }
            }
            follower.update();
        }

        // Stop shooterServo1 and shooter after shooting phase completes
        if (Constants.shooterServo1 != null) {
            Constants.shooterServo1.setPower(0.0);
        }
        if (Constants.shooter != null) {
            Constants.shooter.setPower(0.0);
        }

        // 3. Robot follows path1
        follower.follow(path1());

        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

        // 4. Robot follows path2
        follower.follow(path2());

        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
