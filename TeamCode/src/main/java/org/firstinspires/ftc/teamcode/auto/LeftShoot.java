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

@Autonomous(name = "LeftShoot", group = "Autonomous")
public class LeftShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.4477, 135.7038, 90);
    private final Pose path1 = poseFactory.of(60.2992, 112.0773, 180);
    private final Pose point2 = poseFactory.of(4.9247, 110.7997, -85.3216);

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
            Constants.shooter.setPower(.56);
        }

        // 10 second pause before shooting loop
        ElapsedTime pauseTimer = new ElapsedTime();
        pauseTimer.reset();
        while (opModeIsActive() && pauseTimer.seconds() < 10.0) {
            follower.update();
        }

        // 2. shooterServo1 spins then waits .5 seconds for 5 seconds total
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        while (opModeIsActive() && shootTimer.seconds() < 5.0) {
            double elapsed = shootTimer.seconds();
            // 1.0 second cycle: spin for first 0.5s, wait for next 0.5s
            if ((elapsed % 1.0) < 1.5) {
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

        // Stop shooterServo1 after shooting phase completes
        if (Constants.shooterServo1 != null) {
            Constants.shooterServo1.setPower(0.0);
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
