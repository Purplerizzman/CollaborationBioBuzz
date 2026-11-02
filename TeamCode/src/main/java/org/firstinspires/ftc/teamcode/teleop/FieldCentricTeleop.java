/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "FieldCentricTeleop", group = "StarterBot")
public class FieldCentricTeleop extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        // Creates the Follower instance (configured with your Pinpoint Localizer in Constants.java)
        follower = Constants.create(hardwareMap);

        telemetry.addData("Status", "Initialized (Field-Centric with Pedro Pathing)");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        if (Constants.shooter != null) {
            Constants.shooter.setPower(1.0);
        }
    }

    @Override
    public void loop() {
        // Reset IMU heading if Driver 1 presses 'x' or 'options'
        if (gamepad1.x || gamepad1.options) {
            follower.setPose(new Pose(follower.pose().x(), follower.pose().y(), 0));
        }

        // 1. Field-Centric Drivetrain Control (Gamepad 1)
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Field-Centric rotation using current heading from follower.pose() (avoids I2C IMU blocking hiccups)
        double heading = follower.pose().heading();
        double rotatedForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotatedStrafe = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        follower.drivetrain.drive(new DrivePowers(rotatedForward, rotatedStrafe, turn), false);

        // Updates the follower (drives motors & updates localizer pose)
        follower.update();

        // 2. Intake Motor & Servos Control (Gamepad 2 A/B)
        double intakePowerControl = 0.0;
        double lsPower = 0.0;
        double rsPower = 0.0;

        if (gamepad2.a) {
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

        // 4. Shooter Motor Control
        if (Constants.shooter != null) {
            Constants.shooter.setPower(0.5);
        }

        // Telemetry readout
        telemetry.addData("X Position (in)", follower.pose().x());
        telemetry.addData("Y Position (in)", follower.pose().y());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake Power", intakePowerControl);
        if (Constants.shooter != null) {
            telemetry.addData("Shooter Power", Constants.shooter.getPower());
        }
        telemetry.update();
    }

    @Override
    public void stop() {}
}
