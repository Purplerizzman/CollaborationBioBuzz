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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file includes a field-centric teleop file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge.
 */

@TeleOp(name = "FieldCentricTeleop", group = "StarterBot")
public class FieldCentricTeleop extends OpMode {

    // Declare OpMode member variables (unassigned at declaration, mapped in init()).
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotorEx shooter;
    private DcMotor intake1;
    private CRServo lsIntake;
    private CRServo rsIntake;
    private CRServo shooterServo1;
    private IMU imu;

    // Power tracking variables for telemetry and control
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /*
         * Initialize the hardware variables using the exact names
         * assigned during your robot configuration on the Driver Hub.
         */
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LFD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFD");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "LRD");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "RRD");
        
        intake1         = hardwareMap.get(DcMotor.class, "Intake1");
        shooter         = hardwareMap.get(DcMotorEx.class, "Shooter");
        
        lsIntake        = hardwareMap.get(CRServo.class, "LSIntake");
        rsIntake        = hardwareMap.get(CRServo.class, "RSIntake");
        shooterServo1   = hardwareMap.get(CRServo.class, "ShooterServo1");

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode".
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake1.setZeroPowerBehavior(BRAKE);

        /*
         * Set launcher to RUN_WITHOUT_ENCODER runmode for full power.
         */
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * Initialize servos to neutral power (0).
         */
        lsIntake.setPower(0);
        rsIntake.setPower(0);
        shooterServo1.setPower(0);

        telemetry.addData("Status", "Initialized (Field-Centric)");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they start
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        shooter.setPower(1.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Reset IMU heading if Driver 1 presses the x / Back button
        if (gamepad1.x) {
            imu.resetYaw();
        }

        // 1. Field-Centric Drivetrain Control (Gamepad 1)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Read robot heading from the IMU (in radians)
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement vector by the bot's heading angle for field-centric driving
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        leftFrontPower  = rotY + rotX + rotate;
        rightFrontPower = rotY - rotX - rotate;
        leftBackPower   = rotY - rotX + rotate;
        rightBackPower  = rotY + rotX - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

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

        intake1.setPower(intakePowerControl);
        lsIntake.setPower(lsPower);
        rsIntake.setPower(rsPower);

        // 3. Shooter Servo 1 Control (Gamepad 2 Left and Right Bumpers)
        double shooterServoPower = 0.0;
        if (gamepad2.left_bumper) {
            shooterServoPower = 1.0;
        } else if (gamepad2.right_bumper) {
            shooterServoPower = -1.0;
        }
        shooterServo1.setPower(shooterServoPower);

        // 4. Shooter Motor Control (Runs continuously at full power after START)
        shooter.setPower(0.5);

        // Telemetry readout
        telemetry.addData("Heading (Deg)", Math.toDegrees(botHeading));
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Intake Power", intakePowerControl);
        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}
