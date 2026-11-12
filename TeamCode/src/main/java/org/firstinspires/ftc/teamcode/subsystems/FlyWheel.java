package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class FlyWheel {
    private final DcMotorEx flywheelMotor;
    private final Controller velocityController;

    // goBILDA 6000 RPM (5202-0002-0001) Hardware Constants
    public static double TICKS_PER_REV = 28.0;
    public static double MAX_TICKS_PER_SEC = 2800.0;

    // Starting Pedro Pathing 3.0 Tuning Constants for goBILDA 6000 RPM (Tunable via FTC Dashboard)
    public static double kV = 0.000375; // Slightly above theoretical 0.000357 to account for wheel load
    public static double kS = 0.03;     // Static friction compensation
    public static double kP = 0.0010;   // Recovery gain after shooting a game piece
    public static double kI = 0.0;      // Keep at 0 to prevent spin-up overshoot
    public static double kD = 0.00004;  // Light damping for high-RPM stability

    public static boolean UPDATE_PID = false;

    private double targetVelocity = 0.0; // Stored in ticks per second

    public FlyWheel(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Build Pedro Pathing 3.0 composite controller (FF + Static FF + PID)
        Controller feedforward = Controller.proportionalFeedforward(kV);
        Controller staticFf = Controller.staticFeedforward(kS);
        Controller pid = Controller.pid(kP, kI, kD);

        velocityController = feedforward.plus(staticFf).plus(pid);
    }

    /**
     * Set target speed in encoder ticks per second.
     */
    public void setTargetVelocity(double ticksPerSecond) {
        this.targetVelocity = ticksPerSecond;
    }

    /**
     * Set flywheel target speed directly in RPM (0 to 6000).
     */
    public void setTargetRPM(double rpm) {
        double clampedRpm = Math.max(0, Math.min(6000.0, rpm));
        this.targetVelocity = (clampedRpm * TICKS_PER_REV) / 60.0;
    }

    /**
     * Call every loop cycle to maintain target speed/RPM.
     */
    public void update() {
        if (targetVelocity <= 0) {
            flywheelMotor.setPower(0);
            velocityController.reset();
            return;
        }

        if (UPDATE_PID) {
            UPDATE_PID = false;

        }

        double currentVelocity = flywheelMotor.getVelocity();

        // Pedro 3.0 Controller calculates combined FF + PID power using target and current velocity
        double power = velocityController.calculate(targetVelocity, currentVelocity);

        // Clamp output to valid motor power [-1.0, 1.0]
        flywheelMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public double getCurrentVelocity() {
        return flywheelMotor.getVelocity();
    }

    public double getCurrentRPM() {
        return (flywheelMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return (targetVelocity * 60.0) / TICKS_PER_REV;
    }

    /**
     * Checks if flywheel is within an acceptable speed window (ticks/sec or RPM tolerance).
     */
    public boolean isAtSpeed(double tolerance) {
        if (tolerance > 10.0 && tolerance <= 500.0) {
            return targetVelocity > 0 && Math.abs(getTargetRPM() - getCurrentRPM()) <= tolerance;
        }
        return targetVelocity > 0 && Math.abs(targetVelocity - getCurrentVelocity()) <= tolerance;
    }
}
