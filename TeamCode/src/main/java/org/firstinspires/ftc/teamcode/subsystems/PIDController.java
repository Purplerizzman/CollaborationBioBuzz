package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp, ki, kd, kf;

    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Constructor to set our coefficients
    public PIDController(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        timer.reset();
    }

    public double calculate(double targetVelocity, double currentVelocity) {
        // Calculate time elapsed since last loop
        double seconds = timer.seconds();
        timer.reset();

        // Calculate velocity error
        double error = targetVelocity - currentVelocity;

        // Rate of change of the error (Derivative)
        double derivative = (error - lastError) / seconds;

        // Total error over time (Integral)
        integralSum += error * seconds;

        // Save error for next loop
        lastError = error;

        // Combine all components + feedforward baseline
        double output = (kp * error) + (ki * integralSum) + (kd * derivative) + (kf * targetVelocity);

        return output;
    }

    // Helper to reset the integral sum if target changes drastically
    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}

