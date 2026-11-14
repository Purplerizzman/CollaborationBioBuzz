package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    public static double SHOOTER_POWER = 0.54;
    public static double INTAKE_SPEED = 1.0;

    // Hardware names from Robot Controller configuration.
    public static String LED1_NAME = "led1";
    public static String LED2_NAME = "led2";
    public static String CAM_NAME = "cam";

    public static double shooterP = 150;
    public static double shooterI = 0;
    public static double shooterD = 0;
    public static double shooterF = 15;
    public static double shooterVelocity = 1300;
}
