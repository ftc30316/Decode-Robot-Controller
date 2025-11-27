package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class InputValues {

    // Shooting flywheels
    public static double FLYWHEEL_SLOPE = 9.3;
    public static double FLYWHEEL_Y_INTERCEPT = 900;

    // Lift flywheels
    public static int LIFT_FLYWHEEL_WAIT_MILLISECONDS = 3000;

    // Belt
    public static double BELT_SERVO_POWER = 1.0;
    public static double SLOW_BELT_SERVO_POWER = 0.5;

    // Turret aim motor
    public static double TURRET_MOTOR_POWER = 0.8;
    public static int TURRET_THREAD_SLEEP_TIME_MILLIS = 50;
    public static double TURRET_OFFSET_X = -4.0;
    public static double TURRET_OFFSET_Y = 0;

    // Intake motor
    public static double INTAKE_SPEED = 2500;
    public static double SLOW_INTAKE_SPEED = 1700;

    // Use to compute position for auto aiming turret motor
    public static double TICKS_PER_DEGREE = 5.3277777778;

    // Goal positions
    public static Vector2d BLUE_GOAL_POSITION = new Vector2d(-66,-66);

    public static Vector2d RED_GOAL_POSITION = new Vector2d(-66,66);

}
