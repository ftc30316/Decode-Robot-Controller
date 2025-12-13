package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class InputValues {

    // Shooting flywheels
    public static double FLYWHEEL_SLOPE = 9.3;
    public static double FLYWHEEL_Y_INTERCEPT = 900;
    public static final boolean FLYWHEEL_TEST_ON = false;
    public static double FLYWHEEL_TEST_VELOCITY = 500;
    public static double FLYWHEEL_MULTIPLIER = 4.0;

    // Lift flywheels
    public static int LIFT_WHEEL_WAIT_MILLISECONDS = 1000;
    public static final double LIFT_WHEEL_WAIT_SECONDS = LIFT_WHEEL_WAIT_MILLISECONDS / 1000.0;

    // Belt
    public static double INTAKE_SERVO_POWER = 1.0;
    public static double SLOW_BELT_SERVO_POWER = 0.5;

    // Turret aim motor
    public static double TURRET_MOTOR_POWER = 0.8;
    public static int TURRET_THREAD_SLEEP_TIME_MILLIS = 50;
    public static double TURRET_OFFSET_X = -5.0;
    public static double TURRET_OFFSET_Y = 0;

    // Intake motor
    public static double INTAKE_POWER = 1.0;
    public static final double INTAKE_POWER_SLOW = 0.1;
    public static double SLOW_INTAKE_SPEED = 1700;


    // Distance from sensor to detect artifact
    public static double ARTIFACT_DISTANCE_DETECTION = 2.6;

    // Use to compute position for auto aiming turret motor
    public static double TICKS_PER_DEGREE = 5.3277777778;

    // Goal positions
    public static Vector2d BLUE_GOAL_POSITION = new Vector2d(-64,-60);

    public static Vector2d RED_GOAL_POSITION = new Vector2d(-64,60);

}
