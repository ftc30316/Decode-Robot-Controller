package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class InputValues {

    // Flywheels
    public static double FLYWHEEL_SPEED = 2000;
    public static double SLOW_FLYWHEEL_SPEED = 250;
    public static double FLYWHEEL_SLOPE = 9.3;
    public static double FLYWHEEL_Y_INTERCEPT = 900;

    // Turret aim motor
    public static double TURRET_MOTOR_POWER = 0.8;
    public static int TURRET_THREAD_SLEEP_TIME = 50;
    public static double TURRET_OFFSET_X = -4.0;
    public static double TURRET_OFFSET_Y = 0;

    // Lift
    public static double LIFT_TRAVEL_TIME = 0.7;
    public static double LIFT_START_POS = 0.0;
    public static double LIFT_END_POS = 0.265;

    // Intake
    public static double INTAKE_SPEED = 2500;
    public static double SLOW_INTAKE_SPEED = 1700;

    // Flickers
    public static double FLICK_TRAVEL_TIME = 0.3;
    public static double FLICK_POS_RIGHT = 0.28; //previously 0.115, increasing value reduces how far it goes
    public static double RESET_POS_RIGHT = 0.21;
    public static double FLICK_POS_LEFT = 0.13; //prev 0.1,
    public static double RESET_POS_LEFT = 0.19; //prev 0.005, small number = closer to artifact
    public static double FLICK_POS_MIDDLE = 0.12;
    public static double RESET_POS_MIDDLE = 0.18;

    // Gate
    public static double CLOSED_GATE_POS_LEFT = 0.1805;
    public static double CLOSED_GATE_POS_RIGHT = 0.218;
    public static double OPEN_GATE_POS_LEFT = 0.21;
    public static double OPEN_GATE_POS_RIGHT = 0.19;


    // Sleep time for the artifact to settle in the lift divot during auto
    public static double SETTLE_TIME = 1.7;


    // Use to compute position for auto aiming turret motor
    public static double TICKS_PER_DEGREE = 5.3277777778;

    // Goal positions
    public static Vector2d BLUE_GOAL_POSITION = new Vector2d(-66,-66);
    public static Vector2d RED_GOAL_POSITION = new Vector2d(-66,66);

}
