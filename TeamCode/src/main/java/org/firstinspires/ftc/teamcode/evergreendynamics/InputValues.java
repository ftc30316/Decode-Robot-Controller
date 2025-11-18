package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class InputValues {
    public static double FLYWHEEL_SPEED = 2000;
    public static double SLOW_FLYWHEEL_SPEED = 250;
    public static double INTAKE_SPEED = 2500;
    public static double SLOW_INTAKE_SPEED = 1700;
    public static double FLICK_POS_RIGHT = 0.285; //previously 0.115, increasing value reduces how far it goes
    public static double RESET_POS_RIGHT = 0.2;
    public static double FLICK_POS_LEFT = 0.13; //prev 0.1,
    public static double RESET_POS_LEFT = 0.2; //prev 0.005, small number = closer to artifact
    public static double FLICK_POS_MIDDLE = 0.115;
    public static double RESET_POS_MIDDLE = 0.2;
    public static double CLOSED_GATE_POS_LEFT = 0.1805;
    public static double CLOSED_GATE_POS_RIGHT = 0.218;
    public static double OPEN_GATE_POS_LEFT = 0.21;
    public static double OPEN_GATE_POS_RIGHT = 0.19;
    public static double FLICK_TRAVEL_TIME = 1.1;
    public static double FLYWHEEL_SLOPE = 9.5;
    public static double FLYWHEEL_Y_INTERCEPT = 1000;
    public static double TURRET_MOTOR_POWER = 0.8;
    public static double SETTLE_TIME= 1.0;
    public static double LIFT_TRAVEL_TIME = 0.7;
    public static long MOTIF_LOOP_WAIT = 250;
    public static double LIFT_START_POS = 0.0;
    public static double LIFT_END_POS = 0.222;
    public static double TICKS_PER_DEGREE = 5.3277777778;
    public static int EXPOSURE = 10;
    public static int GAIN = 48;
    public static int WHITE_BALANCE = 4800;
    public static int FOCUS = 0;
    public static long SLEEP_PER_AUTO_FRAMES = 25;
    public static Vector2d BLUE_GOAL_POSITION = new Vector2d(-66,-66);
    public static Vector2d RED_GOAL_POSITION = new Vector2d(-66,66);
}
