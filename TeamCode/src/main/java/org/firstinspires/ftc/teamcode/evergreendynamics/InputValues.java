package org.firstinspires.ftc.teamcode.evergreendynamics;

import com.acmerobotics.dashboard.config.Config;

@Config
public class InputValues {
    public static double FLYWHEEL_SPEED = 2300;
    public static double SLOW_FLYWHEEL_SPEED = 250;
    public static double INTAKE_SPEED = 2500;
    public static double SLOW_INTAKE_SPEED = 1700;
    public final static double FLICK_POS_RIGHT = 0; //previously 0.115, increasing value reduces how far it goes
    public final static double RESET_POS_RIGHT = 0.2;
    public final static double FLICK_POS_LEFT = 0.285; //prev 0.1,
    public final static double RESET_POS_LEFT = 0.2; //prev 0.005, small number = closer to artifact
    public final static double FLICK_POS_MIDDLE = 0.115;
    public final static double RESET_POS_MIDDLE = 0.2;
    public final static double FLICK_TRAVEL_TIME = 1.1;
    public static double TURRET_SPEED = 0.5;
    public static double SETTLE_TIME= 1.0;
    public final static double LIFT_TRAVEL_TIME = 0.7;
    public final static long MOTIF_LOOP_WAIT = 250;
    public final static double LIFT_START_POS = 0.0;
    public final static double LIFT_END_POS = 0.19;
    public static int EXPOSURE = 10;
    public static int GAIN = 48;
    public static int WHITE_BALANCE = 4800;
    public static int FOCUS = 0;
    public final static long SLEEP_PER_AUTO_FRAMES = 25;
}
