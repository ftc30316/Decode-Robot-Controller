package org.firstinspires.ftc.teamcode.evergreendynamics.robot;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    private static final String PREF_NAME = "RobotPose";
    private static final String KEY_X = "x";
    private static final String KEY_Y = "y";
    private static final String KEY_HEADING = "heading";

    private static final String KEY_TURRET = "turret";

    public static void savePose(Context context, Pose2d pose, double turretHeading) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        prefs.edit()
                .putFloat(KEY_X, (float) pose.position.x)
                .putFloat(KEY_Y, (float) pose.position.y)
                .putFloat(KEY_HEADING, (float) pose.heading.toDouble())
                .putFloat(KEY_TURRET, (float) turretHeading)
                .apply();
    }

    public static Pose2d loadPose(Context context) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        float x = prefs.getFloat(KEY_X, 0f);
        float y = prefs.getFloat(KEY_Y, 0f);
        float heading = prefs.getFloat(KEY_HEADING, 0f);

        return new Pose2d(x, y, heading);
    }


    public static float loadTurretHeading(Context context) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        return prefs.getFloat(KEY_TURRET, 0f);
    }
}

