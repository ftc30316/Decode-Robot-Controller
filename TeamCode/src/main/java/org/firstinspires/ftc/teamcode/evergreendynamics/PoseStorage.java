package org.firstinspires.ftc.teamcode.evergreendynamics;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    private static final String PREF_NAME = "RobotPose";
    private static final String KEY_X = "x";
    private static final String KEY_Y = "y";
    private static final String KEY_HEADING = "heading";

    public static void savePose(Context context, Pose2d pose) {
        SharedPreferences prefs =
                context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);

        prefs.edit()
                .putFloat(KEY_X, (float) pose.position.x)
                .putFloat(KEY_Y, (float) pose.position.y)
                .putFloat(KEY_HEADING, (float) pose.heading.toDouble())
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
}

