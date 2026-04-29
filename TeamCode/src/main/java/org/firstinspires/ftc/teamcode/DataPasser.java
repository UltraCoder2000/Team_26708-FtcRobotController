package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class DataPasser {
    public enum Alliance {
        RED,
        BLUE,
        NULL
    }
    public static Alliance currentAlliance = Alliance.NULL;

    public static Pose endAutoPose = new Pose(72, 72, 90);
}