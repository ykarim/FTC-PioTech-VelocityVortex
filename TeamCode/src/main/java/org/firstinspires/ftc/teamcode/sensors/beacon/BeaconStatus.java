package org.firstinspires.ftc.teamcode.sensors.beacon;

import org.firstinspires.ftc.teamcode.utils.Color;

public class BeaconStatus {

    private static Color leftColor = Color.NA;
    private static Color rightColor = Color.NA;

    public static Color getLeftColor() {
        return leftColor;
    }

    public static Color getRightColor() {
        return rightColor;
    }

    public static void setLeftColor(Color leftColor) {
        BeaconStatus.leftColor = leftColor;
    }

    public static void setRightColor(Color rightColor) {
        BeaconStatus.rightColor = rightColor;
    }
}
