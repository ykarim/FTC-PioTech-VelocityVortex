package org.firstinspires.ftc.teamcode.sensors.beacon;

public class BeaconStatus {

    public enum Color {
        RED, BLUE, NA
    }

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
