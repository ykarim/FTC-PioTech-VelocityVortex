package org.firstinspires.ftc.teamcode.utils;

public enum Color {
    RED("RED"),
    BLUE("BLUE"),
    NA("NA");

    private final String color;

    Color(String color) {
        this.color = color;
    }

    public String getColor() {
        return color;
    }
}
