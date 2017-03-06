package org.firstinspires.ftc.teamcode.utils;

public enum Power {
    LOW(0.25),
    MEDIUM(0.5),
    MIDHIGH(0.75),
    HIGH(1.0);

    private final double power;

    public double getPower() {
        return power;
    }

    Power(double power) {
        this.power = power;
    }
}
