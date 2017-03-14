package org.firstinspires.ftc.teamcode.sensors.light;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class LightSensor {

    private OpticalDistanceSensor sensor = null;

    public enum Status {
        NONE,
        FLOOR(0.0, 0.3),
        LINE(0.6, 1.0);

        private double minRange, maxRange;

        Status() {
            minRange = 0;
            maxRange = 0;
        }

        Status(double minRange, double maxRange) {
            this.minRange = minRange;
            this.maxRange = maxRange;
        }

        public boolean inRange(double value) {
            return (value >= minRange && value <= maxRange);
        }
    }

    public LightSensor(String deviceName, HardwareMap hardwareMap) {
        this.sensor = hardwareMap.opticalDistanceSensor.get(deviceName);
    }

    public Status getStatus() {
        if (Status.FLOOR.inRange(sensor.getLightDetected())) {
            return Status.FLOOR;
        } else if (Status.LINE.inRange(sensor.getLightDetected())) {
            return Status.LINE;
        } else {
            return Status.NONE;
        }
    }
}
