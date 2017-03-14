package org.firstinspires.ftc.teamcode.sensors.light;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.sensors.Sensor;

import java.util.Map;

public class LightSensor implements Sensor {

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

    public LightSensor(@NonNull String deviceName, @NonNull HardwareMap hardwareMap) {
        this.sensor = getDevice(hardwareMap.opticalDistanceSensor, deviceName);
    }

    @Override
    public void start() {
        //Sensor automatically started once plugged in
    }

    @Override
    public void stop() {
        //Can't stop sensor
    }

    /**
     * Get the value associated with an id and instead of raising an error return null and log it
     *
     * @param map  the hardware map from the HardwareMap
     * @param name The ID in the hardware map
     * @param <T>  the type of hardware map
     * @return the hardware device associated with the name
     */
    private <T extends HardwareDevice> T getDevice(@NonNull HardwareMap.DeviceMapping<T> map, String name) {
        for (Map.Entry<String, T> item : map.entrySet()) {
            if (!item.getKey().equalsIgnoreCase(name)) {
                continue;
            }
            return item.getValue();
        }
        RobotLog.e("ERROR: " + name + " not found!");
        return null;
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

    @Override
    public Status getValues() {
        return getStatus();
    }
}
