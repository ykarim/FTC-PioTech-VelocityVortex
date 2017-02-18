package org.firstinspires.ftc.teamcode.sensors.accel;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Uses the phone accelerometer after accounting for gravity to determine if the phone is moving.
 * Because the gravity is accounted for it uses the relative movement passed its resolution to
 * determine if the robot moves
 */
public class PhoneAccelerometer extends org.firstinspires.ftc.teamcode.sensors.Sensor
        implements SensorEventListener {

    private double movement, resolution;
    private Double[] values = new Double[4];

    public enum Status {
        MOVING,
        STOPPED
    }

    @Nullable
    private SensorManager manager;

    @NonNull
    public Status getStatus() {
        //if the movement is greater than the resolution say its moving
        return (movement > resolution) ? Status.MOVING : Status.STOPPED;
    }

    @Override
    public void start(@NonNull HardwareMap map) {
        movement = 0;
        manager = (SensorManager) map.appContext.getSystemService(Context.SENSOR_SERVICE);
        Sensor sensor = manager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        resolution = sensor.getResolution();
        manager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    public void stop() {
        manager.unregisterListener(this);
        manager = null;
    }

    @Override
    public void onSensorChanged(@NonNull SensorEvent event) {
        float x = event.values[0];
        float y = event.values[1];
        float z = event.values[2];
        //convert the movement to a general force
        movement = Math.sqrt(x * x + y * y + z * z);

        values[0] = (double) x;
        values[1] = (double) y;
        values[2] = (double) z;
        values[3] = movement;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    @Override
    public Object[] getValues() {
        return values;
    }
}