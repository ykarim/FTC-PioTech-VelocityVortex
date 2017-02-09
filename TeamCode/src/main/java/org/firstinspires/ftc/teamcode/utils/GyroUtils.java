package org.firstinspires.ftc.teamcode.utils;

import android.hardware.Sensor;

public class GyroUtils {

    static float azimuth = 0;
    static float pitch = 0;
    static float roll = 0;

    /**
     * Returns speed of rotation around y-axis of phone's gyro sensor
     * @param gyro
     * @return rotation around y-axis of phone
     * @see <a href="https://rpappalax.files.wordpress.com/2014/01/sensors_yaw.jpg"></a>
     */
    public static double getRoll (PhoneSensor gyro) {
        if (gyro != null && gyro.getType() == Sensor.TYPE_GYROSCOPE) {
            return gyro.values[1];
        } else {
            return 0;
        }
    }

    public static float getAzimuth() {
        return azimuth;
    }

    public static float getPitch() {
        return pitch;
    }

    public static float getRoll() {
        return roll;
    }


}
