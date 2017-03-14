package org.firstinspires.ftc.teamcode.sensors.gyro;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.sensors.Sensor;

public class AdafruitIMU implements Sensor {

    private final BNO055IMU imu;
    private final String name;

    private double startingHeading;
    private double startingPitch;
    private double startingRoll;

    public AdafruitIMU(String name, HardwareMap hardwareMap) {
        this.name = name;
        imu = hardwareMap.get(BNO055IMU.class, name);
        setParameters();
    }

    /**
     * After initialization in constructor must call this method to record initial values
     */
    public void start() {
        startingHeading = getHeading();
        startingPitch = getPitch();
        startingRoll = getRoll();
    }


    public void stop() {
        //Can't stop this sensor
    }

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }
    private double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        double roll = Math.atan2( 2*(w*x + y*z) , 1 - 2*(x*x + y*y) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }

    public double adjustAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    } //puts angle from 0-360 to -180 to 180

    private double getHeading() {
        return getAngles()[0];
    }
    private double getPitch() {
        return getAngles()[1];
    }
    private double getRoll() {
        return getAngles()[2];
    }

    public double getStartingHeading() {
        return startingHeading;
    }

    public double getStartingRoll() {
        return startingRoll;
    }

    public double getStartingPitch() {
        return startingPitch;
    }

    /**
     * @see Sensor#getValues()
     * @return [0] : heading, [1] : pitch, [2] : roll
     */
    @Override
    public double[] getValues() {
        return new double[]{getHeading(), getPitch(), getRoll()};
    }
}
