package org.firstinspires.ftc.teamcode.sensors;

public interface Sensor {

    /**
     * Starts sensor and performs all required initialization tasks
     */
    void start();

    void stop();

    /**
     * Returns sensor data
     * @return <T> of data
     */
    <T> T getValues();
}
