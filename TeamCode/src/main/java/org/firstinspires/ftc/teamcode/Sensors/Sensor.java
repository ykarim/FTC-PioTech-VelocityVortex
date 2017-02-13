package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class Sensor {

    /**
     * Starts sensor and performs all required initialization tasks
     */
    public abstract void start(HardwareMap hwMap);

    public abstract void stop();

    /**
     * Returns sensor data
     * @return Object array
     */
    public abstract Object[] getValues();
}
