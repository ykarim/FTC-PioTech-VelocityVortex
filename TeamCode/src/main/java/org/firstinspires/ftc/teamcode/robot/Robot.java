package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.sensors.gyro.AdafruitIMU;
import org.firstinspires.ftc.teamcode.sensors.light.LightSensor;
import org.firstinspires.ftc.teamcode.utils.Power;

import java.util.Map;

public abstract class Robot {

    private DcMotor leftRearMotor, leftFrontMotor, rightRearMotor, rightFrontMotor;
    private HardwareMap hardwareMap = null;
    private AdafruitIMU adafruitIMU = null;
    private LightSensor ods = null;

    public enum Directions implements DriveTrainDirections{

        NORTH(0, 0, 0, 0), SOUTH(0, 0, 0, 0),
        EAST(0, 0, 0, 0), WEST(0, 0, 0, 0),
        NORTHEAST(0, 0, 0, 0), NORTHWEST(0, 0, 0, 0),
        SOUTHEAST(0, 0, 0, 0), SOUTHWEST(0, 0, 0, 0),
        ROTATE_RIGHT(0, 0, 0, 0), ROTATE_LEFT(0, 0, 0, 0);

        private final double leftRearPower, leftFrontPower, rightRearPower, rightFrontPower;

        public double getLeftRearPower() {
            return leftRearPower;
        }

        public double getLeftFrontPower() {
            return leftFrontPower;
        }

        public double getRightRearPower() {
            return rightRearPower;
        }

        public double getRightFrontPower() {
            return rightFrontPower;
        }

        Directions(double leftRearPower, double leftFrontPower, double rightRearPower, double rightFrontPower) {
            this.leftRearPower = leftRearPower;
            this.leftFrontPower = leftFrontPower;
            this.rightRearPower = rightRearPower;
            this.rightFrontPower = rightFrontPower;
        }
    }

    public DcMotor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotor getRightRearMotor() {
        return rightRearMotor;
    }

    public DcMotor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotor getLeftRearMotor() {
        return leftRearMotor;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public AdafruitIMU getAdafruitIMU() {
        return adafruitIMU;
    }

    public LightSensor getOds() {
        return ods;
    }

    public final void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initMotors();
        initServos();
        initSensors();

    }

    public void initMotors() {
        leftRearMotor = getDevice(hardwareMap.dcMotor, "RL");
        leftFrontMotor = getDevice(hardwareMap.dcMotor, "RR");
        rightRearMotor = getDevice(hardwareMap.dcMotor, "FL");
        rightFrontMotor = getDevice(hardwareMap.dcMotor, "FR");

        setMotorDirections();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public abstract void initServos();

    public void initSensors() {
        adafruitIMU = new AdafruitIMU("imu", hardwareMap);
        ods = new LightSensor("ods", hardwareMap);
    }

    /**
     * Get the value associated with an id and instead of raising an error return null and log it
     *
     * @param map  the hardware map from the HardwareMap
     * @param name The ID in the hardware map
     * @param <T>  the type of hardware map
     * @return the hardware device associated with the name
     */
    public final <T extends HardwareDevice> T getDevice(HardwareMap.DeviceMapping<T> map, String name) {
        for (Map.Entry<String, T> item : map.entrySet()) {
            if (!item.getKey().equalsIgnoreCase(name)) {
                continue;
            }
            return item.getValue();
        }
        RobotLog.e("ERROR: " + name + " not found!");
        return null;
    }

    public final void move(DriveTrainDirections direction) {
        leftRearMotor.setPower(direction.getLeftRearPower());
        leftFrontMotor.setPower(direction.getLeftFrontPower());
        rightRearMotor.setPower(direction.getRightRearPower());
        rightFrontMotor.setPower(direction.getRightFrontPower());
    }

    public abstract void move(DriveTrainDirections direction, Power power);

    /**
     * Moves robot a given distance using encoders
     * @param direction
     * @param distance
     */
    public abstract void move(DriveTrainDirections direction, double distance);

    /**
     * Rotates the robot a certain angle with a given direction
     * Uses Encoders
     * @param direction
     * @param angle
     */
    public abstract void rotateEncoders(DriveTrainDirections direction, int angle);

    /**
     * @see #rotateEncoders(DriveTrainDirections, int)
     * Uses IMU
     * @param direction
     * @param angle
     */
    public abstract void rotateIMU(DriveTrainDirections direction, int angle);

    /**
     * Rotates robot to a certain angle compared to the starting position declared in initialization
     * @param angle
     */
    public abstract void robotAlignToAngle(int angle);

    public final void setMotorDirections(DcMotorSimple.Direction... directions){
        if (directions.length == 4) {
            leftRearMotor.setDirection(directions[0]);
            leftFrontMotor.setDirection(directions[1]);
            rightRearMotor.setDirection(directions[2]);
            rightFrontMotor.setDirection(directions[3]);
        } else {
            //error out
        }
    }

    public final void setMotorModes(DcMotor.RunMode mode) {
        leftRearMotor.setMode(mode);
        leftFrontMotor.setMode(mode);
        rightRearMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
    }
}
