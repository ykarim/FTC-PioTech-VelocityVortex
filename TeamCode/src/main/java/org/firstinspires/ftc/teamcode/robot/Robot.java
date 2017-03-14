package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.sensors.gyro.AdafruitIMU;

// May need to be abstract //
public class Robot {

    private DcMotor rearLeftMotor, rearRightMotor, frontLeftMotor, frontRightMotor;
    private HardwareMap hardwareMap = null;
    private AdafruitIMU adafruitIMU = null;
    private OpticalDistanceSensor ods = null;

    public enum Directions implements DriveTrainDirections{

        NORTH(0, 0, 0, 0), SOUTH(0, 0, 0, 0),
        EAST(0, 0, 0, 0), WEST(0, 0, 0, 0),
        NORTHEAST(0, 0, 0, 0), NORTHWEST(0, 0, 0, 0),
        SOUTHEAST(0, 0, 0, 0), SOUTHWEST(0, 0, 0, 0),
        ROTATE_RIGHT(0, 0, 0, 0), ROTATE_LEFT(0, 0, 0, 0);

        private final double rearLeftPower, rearRightPower, frontLeftPower, frontRightPower;

        @Override
        public double getRearLeftPower() {
            return rearLeftPower;
        }

        @Override
        public double getRearRightPower() {
            return rearRightPower;
        }

        @Override
        public double getFrontLeftPower() {
            return frontLeftPower;
        }

        @Override
        public double getFrontRightPower() {
            return frontRightPower;
        }

        Directions(double rearLeftPower, double rearRightPower, double frontLeftPower, double frontRightPower) {
            this.rearLeftPower = rearLeftPower;
            this.rearRightPower = rearRightPower;
            this.frontLeftPower = frontLeftPower;
            this.frontRightPower = frontRightPower;
        }
    }

    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public DcMotor getRearRightMotor() {
        return rearRightMotor;
    }

    public DcMotor getRearLeftMotor() {
        return rearLeftMotor;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public AdafruitIMU getAdafruitIMU() {
        return adafruitIMU;
    }

    public OpticalDistanceSensor getOds() {
        return ods;
    }

    public final void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initMotors();
        initServos();
        initSensors();

    }

    public void initMotors() {
        rearLeftMotor = hardwareMap.dcMotor.get("RL");
        rearRightMotor = hardwareMap.dcMotor.get("RR");
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");

        setMotorDirections();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initServos() {

    }

    public void initSensors() {
        adafruitIMU = new AdafruitIMU("imu", hardwareMap);
        ods = hardwareMap.opticalDistanceSensor.get("ods");
    }

    public final void move(DriveTrainDirections directions) {
        rearLeftMotor.setPower(directions.getRearLeftPower());
        rearRightMotor.setPower(directions.getRearRightPower());
        frontLeftMotor.setPower(directions.getFrontLeftPower());
        frontRightMotor.setPower(directions.getFrontRightPower());
    }

    /**
     * Moves robot a given distance using encoders
     * @param direction
     * @param distance
     */
    public void move(DriveTrainDirections direction, double distance) {}

    /**
     * Rotates the robot a certain angle with a given direction
     * Uses Encoders
     * @param direction
     * @param angle
     */
    public void rotateEncoders(DriveTrainDirections direction, int angle) {}

    /**
     * @see #rotateEncoders(DriveTrainDirections, int)
     * Uses IMU
     * @param direction
     * @param angle
     */
    public void rotateIMU(DriveTrainDirections direction, int angle) {}

    /**
     * Rotates robot to a certain angle compared to the starting position declared in initialization
     * @param angle
     */
    public void robotAlignToAngle(int angle) {

    }

    public final void setMotorDirections(DcMotorSimple.Direction... directions){
        if (directions.length == 4) {
            rearLeftMotor.setDirection(directions[0]);
            rearRightMotor.setDirection(directions[1]);
            frontLeftMotor.setDirection(directions[2]);
            frontRightMotor.setDirection(directions[3]);
        } else {
            //error out
        }
    }

    public final void setMotorModes(DcMotor.RunMode mode) {
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
    }
}
