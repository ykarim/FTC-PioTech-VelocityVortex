package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotInterface {

    private DcMotor rearLeftMotor, rearRightMotor, frontLeftMotor, frontRightMotor;
    private HardwareMap hardwareMap = null;

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

    public final void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initMotors();
        initServos();
        initSensors();

    }

    private void initMotors() {
        rearLeftMotor = hardwareMap.dcMotor.get("RL");
        rearRightMotor = hardwareMap.dcMotor.get("RR");
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
    }

    private void initServos() {

    }

    private void initSensors() {

    }

    public void move(DriveTrainDirections direction) {
        rearLeftMotor.setPower(direction.getRearLeftPower());
        rearRightMotor.setPower(direction.getRearRightPower());
        frontLeftMotor.setPower(direction.getFrontLeftPower());
        frontRightMotor.setPower(direction.getFrontRightPower());
    }

}
