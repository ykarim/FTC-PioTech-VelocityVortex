package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankRobot extends Robot {

    private DcMotor intake = null, shoot = null;

    public enum Directions implements DriveTrainDirections{

        NORTH(1.0, 1.0, -1.0, -1.0), SOUTH(-1.0, -1.0, 1.0, 1.0),
        ROTATE_RIGHT(1.0, 1.0, 1.0, 1.0), ROTATE_LEFT(-1.0, -1.0, -1.0, -1.0),
        NONE(0, 0, 0, 0);

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

    public TankRobot(HardwareMap hwMap) {
        init(hwMap);
        setMotorDirections(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void initMotors() {
        super.initMotors();
        intake = getHardwareMap().dcMotor.get("INTAKE");
        shoot = getHardwareMap().dcMotor.get("SHOOT");
    }

    /**
     * @see Robot#move(DriveTrainDirections, double)
     * @param direction
     * @param distance
     */
    @Override
    public void move(DriveTrainDirections direction, double distance) {
        int flTarget = 0;
        int frTarget = 0;
        int rlTarget = 0;
        int rrTarget = 0;

        if (direction == Directions.NORTH) {
            flTarget = getFrontLeftMotor().getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = getFrontRightMotor().getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            rlTarget = getRearLeftMotor().getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            rrTarget = getRearRightMotor().getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Directions.SOUTH) {
            flTarget = getFrontLeftMotor().getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = getFrontRightMotor().getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            rlTarget = getRearLeftMotor().getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            rrTarget = getRearRightMotor().getTargetPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
        }

        getFrontLeftMotor().setTargetPosition(flTarget);
        getFrontRightMotor().setTargetPosition(frTarget);
        getRearLeftMotor().setTargetPosition(rlTarget);
        getRearRightMotor().setTargetPosition(rrTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        move(direction);

        while (getRearLeftMotor().isBusy() && getRearRightMotor().isBusy()
                && getFrontLeftMotor().isBusy() && getFrontRightMotor().isBusy()) {}

        move(Directions.NONE);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @see Robot#rotateEncoders(DriveTrainDirections, int)
     * @param direction
     * @param angle
     */
    @Override
    public void rotateEncoders(DriveTrainDirections direction, int angle) {
        double radians = Math.toRadians(angle);
        double distanceToMove = radians * RobotConstants.distFromCenterToWheel;

        //Set target position
        int flTarget = 0;
        int frTarget = 0;
        int rlTarget = 0;
        int rrTarget = 0;

        if (direction == Directions.ROTATE_LEFT) {
            flTarget = getFrontLeftMotor().getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            frTarget = getFrontRightMotor().getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rlTarget = getRearLeftMotor().getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rrTarget = getRearRightMotor().getTargetPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Directions.ROTATE_RIGHT) {
            flTarget = getFrontLeftMotor().getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            frTarget = getFrontRightMotor().getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rlTarget = getRearLeftMotor().getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rrTarget = getRearRightMotor().getTargetPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
        }

        getFrontLeftMotor().setTargetPosition(flTarget);
        getFrontRightMotor().setTargetPosition(frTarget);
        getRearLeftMotor().setTargetPosition(rlTarget);
        getRearRightMotor().setTargetPosition(rrTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        move(direction);

        while (getRearLeftMotor().isBusy() && getRearRightMotor().isBusy()
                && getFrontLeftMotor().isBusy() && getFrontRightMotor().isBusy()) {}

        move(Directions.NONE);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @see #rotateEncoders(DriveTrainDirections, int)
     * Uses IMU
     * @param direction
     * @param angle
     */
    @Override
    public void rotateIMU(DriveTrainDirections direction, int angle) {
        if (direction == Directions.ROTATE_LEFT) {
            while (getAdafruitIMU().getHeading() < -angle) {
                move(Directions.ROTATE_LEFT);
            }
        } else if (direction == Directions.ROTATE_RIGHT) {
            while (getAdafruitIMU().getHeading() > angle) {
                move(Directions.ROTATE_RIGHT);
            }
        }
    }

    /**
     * Rotates robot to a certain angle compared to the starting position declared in initialization
     * @param angle
     */
    public void robotAlignToAngle(int angle) {
        if (getAdafruitIMU().getHeading() > getAdafruitIMU().getStartingHeading()) {
            rotateIMU(Directions.ROTATE_RIGHT,
                    (int) (getAdafruitIMU().getHeading()- getAdafruitIMU().getStartingHeading()));
        } else if (getAdafruitIMU().getHeading() < getAdafruitIMU().getStartingHeading()) {
            rotateIMU(Directions.ROTATE_LEFT,
                    (int) (getAdafruitIMU().getHeading()- getAdafruitIMU().getStartingHeading()));
        }
    }
}
