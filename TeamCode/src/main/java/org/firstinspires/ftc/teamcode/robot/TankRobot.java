package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.light.LightSensor;
import org.firstinspires.ftc.teamcode.utils.Power;

public class TankRobot extends Robot {

    private DcMotor intake = null, shoot = null;

    public enum Directions implements DriveTrainDirections{

        NORTH(1.0, 1.0, -1.0, -1.0), SOUTH(-1.0, -1.0, 1.0, 1.0),
        ROTATE_RIGHT(1.0, 1.0, 1.0, 1.0), ROTATE_LEFT(-1.0, -1.0, -1.0, -1.0),
        NONE(0, 0, 0, 0);

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
     * Sort of a hack to decrease robot speed
     * Since default power is always 1 or -1 can simply subtract desired power by 1 and
     * add/subtract that from default power to achieve the desired speed.
     * @param direction
     * @param power
     */
    public void move(DriveTrainDirections direction, Power power) {
        double powerDecrease = 1 - power.getPower();
        if (direction == Directions.NORTH) {
            getLeftRearMotor().setPower(direction.getLeftRearPower() - powerDecrease);
            getLeftFrontMotor().setPower(direction.getLeftFrontPower() - powerDecrease);
            getRightRearMotor().setPower(direction.getRightRearPower() + powerDecrease);
            getRightFrontMotor().setPower(direction.getRightFrontPower() + powerDecrease);
        } else if (direction == Directions.SOUTH) {
            getLeftRearMotor().setPower(direction.getLeftRearPower() + powerDecrease);
            getLeftFrontMotor().setPower(direction.getLeftFrontPower() + powerDecrease);
            getRightRearMotor().setPower(direction.getRightRearPower() - powerDecrease);
            getRightFrontMotor().setPower(direction.getRightFrontPower() - powerDecrease);
        } else if (direction == Directions.ROTATE_LEFT) {
            getLeftRearMotor().setPower(direction.getLeftRearPower() + powerDecrease);
            getLeftFrontMotor().setPower(direction.getLeftFrontPower() + powerDecrease);
            getRightRearMotor().setPower(direction.getRightRearPower() + powerDecrease);
            getRightFrontMotor().setPower(direction.getRightFrontPower() + powerDecrease);
        } else if (direction == Directions.ROTATE_RIGHT) {
            getLeftRearMotor().setPower(direction.getLeftRearPower() - powerDecrease);
            getLeftFrontMotor().setPower(direction.getLeftFrontPower() - powerDecrease);
            getRightRearMotor().setPower(direction.getRightRearPower() - powerDecrease);
            getRightFrontMotor().setPower(direction.getRightFrontPower() - powerDecrease);
        }
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
            flTarget = getRightRearMotor().getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = getRightFrontMotor().getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            rlTarget = getLeftRearMotor().getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            rrTarget = getLeftFrontMotor().getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Directions.SOUTH) {
            flTarget = getRightRearMotor().getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = getRightFrontMotor().getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            rlTarget = getLeftRearMotor().getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            rrTarget = getLeftFrontMotor().getTargetPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
        }

        getRightRearMotor().setTargetPosition(flTarget);
        getRightFrontMotor().setTargetPosition(frTarget);
        getLeftRearMotor().setTargetPosition(rlTarget);
        getLeftFrontMotor().setTargetPosition(rrTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        move(direction);

        while (getLeftRearMotor().isBusy() && getLeftFrontMotor().isBusy()
                && getRightRearMotor().isBusy() && getRightFrontMotor().isBusy()) {}

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

        //Set target positions
        int flTarget = 0;
        int frTarget = 0;
        int rlTarget = 0;
        int rrTarget = 0;

        if (direction == Directions.ROTATE_LEFT) {
            flTarget = getRightRearMotor().getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            frTarget = getRightFrontMotor().getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rlTarget = getLeftRearMotor().getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rrTarget = getLeftFrontMotor().getTargetPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Directions.ROTATE_RIGHT) {
            flTarget = getRightRearMotor().getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            frTarget = getRightFrontMotor().getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rlTarget = getLeftRearMotor().getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            rrTarget = getLeftFrontMotor().getTargetPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
        }

        getRightRearMotor().setTargetPosition(flTarget);
        getRightFrontMotor().setTargetPosition(frTarget);
        getLeftRearMotor().setTargetPosition(rlTarget);
        getLeftFrontMotor().setTargetPosition(rrTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        move(direction);

        while (getLeftRearMotor().isBusy() && getLeftFrontMotor().isBusy()
                && getRightRearMotor().isBusy() && getRightFrontMotor().isBusy()) {}

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
    @Override
    public void robotAlignToAngle(int angle) {
        if (getAdafruitIMU().getHeading() > getAdafruitIMU().getStartingHeading()) {
            rotateIMU(Directions.ROTATE_RIGHT,
                    (int) (getAdafruitIMU().getHeading()- getAdafruitIMU().getStartingHeading()));
        } else if (getAdafruitIMU().getHeading() < getAdafruitIMU().getStartingHeading()) {
            rotateIMU(Directions.ROTATE_LEFT,
                    (int) (getAdafruitIMU().getHeading()- getAdafruitIMU().getStartingHeading()));
        }
    }

    /**
     * Uses robot's light sensor to move in a given direction until it reaches the line
     * @param direction
     */
    public void robotAlignToLine(Directions direction) {
        while(getOds().getStatus() == LightSensor.Status.FLOOR) {
            move(direction, Power.LOW);
        }
    }
}
