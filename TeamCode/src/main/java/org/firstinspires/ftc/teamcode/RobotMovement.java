package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RobotConstants.INCHES_PER_TICK;

public class RobotMovement {

    Robot robot = null;
    double moveSpeed = 1.0;
    double rotateSpeed = 0.5;

    enum Direction {
        NORTH, SOUTH, EAST, WEST, NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST,
        ROTATE_LEFT, ROTATE_RIGHT
    }

    public void init(Robot robot, double moveSpeed, double rotateSpeed) {
        this.robot = robot;
        this.moveSpeed = moveSpeed;
        this.rotateSpeed = rotateSpeed;
    }

    public void move(Direction direction) {
        if (direction == Direction.NORTH) {
            robot.fl.setPower(-moveSpeed);
            robot.fr.setPower(moveSpeed);
            robot.bl.setPower(-moveSpeed);
            robot.br.setPower(moveSpeed);
        } else if (direction == Direction.SOUTH) {
            robot.fl.setPower(moveSpeed);
            robot.fr.setPower(-moveSpeed);
            robot.bl.setPower(moveSpeed);
            robot.br.setPower(-moveSpeed);
        } else if (direction == Direction.EAST) {
            robot.fl.setPower(-moveSpeed);
            robot.fr.setPower(-moveSpeed);
            robot.bl.setPower(moveSpeed);
            robot.br.setPower(moveSpeed);
        } else if (direction == Direction.WEST) {
            robot.fl.setPower(moveSpeed);
            robot.fr.setPower(moveSpeed);
            robot.bl.setPower(-moveSpeed);
            robot.br.setPower(-moveSpeed);
        } else if (direction == Direction.NORTHEAST) {
            robot.fl.setPower(-moveSpeed);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(moveSpeed);
        } else if (direction == Direction.NORTHWEST) {
            robot.fl.setPower(0);
            robot.fr.setPower(moveSpeed);
            robot.bl.setPower(-moveSpeed);
            robot.br.setPower(0);
        } else if (direction == Direction.SOUTHEAST) {
            robot.fl.setPower(0);
            robot.fr.setPower(-moveSpeed);
            robot.bl.setPower(moveSpeed);
            robot.br.setPower(0);
        } else if (direction == Direction.SOUTHWEST) {
            robot.fl.setPower(moveSpeed);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(-moveSpeed);
        }
    }

    /**
     * Prereqs:
     *    Encoders are attached
     *    Robot is init in auto mode.
     */
    private void move(Direction direction, double distance) {
        int flTarget = 0;
        int frTarget = 0;
        int blTarget = 0;
        int brTarget = 0;

        if (direction == Direction.NORTH) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() + (int) (distance * INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distance * INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distance * INCHES_PER_TICK);
        } else if (direction == Direction.SOUTH) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distance * INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() - (int) (distance * INCHES_PER_TICK);
        } else if (direction == Direction.EAST) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distance * INCHES_PER_TICK);
        } else if (direction == Direction.WEST) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distance * INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() + (int) (distance * INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distance * INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() - (int) (distance * INCHES_PER_TICK);
        }

        robot.fl.setTargetPosition(flTarget);
        robot.fr.setTargetPosition(frTarget);
        robot.bl.setTargetPosition(blTarget);
        robot.br.setTargetPosition(brTarget);

        robot.setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(moveSpeed);

        while (robot.fl.isBusy() && robot.fr.isBusy()
                && robot.bl.isBusy() && robot.br.isBusy()) { }

        robot.setDriveMotorPower(0);
        robot.setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotate(Direction direction) {
        if (direction == Direction.ROTATE_LEFT) {
            for (DcMotor motor : robot.driveMotors) {
                motor.setPower(rotateSpeed);
            }
        } else if (direction == Direction.ROTATE_RIGHT) {
            for (DcMotor motor : robot.driveMotors) {
                motor.setPower(-rotateSpeed);
            }
        }
    }

    public void moveForward(double dist) {
        move(Direction.NORTH, dist);
    }

    public void moveBackward(double dist) {
        move(Direction.SOUTH, dist);
    }

    public void moveRight(double dist) {
        move(Direction.EAST, dist);
    }

    public void moveLeft(double dist) {
        move(Direction.WEST, dist);
    }
}
