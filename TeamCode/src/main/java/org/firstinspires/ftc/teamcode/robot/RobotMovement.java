package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotMovement {

    private Robot robot = null;
    private Orientation currentOrientation = Orientation.FRONT;

    public enum Direction {
        NORTH, SOUTH, EAST, WEST, NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST,
        ROTATE_LEFT, ROTATE_RIGHT, NONE
    }

    public enum Orientation {
        FRONT, BACK, RIGHT, LEFT
    }

    public RobotMovement(Robot robot) {
        this.robot = robot;
        orient(Orientation.FRONT);
    }

    public void move(Direction direction) {
        if (direction == Direction.NORTH) {
            robot.fl.setPower(-RobotConstants.moveSpeed);
            robot.fr.setPower(RobotConstants.moveSpeed);
            robot.bl.setPower(-RobotConstants.moveSpeed);
            robot.br.setPower(RobotConstants.moveSpeed);
        } else if (direction == Direction.SOUTH) {
            robot.fl.setPower(RobotConstants.moveSpeed);
            robot.fr.setPower(-RobotConstants.moveSpeed);
            robot.bl.setPower(RobotConstants.moveSpeed);
            robot.br.setPower(-RobotConstants.moveSpeed);
        } else if (direction == Direction.EAST) {
            robot.fl.setPower(-RobotConstants.moveSpeed);
            robot.fr.setPower(-RobotConstants.moveSpeed);
            robot.bl.setPower(RobotConstants.moveSpeed);
            robot.br.setPower(RobotConstants.moveSpeed);
        } else if (direction == Direction.WEST) {
            robot.fl.setPower(RobotConstants.moveSpeed);
            robot.fr.setPower(RobotConstants.moveSpeed);
            robot.bl.setPower(-RobotConstants.moveSpeed);
            robot.br.setPower(-RobotConstants.moveSpeed);
        } else if (direction == Direction.NORTHEAST) {
            robot.fl.setPower(-RobotConstants.moveSpeed);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(RobotConstants.moveSpeed);
        } else if (direction == Direction.NORTHWEST) {
            robot.fl.setPower(0);
            robot.fr.setPower(RobotConstants.moveSpeed);
            robot.bl.setPower(-RobotConstants.moveSpeed);
            robot.br.setPower(0);
        } else if (direction == Direction.SOUTHEAST) {
            robot.fl.setPower(0);
            robot.fr.setPower(-RobotConstants.moveSpeed);
            robot.bl.setPower(RobotConstants.moveSpeed);
            robot.br.setPower(0);
        } else if (direction == Direction.SOUTHWEST) {
            robot.fl.setPower(RobotConstants.moveSpeed);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(-RobotConstants.moveSpeed);
        } else if (direction == Direction.ROTATE_LEFT) {
            for (DcMotor motor : robot.driveMotors) {
                motor.setPower(RobotConstants.rotateSpeed);
            }
        } else if (direction == Direction.ROTATE_RIGHT) {
            for (DcMotor motor : robot.driveMotors) {
                motor.setPower(-RobotConstants.rotateSpeed);
            }
        } else if (direction == Direction.NONE) {
            robot.fl.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(0);
        }
    }

    /**
     * Prereqs:
     *    Encoders are attached
     *    Robot is init in auto mode.
     * @param direction
     * @param distance
     */
    public void move(Direction direction, double distance) {
        int flTarget = 0;
        int frTarget = 0;
        int blTarget = 0;
        int brTarget = 0;

        if (direction == Direction.NORTH) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Direction.SOUTH) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Direction.EAST) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Direction.WEST) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
        }
        robot.fl.setTargetPosition(flTarget);
        robot.fr.setTargetPosition(frTarget);
        robot.bl.setTargetPosition(blTarget);
        robot.br.setTargetPosition(brTarget);

        robot.setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(RobotConstants.moveSpeed);

        while (robot.fl.isBusy() & robot.fr.isBusy()
            && robot.bl.isBusy() && robot.br.isBusy()) {
        }

        robot.setDriveMotorPower(0);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Prereqs:
     *     Encoders are attached
     *     Robot is init in auto mode
     * @param direction LEFT or RIGHT
     * @param angle in degrees
     */
    public void rotate(Direction direction, double angle) {
        double radians = Math.toRadians(angle);
        double distanceToMove = radians * RobotConstants.distFromCenterToWheel;

        //Set target position
        int flTarget = 0;
        int frTarget = 0;
        int blTarget = 0;
        int brTarget = 0;

        if (direction == Direction.ROTATE_LEFT) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
        } else if (direction == Direction.ROTATE_RIGHT) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() - (int) (distanceToMove * RobotConstants.INCHES_PER_TICK);
        }
        robot.fl.setTargetPosition(flTarget);
        robot.fr.setTargetPosition(frTarget);
        robot.bl.setTargetPosition(blTarget);
        robot.br.setTargetPosition(brTarget);

        robot.setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(RobotConstants.moveSpeed);

        while (robot.fl.isBusy() && robot.fr.isBusy()
                && robot.bl.isBusy() && robot.br.isBusy()) { }

        robot.setDriveMotorPower(0);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void orient(Orientation orientation) {
        DcMotor tempFL = robot.fl;
        DcMotor tempFR = robot.fr;
        DcMotor tempBL = robot.bl;
        DcMotor tempBR = robot.br;

        switch (orientation) {
            case BACK:
                if (currentOrientation == Orientation.FRONT) {
                    //Do nothing
                } else if (currentOrientation == Orientation.BACK) {
                    robot.fl = tempBR;
                    robot.fr = tempBL;
                    robot.bl = tempFR;
                    robot.br = tempFL;
                } else if (currentOrientation == Orientation.RIGHT) {
                    robot.fl = tempBL;
                    robot.fr = tempFL;
                    robot.bl = tempBR;
                    robot.br = tempFR;
                } else if (currentOrientation == Orientation.LEFT) {
                    robot.fl = tempFR;
                    robot.fr = tempBR;
                    robot.bl = tempFL;
                    robot.br = tempBL;
                }

                break;
            case FRONT:
                if (currentOrientation == Orientation.FRONT) {
                    robot.fl = tempBR;
                    robot.fr = tempBL;
                    robot.bl = tempFR;
                    robot.br = tempFL;
                } else if (currentOrientation == Orientation.BACK) {
                    //Do nothing
                } else if (currentOrientation == Orientation.RIGHT) {
                    robot.fl = tempBL;
                    robot.fr = tempFL;
                    robot.bl = tempBR;
                    robot.br = tempFR;
                } else if (currentOrientation == Orientation.LEFT) {
                    robot.fl = tempBL;
                    robot.fr = tempFL;
                    robot.bl = tempBR;
                    robot.br = tempFR;
                }

                break;
            case LEFT:
                if (currentOrientation == Orientation.FRONT) {
                    robot.fl = tempFR;
                    robot.fr = tempBR;
                    robot.bl = tempFL;
                    robot.br = tempBL;
                } else if (currentOrientation == Orientation.BACK) {
                    robot.fl = tempBL;
                    robot.fr = tempFL;
                    robot.bl = tempBR;
                    robot.br = tempFR;
                } else if (currentOrientation == Orientation.RIGHT) {
                    //Do nothing
                } else if (currentOrientation == Orientation.LEFT) {
                    robot.fl = tempBR;
                    robot.fr = tempBL;
                    robot.bl = tempFR;
                    robot.br = tempFL;
                }

                break;
            case RIGHT:
                if (currentOrientation == Orientation.FRONT) {
                    robot.fl = tempBL;
                    robot.fr = tempFL;
                    robot.bl = tempBR;
                    robot.br = tempFR;
                } else if (currentOrientation == Orientation.BACK) {
                    robot.fl = tempFR;
                    robot.fr = tempBR;
                    robot.bl = tempFL;
                    robot.br = tempBL;
                } else if (currentOrientation == Orientation.RIGHT) {
                    robot.fl = tempBR;
                    robot.fr = tempBL;
                    robot.bl = tempFR;
                    robot.br = tempFL;
                } else if (currentOrientation == Orientation.LEFT) {
                    //Do nothing
                }

                break;
        }
        currentOrientation = orientation;
    }

    public void invertDirection() {
        if (RobotConstants.inverted) {
            orient(Orientation.FRONT);
            RobotConstants.inverted = false;
        } else {
            orient(Orientation.BACK);
            RobotConstants.inverted = true;
        }
    }
}
