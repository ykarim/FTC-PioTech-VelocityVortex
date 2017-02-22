package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotMovement {

    private Robot robot = null;
    private Orientation currentOrientation = Orientation.FRONT;

    public enum Direction {
        NORTH("NORTH"), SOUTH("SOUTH"), EAST("EAST"), WEST("WEST"),
        NORTHEAST("NORTHEAST"), NORTHWEST("NORTHWEST"), SOUTHEAST("SOUTHEAST"), SOUTHWEST("SOUTHWEST"),
        ROTATE_LEFT("ROTATE LEFT"), ROTATE_RIGHT("ROTATE RIGHT"), NONE("NONE");

        private final String direction;

        Direction(String direction) {
            this.direction = direction;
        }

        public String getDirection() {
            return direction;
        }
    }

    public enum Orientation {
        FRONT, BACK, RIGHT, LEFT
    }

    public RobotMovement(Robot robot) {
        this.robot = robot;
        orient(Orientation.FRONT);
    }

    public void move(Direction direction) {
        Direction orientedDirection = direction;
        if (direction != Direction.ROTATE_LEFT && direction != Direction.ROTATE_RIGHT) {
            orientedDirection = getOrientedDirection(direction);
        }

        if (orientedDirection == Direction.NORTH) {
            robot.fl.setPower(-RobotConstants.moveSpeed);
            robot.fr.setPower(RobotConstants.moveSpeed);
            robot.bl.setPower(-RobotConstants.moveSpeed);
            robot.br.setPower(RobotConstants.moveSpeed);
        } else if (orientedDirection == Direction.SOUTH) {
            robot.fl.setPower(RobotConstants.moveSpeed);
            robot.fr.setPower(-RobotConstants.moveSpeed);
            robot.bl.setPower(RobotConstants.moveSpeed);
            robot.br.setPower(-RobotConstants.moveSpeed);
        } else if (orientedDirection == Direction.EAST) {
            robot.fl.setPower(-RobotConstants.moveSpeed);
            robot.fr.setPower(-RobotConstants.moveSpeed);
            robot.bl.setPower(RobotConstants.moveSpeed);
            robot.br.setPower(RobotConstants.moveSpeed);
        } else if (orientedDirection == Direction.WEST) {
            robot.fl.setPower(RobotConstants.moveSpeed);
            robot.fr.setPower(RobotConstants.moveSpeed);
            robot.bl.setPower(-RobotConstants.moveSpeed);
            robot.br.setPower(-RobotConstants.moveSpeed);
        } else if (orientedDirection == Direction.NORTHEAST) {
            robot.fl.setPower(-RobotConstants.moveSpeed);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(RobotConstants.moveSpeed);
        } else if (orientedDirection == Direction.NORTHWEST) {
            robot.fl.setPower(0);
            robot.fr.setPower(RobotConstants.moveSpeed);
            robot.bl.setPower(-RobotConstants.moveSpeed);
            robot.br.setPower(0);
        } else if (orientedDirection == Direction.SOUTHEAST) {
            robot.fl.setPower(0);
            robot.fr.setPower(-RobotConstants.moveSpeed);
            robot.bl.setPower(RobotConstants.moveSpeed);
            robot.br.setPower(0);
        } else if (orientedDirection == Direction.SOUTHWEST) {
            robot.fl.setPower(RobotConstants.moveSpeed);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.br.setPower(-RobotConstants.moveSpeed);
        } else if (orientedDirection == Direction.ROTATE_LEFT) {
            for (DcMotor motor : robot.driveMotors) {
                motor.setPower(RobotConstants.rotateSpeed);
            }
        } else if (orientedDirection == Direction.ROTATE_RIGHT) {
            for (DcMotor motor : robot.driveMotors) {
                motor.setPower(-RobotConstants.rotateSpeed);
            }
        } else if (orientedDirection == Direction.NONE) {
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
     * @param direction Direction to move
     * @param distance Inches to move in direction
     */
    public void move(Direction direction, double distance) {
        int flTarget = 0;
        int frTarget = 0;
        int blTarget = 0;
        int brTarget = 0;

        Direction orientedDirection = getOrientedDirection(direction);

        if (orientedDirection == Direction.NORTH) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (orientedDirection == Direction.SOUTH) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (orientedDirection == Direction.EAST) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = robot.br.getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (orientedDirection == Direction.WEST) {
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

        while (robot.fl.isBusy() && robot.fr.isBusy()
            && robot.bl.isBusy() && robot.br.isBusy()) {}

        robot.setDriveMotorPower(0);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(Direction direction, double distance) {
        int flTarget = 0;
        int frTarget = 0;
        int blTarget = 0;
        int brTarget = 0;

        Direction orientedDirection = getOrientedDirection(direction);

        if (orientedDirection == Direction.NORTHEAST) {
            flTarget = robot.fl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = 0;
            blTarget = 0;
            brTarget = robot.br.getTargetPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
        } else if (orientedDirection == Direction.NORTHWEST) {
            flTarget = 0;
            frTarget = robot.fr.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = 0;
        } else if (orientedDirection == Direction.SOUTHEAST) {
            flTarget = 0;
            frTarget = robot.fr.getCurrentPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
            blTarget = robot.bl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            brTarget = 0;
        } else if (orientedDirection == Direction.SOUTHWEST) {
            flTarget = robot.fl.getCurrentPosition() + (int) (distance * RobotConstants.INCHES_PER_TICK);
            frTarget = 0;
            blTarget = 0;
            brTarget = robot.br.getTargetPosition() - (int) (distance * RobotConstants.INCHES_PER_TICK);
        }
        robot.fl.setTargetPosition(flTarget);
        robot.fr.setTargetPosition(frTarget);
        robot.bl.setTargetPosition(blTarget);
        robot.br.setTargetPosition(brTarget);

        robot.setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(RobotConstants.moveSpeed);

        if (orientedDirection == Direction.NORTHEAST) {
            while (robot.fl.isBusy() && robot.br.isBusy()) {}
        } else if (orientedDirection == Direction.NORTHWEST) {
            while (robot.fr.isBusy() && robot.bl.isBusy()) {}
        } else if (orientedDirection == Direction.SOUTHEAST) {
            while (robot.fr.isBusy() && robot.bl.isBusy()) {}
        } else if (orientedDirection == Direction.SOUTHWEST) {
            while (robot.fl.isBusy() && robot.br.isBusy()) {}
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
                && robot.bl.isBusy() && robot.br.isBusy()) {}

        robot.setDriveMotorPower(0);
        robot.setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void orient(Orientation orientation) {
        currentOrientation = orientation;
    }

    private Direction getOrientedDirection(Direction origDirection) {
        switch (currentOrientation) {
            case FRONT:
                return origDirection;
            case BACK:
                if (origDirection == Direction.NORTH) {
                    return Direction.SOUTH;
                } else if (origDirection == Direction.SOUTH) {
                    return Direction.NORTH;
                } else if (origDirection == Direction.EAST) {
                    return Direction.WEST;
                } else if (origDirection == Direction.WEST) {
                    return Direction.EAST;
                } else if (origDirection == Direction.NORTHEAST) {
                    return Direction.SOUTHWEST;
                } else if (origDirection == Direction.NORTHWEST) {
                    return Direction.SOUTHEAST;
                } else if (origDirection == Direction.SOUTHEAST) {
                    return Direction.NORTHWEST;
                } else if (origDirection == Direction.SOUTHWEST) {
                    return Direction.NORTHEAST;
                }
                break;
            case RIGHT:
                if (origDirection == Direction.NORTH) {
                    return Direction.EAST;
                } else if (origDirection == Direction.SOUTH) {
                    return Direction.WEST;
                } else if (origDirection == Direction.EAST) {
                    return Direction.SOUTH;
                } else if (origDirection == Direction.WEST) {
                    return Direction.NORTH;
                } else if (origDirection == Direction.NORTHEAST) {
                    return Direction.SOUTHEAST;
                } else if (origDirection == Direction.NORTHWEST) {
                    return Direction.NORTHEAST;
                } else if (origDirection == Direction.SOUTHEAST) {
                    return Direction.SOUTHWEST;
                } else if (origDirection == Direction.SOUTHWEST) {
                    return Direction.NORTHWEST;
                }
                break;
            case LEFT:
                if (origDirection == Direction.NORTH) {
                    return Direction.WEST;
                } else if (origDirection == Direction.SOUTH) {
                    return Direction.EAST;
                } else if (origDirection == Direction.EAST) {
                    return Direction.NORTH;
                } else if (origDirection == Direction.WEST) {
                    return Direction.SOUTH;
                } else if (origDirection == Direction.NORTHEAST) {
                    return Direction.NORTHWEST;
                } else if (origDirection == Direction.NORTHWEST) {
                    return Direction.SOUTHWEST;
                } else if (origDirection == Direction.SOUTHEAST) {
                    return Direction.NORTHEAST;
                } else if (origDirection == Direction.SOUTHWEST) {
                    return Direction.SOUTHEAST;
                }
                break;
            default:
                break;
        }
        return Direction.NONE;
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
