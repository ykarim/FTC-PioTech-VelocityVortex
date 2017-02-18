package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensors.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public class RobotUtilities {

    private Robot robot = null;

    public boolean continuousIntake = false;
    public boolean continuousShoot = false;

    public boolean pressingBeacon = false;

    private boolean lightLED = true;

    private boolean secondUltraTest = false;

    public RobotUtilities(Robot robot) {
        this.robot = robot;
    }

    public void capBall() {
        //TODO
    }

    /**
     * Pushes beacon button once aligned with wall
     * @param robot
     */
    public void pushBeacon(Robot robot, BeaconStatus.Color desiredColor) {
        RobotMovement robotMovement = new RobotMovement(robot);
        robotMovement.orient(RobotMovement.Orientation.RIGHT);

        while (BeaconStatus.getLeftColor() != desiredColor &&
                BeaconStatus.getRightColor() != desiredColor) {
            robotMovement.move(RobotMovement.Direction.NORTH, 4);
            pressingBeacon = true;
            robotMovement.move(RobotMovement.Direction.SOUTH, 2);
            pressingBeacon = false;

            //Fix alignment here if off
        }
    }

    public void toggleLightLED() {
        if (lightLED) {
            lightLED = false;
            robot.lightSensor.enableLed(lightLED);
        } else {
            lightLED = true;
            robot.lightSensor.enableLed(lightLED);
        }
    }

    public void shootDoubleBall(LinearOpMode opMode, double timeout) {
        long stop = System.currentTimeMillis() + Math.round(timeout * 1000);

        shootBalls(true);
        OpModeUtils.waitFor(opMode, RobotConstants.shotWaitPeriod);

        if (System.currentTimeMillis() < stop) {
            intakeBalls(true);
            OpModeUtils.waitFor(opMode, RobotConstants.intakeWaitPeriod);
        }

        intakeBalls(false);
        shootBalls(false);
    }

    public void shootDoubleBall(LinearVisionOpMode opMode, double timeout) {
        long stop = System.currentTimeMillis() + Math.round(timeout * 1000);

        shootBalls(true);
        OpModeUtils.waitFor(opMode, RobotConstants.shotWaitPeriod);

        if (System.currentTimeMillis() < stop) {
            intakeBalls(true);
            OpModeUtils.waitFor(opMode, RobotConstants.intakeWaitPeriod);
        }

        intakeBalls(false);
        shootBalls(false);
    }

    public void intakeBalls(boolean condition) {
        if (condition) {
            robot.intake.setPower(-RobotConstants.intakeSpeed);
        } else {
            robot.intake.setPower(0);
        }
    }

    public void shootBalls(boolean condition) {
        if (condition) {
            robot.shoot.setPower(-RobotConstants.shootSpeed);
        } else {
            robot.shoot.setPower(0);
        }
    }

    public void continuousIntake() {
        if (!continuousIntake) {
            continuousIntake = true;
            robot.intake.setPower(-RobotConstants.intakeSpeed);
        } else {
            continuousIntake = false;
            robot.intake.setPower(0);
        }
    }

    public void continuousShoot() {
        if (!continuousShoot) {
            continuousShoot = true;
            robot.shoot.setPower(-RobotConstants.shootSpeed);
        } else {
            continuousShoot = false;
            robot.shoot.setPower(0);
        }
    }

    /**
     * Aligns ODS by moving left or right given which side line is located on
     * @param direction
     */
    @Deprecated
    public void alignWithLine(LinearOpMode opMode, RobotMovement.Direction direction, int timeoutSec) {
        long stop = System.currentTimeMillis() + (timeoutSec * 1000);

        if (!lightLED) {
            toggleLightLED();
        }

        RobotMovement robotMovement = new RobotMovement(robot);
        RobotConstants.moveSpeed = 0.4;
        robotMovement.move(direction);

        while (opMode.opModeIsActive() &&
                robot.lightSensor.getLightDetected() < RobotConstants.whiteLineValue
                && System.currentTimeMillis() < stop) {}
        robotMovement.move(RobotMovement.Direction.NONE);

        RobotConstants.moveSpeed = 1.0;
        toggleLightLED();

        // Replace with PID loop to slow down once reaching
        if (robot.lightSensor.getLightDetected() < RobotConstants.perfectWhiteLineValue
                && System.currentTimeMillis() < stop) {
            RobotMovement.Direction oppositeDir = RobotMovement.Direction.NONE;
            if (direction == RobotMovement.Direction.NORTH) {
                oppositeDir = RobotMovement.Direction.SOUTH;
            } else if (direction == RobotMovement.Direction.SOUTH) {
                oppositeDir = RobotMovement.Direction.NORTH;
            } else if (direction == RobotMovement.Direction.EAST) {
                oppositeDir = RobotMovement.Direction.WEST;
            } else if (direction == RobotMovement.Direction.WEST) {
                oppositeDir = RobotMovement.Direction.EAST;
            }
            alignWithLine(opMode, oppositeDir, timeoutSec);
        }
    }

    /**
     * Aligns ODS by moving left or right given which side line is located on
     * @param direction
     */
    public void alignWithLine(RobotMovement.Direction direction, int timeoutSec) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (!lightLED) {
            toggleLightLED();
        }

        RobotMovement robotMovement = new RobotMovement(robot);
        RobotConstants.moveSpeed = 0.5;
        robotMovement.orient(RobotMovement.Orientation.RIGHT); //Sensors are currently on the right
        robotMovement.move(direction);

        while (robot.lightSensor.getLightDetected() < RobotConstants.perfectWhiteLineValue
                && time.seconds() < timeoutSec) {
            if (robot.lightSensor.getLightDetected() > RobotConstants.whiteLineValue) {
                break;
            } else if (robot.lightSensor.getLightDetected() > 0.3) {
                RobotConstants.moveSpeed = 0.4;
            } else if (robot.lightSensor.getLightDetected() > 0.2) {
                RobotConstants.moveSpeed = 0.45;
            } else {
                RobotConstants.moveSpeed = 0.5;
            }
        }
        robotMovement.move(RobotMovement.Direction.NONE);

        RobotConstants.moveSpeed = 1.0;
        toggleLightLED();

        // Add PID loop to slow down once reaching
    }

    /**
     *
     * @return if worked or not
     */
    public boolean alignWithWall() {
        double distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
        double distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);

        RobotMovement robotMovement = new RobotMovement(robot);

        //Try using rotate right instead of continuously moving.
        //Probably better to use PID in this case as robot will receive continuous values.
        if (distanceLeft != 0 && distanceRight != 0 && distanceLeft != 255 && distanceRight != 255) {
            while (distanceLeft - distanceRight > RobotConstants.sensorWallOffset) {
                robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 5);
                distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
            }

            while (distanceRight - distanceLeft > RobotConstants.sensorWallOffset) {
                robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, 5);
                distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
            }
            return true;
        } else {
            return false; //Rerun func.
        }
    }

    /**
     * Returns distance in cm
     * TODO: Balance values according to test where on sensor its measured from
     * @return distance from target
     */
    public double getUltrasonicLevel(UltrasonicSensor sensor) {
        double distance = 0;
        if (sensor == robot.ultrasonicSensorLeft) {
            distance = robot.ultrasonicSensorLeft.getUltrasonicLevel();
        } else if (sensor == robot.ultrasonicSensorRight) {
            distance = robot.ultrasonicSensorRight.getUltrasonicLevel();
        }

        if (distance == 0 || distance > 200) {
            if (!secondUltraTest) {
                secondUltraTest = true;
                return getUltrasonicLevel(sensor);
            } else {
                secondUltraTest = false;
                return 0;
            }
        } else {
            return distance;
        }
    }
}
