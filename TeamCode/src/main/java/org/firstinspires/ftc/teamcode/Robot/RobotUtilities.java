package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sensors.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.Sensors.utils.PID;
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
                RobotConstants.moveSpeed = 0.0;
                break;
            } else if (robot.lightSensor.getLightDetected() > 0.3) {
                RobotConstants.moveSpeed = 0.4;
            } else if (robot.lightSensor.getLightDetected() > 0.2) {
                RobotConstants.moveSpeed = 0.45;
            } else {
                RobotConstants.moveSpeed = 0.5;
            }
            robotMovement.move(direction);
        }
        robotMovement.move(RobotMovement.Direction.NONE);

        RobotConstants.moveSpeed = 1.0;
        toggleLightLED();

        // Add PID loop to slow down once reaching
    }

    /**
     * Aligns robot with wall using ultrasonic (distance) sensors and constantly rotating 3 degrees
     * REQUIREMENTS:
     *    Not touching wall
     *    Farther than 5cm from wall
     *
     * @return if worked or not
     */
    public boolean alignWithWall() {
        double distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
        double distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);

        RobotMovement robotMovement = new RobotMovement(robot); //maybe lower speed to help

        if (distanceLeft != 0 && distanceRight != 0 && distanceLeft != 255 && distanceRight != 255) {
            while (distanceLeft - distanceRight > RobotConstants.sensorWallOffset) { //mb try with lower offset and correct sensor values in todo below
                robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 3); //3 PERFECT
                distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
            }

            while (distanceRight - distanceLeft > RobotConstants.sensorWallOffset) {
                robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, 3); //3 PERFECT
                distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
            }
            return true;
        } else {
            return false; //Rerun func.
        }
    }

    public boolean alignWithWallUsingRotation() {
        double distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
        double distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);

        RobotMovement robotMovement = new RobotMovement(robot);

        if (distanceLeft != 0 && distanceRight != 0 && distanceLeft != 255 && distanceRight != 255) {
            RobotConstants.rotateSpeed = 0.5;
            while (distanceLeft - distanceRight > RobotConstants.sensorWallOffset) {
                //Note: will go too fast make rotate return bool when done and then while (rotate = false) wait
                double angle = Math.toDegrees(Math.atan2(distanceLeft - distanceRight, 18));
                robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, angle);

                distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
            }

            while (distanceRight - distanceLeft > RobotConstants.sensorWallOffset) {
                double angle = Math.toDegrees(Math.atan2(distanceRight - distanceLeft, 18));
                robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, angle);

                distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
            }
            RobotConstants.rotateSpeed = 1.0;
            return true;
        } else {
            return false; //Rerun func.
        }
    }

    public boolean alignWithWallUsingPID(LinearOpMode opMode) {
        double distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
        double distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);

        RobotMovement robotMovement = new RobotMovement(robot);
        RobotConstants.rotateSpeed = 0.5;

        PID wallCtrl = new PID(0.01667, 0, 0);
        wallCtrl.setOpMode(opMode);
        wallCtrl.setTarget(0);

        if (distanceLeft != 0 && distanceRight != 0 && distanceLeft != 255 && distanceRight != 255) {
            if (distanceLeft - distanceRight > RobotConstants.sensorWallOffset) {
                double angle = (Math.atan2(distanceLeft - distanceRight, 18) / Math.PI) * 360;
                robotMovement.move(RobotMovement.Direction.ROTATE_RIGHT);

                //should probably be 0 to let PID do its job and not end early
                //TODO: calculate sensor offsets and test
                while (distanceLeft - distanceRight > RobotConstants.sensorWallOffset) {
                    double pid = wallCtrl.update(angle);

                    RobotConstants.rotateSpeed = 0.5 + pid;
                    robotMovement.move(RobotMovement.Direction.ROTATE_RIGHT); //mb fixes it

                    distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                    distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
                    angle = (Math.atan2(distanceLeft - distanceRight, 18) / Math.PI) * 360;
                    wallCtrl.setKP(RobotConstants.rotateSpeed / angle);
                }
            }

            if (distanceRight - distanceLeft > RobotConstants.sensorWallOffset) {
                double angle = (Math.atan2(distanceRight - distanceLeft, 18) / Math.PI) * 360;
                robotMovement.move(RobotMovement.Direction.ROTATE_LEFT);

                while (distanceRight - distanceLeft > RobotConstants.sensorWallOffset) {
                    double pid = wallCtrl.update(angle);

                    RobotConstants.rotateSpeed = 0.5 + pid;
                    robotMovement.move(RobotMovement.Direction.ROTATE_LEFT);

                    distanceLeft = getUltrasonicLevel(robot.ultrasonicSensorLeft);
                    distanceRight = getUltrasonicLevel(robot.ultrasonicSensorRight);
                    angle = (Math.atan2(distanceRight - distanceLeft, 18) / Math.PI) * 360;
                    wallCtrl.setKP(RobotConstants.rotateSpeed / angle);
                }
            }
            RobotConstants.rotateSpeed = 1.0;
            return true;
        } else {
            return false;
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
