package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public class RobotUtilities {

    private Robot robot = null;

    public boolean continuousIntake = false;
    public boolean continuousShoot = false;

    private boolean leftBeaconPusherExtended = false;
    private boolean rightBeaconPusherExtended = false;

    private boolean lightLED = true;

    public RobotUtilities(Robot robot) {
        this.robot = robot;
    }

    public void capBall() {
        //TODO
    }

    public void toggleBeaconPresser(Servo servo) {
        if (servo == robot.leftBeacon) {
            if (!leftBeaconPusherExtended) {
                RobotConstants.leftBeaconPusherPosition = RobotConstants.beaconPerfectPos;
                servo.setPosition(RobotConstants.leftBeaconPusherPosition);
                leftBeaconPusherExtended = true;
            } else {
                RobotConstants.leftBeaconPusherPosition = RobotConstants.SERVO_MIN;
                servo.setPosition(RobotConstants.leftBeaconPusherPosition);
                leftBeaconPusherExtended = false;
            }
        } else if (servo == robot.rightBeacon) {
            if (!rightBeaconPusherExtended) {
                RobotConstants.rightBeaconPusherPosition = RobotConstants.beaconPerfectPos;
                servo.setPosition(RobotConstants.rightBeaconPusherPosition);
                rightBeaconPusherExtended = true;
            } else {
                RobotConstants.rightBeaconPusherPosition = RobotConstants.SERVO_MIN;
                servo.setPosition(RobotConstants.rightBeaconPusherPosition);
                rightBeaconPusherExtended = false;
            }
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

    public void pushBeaconButton(RobotMovement robotMovement, Beacon.BeaconAnalysis analysis,
                                 Robot.TeamColor teamColor) {
        boolean leftBlue, leftRed, rightBlue, rightRed;

        leftBlue = analysis.isLeftBlue();
        leftRed = analysis.isLeftRed();
        rightBlue = analysis.isRightBlue();
        rightRed = analysis.isRightRed();

        if (teamColor == Robot.TeamColor.BLUE) {
            if (leftBlue) {
                toggleBeaconPresser(robot.leftBeacon);
                robotMovement.move(RobotMovement.Direction.NORTH, 4);
                toggleBeaconPresser(robot.leftBeacon);
            } else if (rightBlue) {
                toggleBeaconPresser(robot.rightBeacon);
                robotMovement.move(RobotMovement.Direction.NORTH, 4);
                toggleBeaconPresser(robot.rightBeacon);
            }
        } else if (teamColor == Robot.TeamColor.RED) {
            if (leftRed) {
                toggleBeaconPresser(robot.leftBeacon);
                robotMovement.move(RobotMovement.Direction.NORTH, 4);
                toggleBeaconPresser(robot.leftBeacon);
            } else if (rightRed) {
                toggleBeaconPresser(robot.rightBeacon);
                robotMovement.move(RobotMovement.Direction.NORTH, 4);
                toggleBeaconPresser(robot.rightBeacon);
            }
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
        RobotConstants.moveSpeed = 0.7;
        robotMovement.move(direction);

        while (robot.lightSensor.getLightDetected() < RobotConstants.whiteLineValue
                && time.seconds() < timeoutSec) {}
        robotMovement.move(RobotMovement.Direction.NONE);

        RobotConstants.moveSpeed = 1.0;
        toggleLightLED();

        // Add PID loop to slow down once reaching
    }

    /**
     * Reutrns distance in cm
     * @return distance from target
     */
    public double getUltrasonicLevel() {
        return robot.ultrasonicSensor.getUltrasonicLevel();
    }
}
