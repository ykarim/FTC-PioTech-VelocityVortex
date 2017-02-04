package org.firstinspires.ftc.teamcode.robot;

import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public class RobotUtilities {

    private Robot robot = null;

    public boolean continuousIntake = false;
    public boolean continuousShoot = false;

    public RobotUtilities(Robot robot) {
        this.robot = robot;
    }

    public void capBall() {
        //TODO
    }

    public void pushBeaconButton(Beacon.BeaconAnalysis analysis, Robot.TeamColor teamColor) {
        //TODO: Extend beacon buttons
        boolean leftBlue, leftRed, rightBlue, rightRed;

        leftBlue = analysis.isLeftBlue();
        leftRed = analysis.isLeftRed();
        rightBlue = analysis.isRightBlue();
        rightRed = analysis.isRightRed();

        if (teamColor == Robot.TeamColor.BLUE) {
            if (leftBlue) {
                //Push left beacon button
            } else if (rightBlue) {
                //Push right beacon button
            }
        } else if (teamColor == Robot.TeamColor.RED) {
            if (leftRed) {
                //Push left beacon button
            } else if (rightRed) {
                //Push right beacon button
            }
        }
    }

    public void shootBall(LinearVisionOpMode opMode) {
        shootBalls(true);
        waitFor(opMode, RobotConstants.shotWaitPeriod);

        intakeBalls(true);
        waitFor(opMode, 3);

        intakeBalls(false);
        shootBalls(false);
    }

    public void shootDoubleBall(LinearVisionOpMode opMode) {
        shootBalls(true);
        waitFor(opMode, RobotConstants.shotWaitPeriod);

        intakeBalls(true);
        waitFor(opMode, 5);

        intakeBalls(false);
        shootBalls(false);
    }

    public void intakeBalls(boolean condition) {
        if (condition) {
            robot.intake.setPower(RobotConstants.intakeSpeed);
        } else {
            robot.intake.setPower(0);
        }
    }

    public void shootBalls(boolean condition) {
        if (condition) {
            robot.shoot.setPower(RobotConstants.shootSpeed);
        } else {
            robot.shoot.setPower(0);
        }
    }

    public void continuousIntake() {
        if (!continuousIntake) {
            continuousIntake = true;
            robot.intake.setPower(RobotConstants.intakeSpeed);
        } else {
            continuousIntake = false;
            robot.intake.setPower(0);
        }
    }

    public void continuousShoot() {
        if (!continuousShoot) {
            continuousShoot = true;
            robot.shoot.setPower(RobotConstants.shootSpeed);
        } else {
            continuousShoot = false;
            robot.shoot.setPower(0);
        }
    }

    /**
     * Aligns ODS by moving left or right given which side line is located on
     * @param direction
     */
    public void alignWithLine(RobotMovement.Direction direction) {
        robot.lightSensor.enableLed(true);

        RobotMovement robotMovement = new RobotMovement(robot);
        robotMovement.move(direction);

        while (robot.lightSensor.getLightDetected() > RobotConstants.whiteLineValue) { }
        robotMovement.move(RobotMovement.Direction.NONE);
    }

    private void waitFor(LinearVisionOpMode opMode, int sec) {
        long millis = sec * 1000;
        long stopTime = System.currentTimeMillis() + millis;
        while(opMode.opModeIsActive() && System.currentTimeMillis() < stopTime) {
            try {
                opMode.waitOneFullHardwareCycle();
            } catch(Exception ex) {}
        }
    }
}
