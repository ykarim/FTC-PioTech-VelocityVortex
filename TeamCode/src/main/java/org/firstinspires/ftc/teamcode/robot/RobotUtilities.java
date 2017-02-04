package org.firstinspires.ftc.teamcode.robot;

public class RobotUtilities {

    private Robot robot = null;

    public boolean continuousIntake = false;
    public boolean continuousShoot = false;

    public void init(Robot robot) {
        this.robot = robot;
    }

    public void capBall() {
        //TODO
    }

    public void pushBeaconButton() {
        //TODO
    }

    public void shootBall() {
        //TODO
    }

    public void intakeBalls(boolean condition) {
        if (condition) {
            robot.intake.setPower(RobotConstants.intakeSpeed);
        } else {
            robot.intake.setPower(0);
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

        RobotMovement robotMovement = new RobotMovement();
        robotMovement.init(robot);
        robotMovement.move(direction);

        while (robot.lightSensor.getLightDetected() > RobotConstants.whiteLineValue) { }
        robot.setDriveMotorPower(0);
    }
}
