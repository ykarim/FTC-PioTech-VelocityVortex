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
}
