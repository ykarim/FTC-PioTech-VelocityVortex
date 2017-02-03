package org.firstinspires.ftc.teamcode;

public class RobotUtilities {

    private Robot robot = null;

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
}
