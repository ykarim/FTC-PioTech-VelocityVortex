package org.firstinspires.ftc.teamcode.sensors.gyro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.TankRobot;
import org.firstinspires.ftc.teamcode.utils.Power;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

@TeleOp (name= "Ada Test", group = "teletest")
public class AdaTest extends LinearVisionOpMode {

    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        waitForStart();
        while(opModeIsActive()) {
            moveLeftWheels();
            moveRightWheels();

            telemetry.addData("Heading", robot.getAdafruitIMU().getHeading());

            telemetry.addData("Pitch", robot.getAdafruitIMU().getPitch());

            telemetry.addData("Roll", robot.getAdafruitIMU().getRoll());
            telemetry.update();
        }
    }

    private void moveLeftWheels() {
        if (-gamepad1.left_stick_y > 0.2) {
            robot.getLeftRearMotor().setPower(Power.HIGH.getPower());
            robot.getLeftFrontMotor().setPower(Power.HIGH.getPower());
        } else if (-gamepad1.left_stick_y < -0.2) {
            robot.getRightRearMotor().setPower(-Power.HIGH.getPower());
            robot.getRightFrontMotor().setPower(-Power.HIGH.getPower());
        } else {
            robot.getLeftRearMotor().setPower(Power.NONE.getPower());
            robot.getLeftFrontMotor().setPower(Power.NONE.getPower());
        }
    }

    private void moveRightWheels() {
        if (-gamepad1.right_stick_y > 0.2) {
            robot.getLeftRearMotor().setPower(-Power.HIGH.getPower());
            robot.getLeftFrontMotor().setPower(-Power.HIGH.getPower());
        } else if (-gamepad1.right_stick_y < -0.2) {
            robot.getRightRearMotor().setPower(Power.HIGH.getPower());
            robot.getRightFrontMotor().setPower(Power.HIGH.getPower());
        } else {
            robot.getRightRearMotor().setPower(Power.NONE.getPower());
            robot.getRightFrontMotor().setPower(Power.NONE.getPower());
        }
    }

    private boolean inThresholdRange(double val) {
        return (val > RobotConstants.gamepadThreshold ||
                val < -RobotConstants.gamepadThreshold);
    }
}
