package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;

@TeleOp (name = "TeleOp", group = "main")
public class MainTeleOp extends OpMode{

    Robot robot = new Robot();
    RobotMovement robotMovement = new RobotMovement();
    RobotUtilities robotUtilities = new RobotUtilities();

    @Override
    public void init() {
        robot.initTeleOp(hardwareMap);
        robotMovement.init(robot);
        robotUtilities.init(robot);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
    }

    @Override
    public void loop() {
        robotMovement.move(convertGamepadToMovement());
        if (inThresholdRange(gamepad2.left_stick_y)) {
            convertGamepadToIntake(gamepad2.left_stick_y);
        } else if (inThresholdRange(gamepad2.right_stick_y)) {
            convertGamepadToShoot(gamepad2.right_stick_y);
        } else if (gamepad2.x) {
            while (gamepad2.x) { }
            robotUtilities.continuousIntake();
        } else {
            if(!robotUtilities.continuousIntake) {
                robot.intake.setPower(0);
            }
            robot.shoot.setPower(0);
//            robot.cap.setPower(0);
        }
    }

    /**
     * Converts gamepad-1 x and y coords to robot directions
     * TODO: Add cases for diagonal movement
     * @return
     */
    private RobotMovement.Direction convertGamepadToMovement() {
        if (gamepad1.left_stick_y > RobotConstants.gamepadThreshold &&
                !inThresholdRange(gamepad1.left_stick_x)) {
            return RobotMovement.Direction.NORTH;
        } else if (gamepad1.left_stick_y < -RobotConstants.gamepadThreshold &&
                !inThresholdRange(gamepad1.left_stick_x)) {
            return RobotMovement.Direction.SOUTH;
        } else if (gamepad1.left_stick_x > RobotConstants.gamepadThreshold &&
                !inThresholdRange(gamepad1.left_stick_y)) {
            return RobotMovement.Direction.EAST;
        } else if (gamepad1.left_stick_x < -RobotConstants.gamepadThreshold &&
                !inThresholdRange(gamepad1.left_stick_y)) {
            return RobotMovement.Direction.WEST;
        } else if (inThresholdRange(gamepad1.left_trigger)) {
            return RobotMovement.Direction.ROTATE_LEFT;
        } else if (inThresholdRange(gamepad1.right_trigger)) {
            return RobotMovement.Direction.ROTATE_RIGHT;
        } else {
            return RobotMovement.Direction.NONE;
        }
    }

    private void convertGamepadToIntake(double value) {
        if (value > RobotConstants.gamepadThreshold) {
            robot.intake.setPower(RobotConstants.intakeSpeed);
        } else if (value < -RobotConstants.gamepadThreshold) {
            robot.intake.setPower(-RobotConstants.intakeSpeed);
        }
    }

    private void convertGamepadToShoot(double value) {
        if (value > RobotConstants.gamepadThreshold) {
            robot.shoot.setPower(RobotConstants.shootSpeed);
        } else if (value < -RobotConstants.gamepadThreshold) {
            robot.shoot.setPower(-RobotConstants.shootSpeed);
        }
    }

    private boolean inThresholdRange(double val) {
        if (val > RobotConstants.gamepadThreshold ||
                val < -RobotConstants.gamepadThreshold) {
            return true;
        } else {
            return false;
        }
    }
}
