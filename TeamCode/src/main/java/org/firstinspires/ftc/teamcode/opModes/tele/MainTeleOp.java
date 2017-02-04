package org.firstinspires.ftc.teamcode.opModes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;

@TeleOp (name = "TeleOp", group = "tele")
public class MainTeleOp extends OpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);
    private String TAG = RobotConstants.teleOpTag + "Main : ";

    @Override
    public void init() {
        telemetry.addData(TAG, "Status : INITIALIZING");
        robot.initTeleOp(hardwareMap);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);

        telemetry.addData(TAG, "Status : READY");
    }

    @Override
    public void loop() {
        updateTelemetryData();
        robotMovement.move(convertGamepadToMovement());
        if (inThresholdRange(gamepad2.left_stick_y)) {
            convertGamepadToIntake(gamepad2.left_stick_y);
        } else if (inThresholdRange(gamepad2.right_stick_y)) {
            convertGamepadToShoot(gamepad2.right_stick_y);
        } else if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) {}
            robotMovement.invertDirection();
        } else if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) {}
            robotMovement.move(RobotMovement.Direction.NONE);
        } else if (gamepad2.x) {
            while (gamepad2.x) { }
            robotUtilities.continuousIntake();
        } else if (gamepad2.y) {
            while (gamepad2.y) { }
            robotUtilities.continuousShoot();
        } else if (gamepad2.left_bumper) {
            while (gamepad2.left_bumper) {}
            RobotConstants.intakeSpeed += 0.05;
        } else if (gamepad2.right_bumper) {
            while (gamepad2.right_bumper) {}
            RobotConstants.intakeSpeed -= 0.05;
        } else if (inThresholdRange(gamepad2.left_trigger)) {
            while (inThresholdRange(gamepad2.left_trigger)) {}
            RobotConstants.shootSpeed += 0.05;
        } else if (inThresholdRange(gamepad2.right_trigger)) {
            while (inThresholdRange(gamepad2.right_trigger)) {}
            RobotConstants.shootSpeed -= 0.05;
        } else {
            if(!robotUtilities.continuousIntake) {
                robot.intake.setPower(0);
            }
            if (!robotUtilities.continuousShoot) {
                robot.shoot.setPower(0);
            }
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

    private void updateTelemetryData() {
        telemetry.addData(TAG, "Status : RUNNING");

        telemetry.addData(TAG, "Gamepad 1 Left Position : (" + gamepad1.left_stick_x + " , " + gamepad1.left_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 1 Right Position : (" + gamepad1.right_stick_x + " , " + gamepad1.right_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 2 Left Position : (" + gamepad2.left_stick_x + " , " + gamepad2.left_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 2 Right Position : (" + gamepad2.right_stick_x + " , " + gamepad2.right_stick_y + ")");

        telemetry.addData(TAG, "Cont. Intake : " + robotUtilities.continuousIntake);
        telemetry.addData(TAG, "Cont. Shoot : " + robotUtilities.continuousShoot);

        telemetry.addData(TAG, "Intake Speed : " + RobotConstants.intakeSpeed);
        telemetry.addData(TAG, "Shoot Speed : " + RobotConstants.shootSpeed);

        telemetry.update();
    }
}
