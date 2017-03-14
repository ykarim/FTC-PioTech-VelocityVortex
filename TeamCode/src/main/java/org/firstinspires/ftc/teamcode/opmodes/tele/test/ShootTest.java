package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;

@TeleOp (name = "Shoot Test", group = "teletest")
public class ShootTest extends LinearOpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);
    private String TAG = RobotConstants.teleOpTag + "ODS Test : ";

    @Override
    public void runOpMode() {
        telemetry.addData(TAG, "Status : INITIALIZING");
        robot.initAutoOp(this, hardwareMap);
        robotUtilities.toggleLightLED();
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        robotMovement.orient(RobotMovement.Orientation.RIGHT);

        telemetry.addData(TAG, "Status : READY");
        waitForStart();

        while (opModeIsActive()) {

            updateTelemetryData();

            robotMovement.move(convertGamepadToMovement());
            convertGamepadToShoot(-gamepad1.right_stick_y);
            convertGamepadToIntake(-gamepad2.left_stick_y);
            if (gamepad1.x) {
                while (gamepad1.y) {
                }
                RobotConstants.shootSpeed += 0.05;
            } else if (gamepad1.a) {
                while (gamepad1.b) {
                }
                RobotConstants.shootSpeed -= 0.05;
            } else if (gamepad1.b) {
                while (gamepad1.b) {
                }
                RobotConstants.shotWaitPeriod += 0.1;
            } else if (gamepad1.x) {
                while (gamepad1.x) {
                }
                RobotConstants.shotWaitPeriod -= 0.1;
            } else if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) {
                }
                shootDoubleBall();
            } else if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up) {
                }
                RobotConstants.intakeWaitPeriod += 0.1;
            } else if (gamepad1.dpad_down) {
                while (gamepad1.dpad_down) {
                }
                RobotConstants.intakeWaitPeriod -= 0.1;
            }
        }
    }

    private void waitFor(double sec) {
        long millis = Math.round(sec * 1000);
        try {
            wait(millis);
        } catch (InterruptedException exception) {
            telemetry.addData(TAG, "ERROR : " + exception);
        }
    }

    private void shootDoubleBall() {
        robotUtilities.shootBalls(true);
        waitFor(RobotConstants.shotWaitPeriod);

        robotUtilities.intakeBalls(true);
        waitFor(RobotConstants.intakeWaitPeriod);

        robotUtilities.intakeBalls(false);
        robotUtilities.shootBalls(false);
    }

    /**
     * Converts gamepad-1 x and y coords to robot directions
     * @return Direction
     */
    private RobotMovement.Direction convertGamepadToMovement() {
        if (gamepad1.left_stick_y > RobotConstants.gamepadThreshold &&
                !inThresholdRange(gamepad1.left_stick_x)) {
            return RobotMovement.Direction.SOUTH;
        } else if (gamepad1.left_stick_y < -RobotConstants.gamepadThreshold &&
                !inThresholdRange(gamepad1.left_stick_x)) {
            return RobotMovement.Direction.NORTH;
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
        } else {
            if (!robotUtilities.continuousIntake) {
                robot.intake.setPower(0);
            }
        }
    }

    private void convertGamepadToShoot(double value) {
        if (value > RobotConstants.gamepadThreshold) {
            robot.shoot.setPower(RobotConstants.shootSpeed);
        } else if (value < -RobotConstants.gamepadThreshold) {
            robot.shoot.setPower(-RobotConstants.shootSpeed);
        } else {
            if (!robotUtilities.continuousShoot) {
                robot.shoot.setPower(0);
            }
        }
    }

    private boolean inThresholdRange(double val) {
        return (val > RobotConstants.gamepadThreshold ||
                val < -RobotConstants.gamepadThreshold);
    }

    private void updateTelemetryData() {
        telemetry.addData(TAG, "Status : RUNNING");

        telemetry.addData(" ", " ");

        telemetry.addData(TAG, "Shoot Speed : " + RobotConstants.shootSpeed);

        telemetry.addData(" ", " ");

        telemetry.addData(TAG, "Shoot Wait : " + RobotConstants.shotWaitPeriod);
        telemetry.addData(TAG, "Intake Wait : " + RobotConstants.intakeWaitPeriod);

        telemetry.addData(" ", " ");

        telemetry.addData("Shoot Rate : ", robotUtilities.getShooterRate());

        telemetry.addData(TAG, "Voltage : " + robot.voltageSensor.getVoltage());
        telemetry.update();
    }
}
