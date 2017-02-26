package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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
        convertGamepadToIntake(-gamepad2.left_stick_y);
        convertGamepadToShoot(-gamepad2.right_stick_y);
        convertGamepadToCap();

        if (gamepad1.a) { //Gamepad 1 - X Button

            //Unimplemented

        } else if (gamepad1.x) { //Gamepad 1 - X Button

            //Unimplemented

        } else if (gamepad1.b) { //Gamepad 1 - B Button

            //Unimp

        } else if (gamepad1.y) { //Gamepad 1 - Y Button

            //Unimp

        } else if (gamepad1.left_bumper) { // Gamepad 1 - Left Bumper : Inverts robot direction

            robotMovement.invertDirection();

        } else if (gamepad1.right_bumper) { //Gamepad 1 - Right Bumper


        } else if (gamepad2.a) { //Gamepad 2 - A Button

            //Unimplemented

        } else if (gamepad2.y) { //Gamepad 2 - B Button

            //Unimplemented

        } else if (gamepad2.x) { //Gamepad 2 - X Button : Toggles continuous intake

            while (gamepad2.x) { }
            robotUtilities.continuousIntake();

        } else if (gamepad2.b) { //Gamepad 2 - Y Button : Toggles continuous shoot

            while (gamepad2.b) { }
            robotUtilities.continuousShoot();

        } else if (gamepad2.left_bumper) { //Gamepad 2 - Left Bumper

            robot.capServo.setPosition(0.5);

        } else if (gamepad2.right_bumper) { //Gamepad 2 - Right Bumper


        } else if (inThresholdRange(gamepad2.left_trigger)) { //Gamepad 2 - Left Trigger : Increases shoot speed

            while (inThresholdRange(gamepad2.left_trigger)) {}
            RobotConstants.shootSpeed += 0.05;
            RobotConstants.shootSpeed = Range.clip(RobotConstants.shootSpeed,
                    RobotConstants.MIN_MOTOR_PWR, RobotConstants.MAX_MOTOR_PWR);

        } else if (inThresholdRange(gamepad2.right_trigger)) { //Gamepad 2 - Right Trigger : Decreases shoot speed

            while (inThresholdRange(gamepad2.right_trigger)) {}
            RobotConstants.shootSpeed -= 0.05;
            RobotConstants.shootSpeed = Range.clip(RobotConstants.shootSpeed,
                    RobotConstants.MIN_MOTOR_PWR, RobotConstants.MAX_MOTOR_PWR);

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
        } else if (gamepad1.left_stick_x > RobotConstants.gamepadDiagonalThreshold &&
                -gamepad1.left_stick_y > RobotConstants.gamepadDiagonalThreshold) {
            return RobotMovement.Direction.NORTHEAST;
        } else if (gamepad1.left_stick_x < -RobotConstants.gamepadDiagonalThreshold &&
                -gamepad1.left_stick_y > RobotConstants.gamepadDiagonalThreshold) {
            return RobotMovement.Direction.NORTHWEST;
        } else if (gamepad1.left_stick_x > RobotConstants.gamepadDiagonalThreshold &&
                -gamepad1.left_stick_y < -RobotConstants.gamepadDiagonalThreshold) {
            return RobotMovement.Direction.SOUTHEAST;
        } else if (gamepad1.left_stick_x < -RobotConstants.gamepadDiagonalThreshold &&
                -gamepad1.left_stick_y < -RobotConstants.gamepadDiagonalThreshold) {
            return RobotMovement.Direction.SOUTHWEST;
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

    private void convertGamepadToCap() {
        if(gamepad2.dpad_up) {
            robot.cap.setPower(1.0);
        } else if (gamepad2.dpad_down) {
            robot.cap.setPower(-1.0);
        } else {
            robot.cap.setPower(0);
        }
    }

    private boolean inThresholdRange(double val) {
        return (val > RobotConstants.gamepadThreshold ||
                val < -RobotConstants.gamepadThreshold);
    }

    private void updateTelemetryData() {
        telemetry.addData(TAG, "Status : RUNNING");

        telemetry.addData(TAG, "Gamepad 1 Left Position : (" + gamepad1.left_stick_x + " , " + gamepad1.left_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 1 Right Position : (" + gamepad1.right_stick_x + " , " + gamepad1.right_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 2 Left Position : (" + gamepad2.left_stick_x + " , " + gamepad2.left_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 2 Right Position : (" + gamepad2.right_stick_x + " , " + gamepad2.right_stick_y + ")");

        telemetry.addData(TAG, "Robot Direction" + convertGamepadToMovement().getDirection());

        telemetry.addData(TAG, "Cont. Intake : " + robotUtilities.continuousIntake);
        telemetry.addData(TAG, "Cont. Shoot : " + robotUtilities.continuousShoot);

        telemetry.addData(TAG, "Shoot Speed : " + RobotConstants.shootSpeed);

        telemetry.addData(TAG, "Inverted : " + RobotConstants.inverted);
        telemetry.update();
    }
}
