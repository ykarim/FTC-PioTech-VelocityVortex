package org.firstinspires.ftc.teamcode.OpModes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotConstants;
import org.firstinspires.ftc.teamcode.Robot.RobotMovement;
import org.firstinspires.ftc.teamcode.Robot.RobotUtilities;

import static org.firstinspires.ftc.teamcode.Robot.RobotMovement.Direction.EAST;
import static org.firstinspires.ftc.teamcode.Robot.RobotMovement.Direction.ROTATE_LEFT;
import static org.firstinspires.ftc.teamcode.Robot.RobotMovement.Direction.ROTATE_RIGHT;

@TeleOp(name = "Rotation Test", group = "teletest")
public class RotationTest extends OpMode {

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);
    private String TAG = RobotConstants.teleOpTag + "ODS Test : ";

    private int rotationAngle = 45;

    @Override
    public void init() {
        telemetry.addData(TAG, "Status : INITIALIZING");
        robot.initTeleOp(hardwareMap);
        robotUtilities.toggleLightLED();
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        robotMovement.orient(RobotMovement.Orientation.FRONT);

        telemetry.addData(TAG, "Status : READY");
    }

    @Override
    public void loop() {
        updateTelemetryData();

        robotMovement.move(convertGamepadToMovement());
        if (gamepad1.x) {
            while (gamepad1.x) {}
            robotMovement.rotate(ROTATE_RIGHT, rotationAngle);
        } else if (gamepad1.b) {
            while (gamepad1.b) {}
            rotationAngle--;
        } else if (gamepad1.y) {
            while (gamepad1.y) {}
            rotationAngle++;
        } else if (gamepad1.a) {
            while (gamepad1.a) {}
            robotMovement.rotate(ROTATE_LEFT, rotationAngle);
        }
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
            return EAST;
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

        printControls();
        telemetry.addData("", "");
        telemetry.addData(TAG, "Rotation Angle : " + rotationAngle);
        telemetry.update();
    }

    private void printControls () {
        telemetry.addData(TAG, "G1 - X : Rotate Right");
        telemetry.addData(TAG, "G1 - A : Rotate Left");
    }
}