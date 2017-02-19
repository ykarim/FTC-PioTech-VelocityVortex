package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
import org.firstinspires.ftc.teamcode.sensors.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;

@Disabled //TODO: change into linVis op so that values can be extracted and thread can be tested
@Autonomous(name = "Beacon Push Test", group = "teletest")
public class BeaconPushTest extends OpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);
    private String TAG = RobotConstants.teleOpTag + "Beacon Test : ";

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
        if (gamepad1.a) {
            while (gamepad1.a) {}
            robotUtilities.pushBeacon(robot, getBeaconColor());
        } else if (gamepad1.dpad_left) {
            while (gamepad1.dpad_left) {}
            BeaconStatus.setLeftColor(BeaconStatus.Color.BLUE);
        } else if (gamepad1.dpad_right) {
            while (gamepad1.dpad_right) {}
            BeaconStatus.setLeftColor(BeaconStatus.Color.RED);
        } else if (gamepad1.dpad_up) {
            while (gamepad1.dpad_up) {}
            BeaconStatus.setRightColor(BeaconStatus.Color.BLUE);
        } else if (gamepad1.dpad_down) {
            while (gamepad1.dpad_down) {}
            BeaconStatus.setRightColor(BeaconStatus.Color.RED);
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

    private boolean inThresholdRange(double val) {
        return (val > RobotConstants.gamepadThreshold ||
                val < -RobotConstants.gamepadThreshold);
    }

    private void updateTelemetryData() {
        telemetry.addData(TAG, "Status : RUNNING");

        telemetry.addData(TAG, "Pressing beacon : " + robotUtilities.pressingBeacon);
        telemetry.update();
    }

    private BeaconStatus.Color getBeaconColor() {
        if (OpModeUtils.getTeamColor() == Robot.TeamColor.BLUE) {
            return BeaconStatus.Color.BLUE;
        } else if (OpModeUtils.getTeamColor() == Robot.TeamColor.RED) {
            return BeaconStatus.Color.RED;
        } else {
            return BeaconStatus.Color.NA;
        }
    }

}
