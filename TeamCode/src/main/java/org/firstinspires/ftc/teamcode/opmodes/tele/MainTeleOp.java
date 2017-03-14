package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.TankRobot;
import org.firstinspires.ftc.teamcode.utils.Power;

@TeleOp (name = "TeleOp", group = "tele")
public class MainTeleOp extends OpMode{

    private Robot robot = null;
    private String TAG = RobotConstants.teleOpTag + "Main : ";
    //Remove Constants class through use of Base classes. Also add telemetry messages

    @Override
    public void init() {
        telemetry.addData(TAG, "Status : INITIALIZING");
        telemetry.update();
        robot = new TankRobot(hardwareMap);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        telemetry.addData(TAG, "Status : READY");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateTelemetryData();

        moveLeftWheels();
        moveRightWheels();
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

    private void updateTelemetryData() {
        telemetry.addData(TAG, "Status : RUNNING");

        telemetry.addData(TAG, "Gamepad 1 Left Position : (" + gamepad1.left_stick_x + " , " + gamepad1.left_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 1 Right Position : (" + gamepad1.right_stick_x + " , " + gamepad1.right_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 2 Left Position : (" + gamepad2.left_stick_x + " , " + gamepad2.left_stick_y + ")");
        telemetry.addData(TAG, "Gamepad 2 Right Position : (" + gamepad2.right_stick_x + " , " + gamepad2.right_stick_y + ")");

        telemetry.update();
    }
}
