package org.firstinspires.ftc.teamcode.sensors.gyro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

@TeleOp (name= "Ada Test", group = "teletest")
public class AdaTest extends LinearVisionOpMode {

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutoOp(this, hardwareMap);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        waitForStart();
        while(opModeIsActive()) {
            robotMovement.move(convertGamepadToMovement());
            if(gamepad1.a) {
                while (gamepad1.a){}
                robotUtilities.alignWithWallUsingGyro();
            }

            telemetry.addData("Calc Heading", robotUtilities.getGyroAngle());
            telemetry.addData("Heading", robot.imu.getHeading());

            telemetry.addData("Pitch", robot.imu.getPitch());

            telemetry.addData("Roll", robot.imu.getRoll());
            telemetry.update();
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
}
