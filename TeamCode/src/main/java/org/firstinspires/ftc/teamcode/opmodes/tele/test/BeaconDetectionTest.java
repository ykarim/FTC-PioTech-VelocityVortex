package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.sensors.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.utils.Color;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@TeleOp (name = "Beacon Detection Test", group = "teletest")
public class BeaconDetectionTest extends LinearVisionOpMode {

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private final String TAG = RobotConstants.autoOpTag + "Beacon Detection Test : ";

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeUtils.waitFor(this, OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();

        robot.initTeleOp(hardwareMap);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        robotMovement.orient(RobotMovement.Orientation.RIGHT);

        BeaconAnalyzer beaconAnalyzer = new BeaconAnalyzer();
        Thread beaconUpdate = new Thread(beaconAnalyzer, "Beacon Analyzer");
        beaconUpdate.start();

        waitForStart();

        while (opModeIsActive()) {
            updateTelemetry();

            robotMovement.move(convertGamepadToMovement());
            if (gamepad1.a) {
                while (gamepad1.a) {}
                beaconAnalyzer.stop();
            } else if (gamepad1.y) {
                while (gamepad1.y) {}
                beaconAnalyzer.resume();
            }
        }
    }

    private void initVision() {
        this.setCamera(Cameras.SECONDARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE); //Landscape = best analysis

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
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

    private void updateTelemetry() {
        telemetry.clearAll();
        telemetry.addData("STATUS", "RUNNING");
        if (BeaconStatus.getLeftColor() == Color.BLUE) {
            telemetry.addData("Left", "BLUE");
        } else if (BeaconStatus.getLeftColor() == Color.RED) {
            telemetry.addData("Left", "RED");
        } else if (BeaconStatus.getLeftColor() == Color.NA) {
            telemetry.addData("Left", "NA");
        }

        if (BeaconStatus.getRightColor() == Color.BLUE) {
            telemetry.addData("Right", "BLUE");
        } else if (BeaconStatus.getRightColor() == Color.RED) {
            telemetry.addData("Right", "RED");
        } else if (BeaconStatus.getRightColor() == Color.NA) {
            telemetry.addData("Right", "NA");
        }
        telemetry.addData("Confidence", beacon.getAnalysis().getConfidence());
        telemetry.addData("Left Known", beacon.getAnalysis().isLeftKnown());
        telemetry.addData("Right Known", beacon.getAnalysis().isRightKnown());
        telemetry.update();
    }

    private class BeaconAnalyzer implements Runnable {
        private volatile boolean exit = false;

        public void run() {
            while (!exit) {
                if (beacon.getAnalysis().isLeftBlue()) {
                    BeaconStatus.setLeftColor(Color.BLUE);
                } else if (beacon.getAnalysis().isLeftRed()) {
                    BeaconStatus.setRightColor(Color.RED);
                }

                if (beacon.getAnalysis().isRightBlue()) {
                    BeaconStatus.setRightColor(Color.BLUE);
                } else if (beacon.getAnalysis().isRightRed()) {
                    BeaconStatus.setRightColor(Color.RED);
                }
            }
        }

        private void resume() {
            exit = false;
        }

        private void stop() {
            exit = true;
        }
    }

}
