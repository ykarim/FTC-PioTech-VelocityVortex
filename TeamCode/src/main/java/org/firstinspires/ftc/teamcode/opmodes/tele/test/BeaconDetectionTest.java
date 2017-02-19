package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.sensors.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@TeleOp (name = "Beacon Detection", group = "teletest")
public class BeaconDetectionTest extends LinearVisionOpMode {

    private final String TAG = RobotConstants.autoOpTag + "Beacon Detection Test : ";

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeUtils.waitFor(this, OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();

        BeaconAnalyzer beaconAnalyzer = new BeaconAnalyzer();
        Thread beaconUpdate = new Thread(beaconAnalyzer, "Beacon Analyzer");
        beaconUpdate.start();

        waitForStart();

        while (opModeIsActive()) {
            updateTelemetry();

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
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }

    private void updateTelemetry() {
        telemetry.clearAll();
        telemetry.addData("STATUS", "RUNNING");
        if (BeaconStatus.getLeftColor() == BeaconStatus.Color.BLUE) {
            telemetry.addData("Left", "BLUE");
        } else if (BeaconStatus.getLeftColor() == BeaconStatus.Color.RED) {
            telemetry.addData("Left", "RED");
        } else if (BeaconStatus.getLeftColor() == BeaconStatus.Color.NA) {
            telemetry.addData("Left", "NA");
        }

        if (BeaconStatus.getRightColor() == BeaconStatus.Color.BLUE) {
            telemetry.addData("Right", "BLUE");
        } else if (BeaconStatus.getRightColor() == BeaconStatus.Color.RED) {
            telemetry.addData("Right", "RED");
        } else if (BeaconStatus.getRightColor() == BeaconStatus.Color.NA) {
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
                    BeaconStatus.setLeftColor(BeaconStatus.Color.BLUE);
                } else if (beacon.getAnalysis().isLeftRed()) {
                    BeaconStatus.setRightColor(BeaconStatus.Color.RED);
                }

                if (beacon.getAnalysis().isRightBlue()) {
                    BeaconStatus.setRightColor(BeaconStatus.Color.BLUE);
                } else if (beacon.getAnalysis().isRightRed()) {
                    BeaconStatus.setRightColor(BeaconStatus.Color.RED);
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
