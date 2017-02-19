package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
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

@Autonomous (name = "Path 2", group = "auto")
public class Path2AutoOp extends LinearVisionOpMode {

    private Robot leo = new Robot();
    private RobotMovement robotMovement = new RobotMovement(leo);
    private RobotUtilities robotUtilities = new RobotUtilities(leo);
    private final String TAG = RobotConstants.autoOpTag + "Path 2 : ";

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeUtils.waitFor(this, OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();
        leo.initAutoOp(this, hardwareMap);

        Color teamColor = OpModeUtils.getTeamColor();
        OpModeUtils.addToTelemetry(this, TAG, "READY on " + teamColor.getColor());

        BeaconAnalyzer beaconAnalyzer = new BeaconAnalyzer();
        Thread beaconUpdate = new Thread(beaconAnalyzer, "Beacon Analyzer");
        waitForStart();

        beaconUpdate.start();
        while (opModeIsActive()) {
            //TODO: Fix rotation where appropriate through testing
            beaconUpdate.start();
            robotMovement.strafe(RobotMovement.Direction.NORTHEAST, 70);
            robotUtilities.alignWithWallUsingRotation();

            robotMovement.orient(RobotMovement.Orientation.RIGHT);
            robotMovement.move(RobotMovement.Direction.SOUTH, 6);
            robotUtilities.alignWithWallUsingRotation();
            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 3);
            robotUtilities.alignWithWallUsingRotation();
            beaconAnalyzer.stop();

            robotUtilities.pushBeacon(leo, getDesiredColor());
            robotMovement.move(RobotMovement.Direction.SOUTH, 12);
            beaconAnalyzer.resume();
            //wait a bit for analysis to finish
            OpModeUtils.waitFor(this, 500 / 1000);

            if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                    BeaconStatus.getRightColor() != getDesiredColor()) {
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.pushBeacon(leo, getDesiredColor());
            }

            robotMovement.move(RobotMovement.Direction.SOUTH, 6);
            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);
            beaconAnalyzer.stop();
            robotUtilities.pushBeacon(leo, getDesiredColor());
            robotMovement.move(RobotMovement.Direction.SOUTH, 12);
            beaconAnalyzer.resume();
            OpModeUtils.waitFor(this, 500 / 1000);

            if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                    BeaconStatus.getRightColor() != getDesiredColor()) {
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.pushBeacon(leo, getDesiredColor());
            }

            robotMovement.strafe(RobotMovement.Direction.SOUTHWEST, 85);
            robotMovement.strafe(RobotMovement.Direction.SOUTHEAST, 40); //change

            OpModeUtils.addToTelemetry(this, TAG, "DONE");
            requestOpModeStop();
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

    private Color getDesiredColor() {
        return OpModeUtils.getTeamColor();
    }

    private class BeaconAnalyzer implements Runnable {
        private volatile boolean exit = false;

        public void run() {
            while (!exit) {
                if (beacon.getAnalysis().isBeaconFound()) {
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
        }

        private void resume() {
            exit = false;
        }

        private void stop() {
            exit = true;
        }
    }
}
