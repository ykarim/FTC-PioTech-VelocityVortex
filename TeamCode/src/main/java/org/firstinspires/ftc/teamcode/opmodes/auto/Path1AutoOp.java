package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@Autonomous (name = "Path1", group = "auto")
@Disabled
public class Path1AutoOp extends LinearVisionOpMode {

    private Robot leo = new Robot();
    private RobotMovement robotMovement = new RobotMovement(leo);
    private RobotUtilities robotUtilities = new RobotUtilities(leo);
    private final String TAG = RobotConstants.autoOpTag + "Path 1 : ";

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeUtils.waitFor(this, OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();
        leo.initAutoOp(this, hardwareMap);

        Color teamColor = OpModeUtils.getTeamColor();
        BeaconAnalyzer beaconAnalyzer = new BeaconAnalyzer();
        Thread beaconUpdate = new Thread(beaconAnalyzer, "Beacon Analyzer");
        OpModeUtils.addToTelemetry(this, TAG, "READY on " + teamColor.getColor());
        waitForStart();

        beaconUpdate.start();
        while (opModeIsActive()) {
            //TODO: Fix rotation where appropriate through testing
            robotMovement.orient(RobotMovement.Orientation.FRONT);
            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 45); //45 PERFECT
            robotMovement.move(RobotMovement.Direction.NORTH, 62); // ~61.56 inches NOT TESTED
            robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, 45); //90 - 45 bc transversal
            robotMovement.orient(RobotMovement.Orientation.RIGHT);

            //Move back a bit more and align with wall via gyro and ultra
            robotMovement.move(RobotMovement.Direction.SOUTH, 6);
            beaconAnalyzer.stop();

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);
            robotUtilities.pushBeacon(leo, getDesiredColor());
            robotMovement.move(RobotMovement.Direction.SOUTH, 12);
            beaconAnalyzer.resume();
            //wait a bit for analysis to finish

            if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                    BeaconStatus.getRightColor() != getDesiredColor()) {
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.pushBeacon(leo, getDesiredColor());
            }

            robotMovement.move(RobotMovement.Direction.SOUTH, 4);
            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);
            beaconAnalyzer.stop();
            robotUtilities.pushBeacon(leo, getDesiredColor());
            robotMovement.move(RobotMovement.Direction.SOUTH, 12);
            beaconAnalyzer.resume();

            if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                    BeaconStatus.getRightColor() != getDesiredColor()) {
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.pushBeacon(leo, getDesiredColor());
            }

            robotMovement.move(RobotMovement.Direction.SOUTH, 4);
            robotMovement.move(RobotMovement.Direction.EAST, 96);

            OpModeUtils.addToTelemetry(this, TAG, "DONE");
            requestOpModeStop();
        }
    }

    private void initVision() {
        this.setCamera(Cameras.SECONDARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);

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