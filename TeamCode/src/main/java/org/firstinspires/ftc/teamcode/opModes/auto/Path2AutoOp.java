package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
import org.firstinspires.ftc.teamcode.sensors.beacon.BeaconStatus;
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

        Robot.TeamColor teamColor = OpModeUtils.getTeamColor();
        OpModeUtils.addToTelemetry(this, TAG, "READY on " + teamColor.getTeamColor());

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

    private BeaconStatus.Color getDesiredColor() {
        if (OpModeUtils.getTeamColor() == Robot.TeamColor.BLUE) {
            return BeaconStatus.Color.BLUE;
        } else if (OpModeUtils.getTeamColor() == Robot.TeamColor.RED) {
            return BeaconStatus.Color.RED;
        }
        return BeaconStatus.Color.NA;
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