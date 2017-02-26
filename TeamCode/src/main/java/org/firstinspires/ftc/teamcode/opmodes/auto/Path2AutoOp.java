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

        while (opModeIsActive()) {
            //TODO: Fix rotation where appropriate through testing
            if (teamColor == Color.RED) {
                robotMovement.orient(RobotMovement.Orientation.BACK);

                //Move to beacon and align with line and push beacon
                robotMovement.strafe(RobotMovement.Direction.NORTHWEST, 70);
                robotMovement.move(RobotMovement.Direction.NORTH, 5);
                robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 3);
                robotUtilities.alignWithWallUsingGyro();
                robotMovement.orient(RobotMovement.Orientation.RIGHT);
                robotUtilities.pushBeacon(leo, getDesiredColor());

                //Shoot ball
                robotMovement.move(RobotMovement.Direction.SOUTH, 12);
                robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, 90);
                robotUtilities.shootDoubleBall(this, 5);
                robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 90);
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.alignWithWallUsingGyro();

                //Analyze beacon and update BeaconStatus
                beaconUpdate.start();
                robotMovement.move(RobotMovement.Direction.SOUTH, 12);
                OpModeUtils.waitFor(this, 1);
                beaconAnalyzer.stop();
                robotMovement.move(RobotMovement.Direction.NORTH, 10);
                robotUtilities.alignWithWallUsingGyro();

                //Push beacon if incorrect
                if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                        BeaconStatus.getRightColor() != getDesiredColor()) {
                    robotUtilities.pushBeacon(leo, getDesiredColor());
                }

                //Move onto next beacon on right and push
                robotMovement.move(RobotMovement.Direction.EAST, 5);
                robotUtilities.alignWithLine(RobotMovement.Direction.EAST, 5);
                robotUtilities.alignWithWallUsingGyro();
                robotUtilities.pushBeacon(leo, getDesiredColor());

                //Analyze 2nd beacon
                beaconAnalyzer.resume();
                robotMovement.move(RobotMovement.Direction.SOUTH, 12);
                OpModeUtils.waitFor(this, 1);
                beaconAnalyzer.stop();
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.alignWithWallUsingGyro();

                //Push beacon if incorrect
                if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                        BeaconStatus.getRightColor() != getDesiredColor()) {
                    robotUtilities.pushBeacon(leo, getDesiredColor());
                }

                //Go onto center vortex
                robotMovement.strafe(RobotMovement.Direction.SOUTHWEST, 85);

                //Move onto corner
                robotMovement.strafe(RobotMovement.Direction.NORTHWEST, 40);
            } else if (teamColor == Color.BLUE) {
                robotMovement.orient(RobotMovement.Orientation.FRONT);

                //Move onto beacon and align with line and push beacon
                robotMovement.strafe(RobotMovement.Direction.NORTHEAST, 70);
                robotMovement.move(RobotMovement.Direction.NORTH, 5);
                robotUtilities.alignWithLine(RobotMovement.Direction.EAST, 3);
                robotUtilities.alignWithWallUsingGyro();
                robotMovement.orient(RobotMovement.Orientation.RIGHT);
                robotUtilities.pushBeacon(leo, getDesiredColor());

                //Shoot Ball
                robotMovement.move(RobotMovement.Direction.SOUTH, 12);
                robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, 90);
                robotUtilities.shootDoubleBall(this, 5);
                robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 90);
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.alignWithWallUsingGyro();

                //Observe the beacon again to ensure correct color
                //Try not to hit beacon again
                beaconUpdate.start();
                robotMovement.move(RobotMovement.Direction.SOUTH, 12);
                OpModeUtils.waitFor(this, 1);
                beaconAnalyzer.stop();
                robotMovement.move(RobotMovement.Direction.NORTH, 10);
                robotUtilities.alignWithWallUsingGyro();

                //Push if incorrect color
                if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                        BeaconStatus.getRightColor() != getDesiredColor()) {
                    robotUtilities.pushBeacon(leo, getDesiredColor());
                }

                //Move onto next beacon which is on left and push
                robotMovement.move(RobotMovement.Direction.WEST, 5);
                robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);
                robotUtilities.alignWithWallUsingGyro();
                robotUtilities.pushBeacon(leo, getDesiredColor());

                //Analyze beacon to ensure correct color
                beaconAnalyzer.resume();
                robotMovement.move(RobotMovement.Direction.SOUTH, 12);
                OpModeUtils.waitFor(this, 1);
                beaconAnalyzer.stop();
                robotMovement.move(RobotMovement.Direction.NORTH, 12);
                robotUtilities.alignWithWallUsingGyro();

                //Push beacon if incorrect color
                if (BeaconStatus.getLeftColor() != getDesiredColor() &&
                        BeaconStatus.getRightColor() != getDesiredColor()) {
                    robotUtilities.pushBeacon(leo, getDesiredColor());
                }

                //Move to center to push ball
                robotMovement.strafe(RobotMovement.Direction.SOUTHEAST, 85);

                //Move onto corner vortex
                robotMovement.strafe(RobotMovement.Direction.NORTHEAST, 40); //change
            }

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
