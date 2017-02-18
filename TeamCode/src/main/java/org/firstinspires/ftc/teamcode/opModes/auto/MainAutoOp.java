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
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@Autonomous (name = "AutoOp", group = "auto")
public class MainAutoOp extends LinearVisionOpMode {

    private Robot leo = new Robot();
    private RobotMovement robotMovement = new RobotMovement(leo);
    private RobotUtilities robotUtilities = new RobotUtilities(leo);
    private final String TAG = RobotConstants.autoOpTag + "Main : ";
    public volatile boolean beaconThreadStop = false;

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeUtils.waitFor(this, OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();
        leo.initAutoOp(this, hardwareMap);

        Robot.TeamColor teamColor = OpModeUtils.getTeamColor();
        OpModeUtils.addToTelemetry(this, TAG, "READY on " + teamColor.getTeamColor());
        waitForStart();

        Thread beaconUpdate = new Thread() {

            @Override
            public void run() {
                while (!beaconThreadStop) {
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
        };

        while (opModeIsActive()) {
            beaconUpdate.start();
            robotMovement.orient(RobotMovement.Orientation.FRONT);
            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 45); //45 PERFECT

            robotMovement.move(RobotMovement.Direction.NORTH, 62); // ~61.56 inches NOT TESTED

            robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT, 45); //90 - 45 bc transversal

            //Move back a bit more and align with wall via gyro and ultra

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);

//            robotMovement.move(RobotMovement.Direction.SOUTH, 12);
//            robotMovement.move(RobotMovement.Direction.NORTH, 12);

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 10);

            //OLD PATH:
//            //Shoot Double Perfect at 13v
//            robotMovement.move(RobotMovement.Direction.NORTH, 12);
//            //TODO: Fix rotation here
//            robotUtilities.shootDoubleBall(this, 10);
//            OpModeUtils.addToTelemetry(this, "Shot Two Balls");
//
//            //Get a bit ahead of the beacon
//            robotMovement.move(RobotMovement.Direction.NORTH, 18);
//            robotMovement.move(RobotMovement.Direction.EAST, 24);
//            robotMovement.move(RobotMovement.Direction.NORTH, 36);
//            robotMovement.move(RobotMovement.Direction.EAST, 12);
//
//            //Change orientation
//            robotMovement.orient(RobotMovement.Orientation.RIGHT);
//
//            //Align with line now on right
//            robotUtilities.alignWithLine(RobotMovement.Direction.EAST, 5);
//
//            //TODO: Fix rotation here and maybe realign
//
//            //Push beacon one
//            robotUtilities.pushBeaconButton(robotMovement, beacon.getAnalysis(), teamColor);
//            OpModeUtils.addToTelemetry(this, TAG, "Pushed beacon 1");
//
//            //Align with line for beacon two
//            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 7);
//
//            //TODO: Fix rotation here and realign if necessary
//
//            //Push beacon two
//            robotUtilities.pushBeaconButton(robotMovement, beacon.getAnalysis(), teamColor);
//            OpModeUtils.addToTelemetry(this, TAG, "Pushed beacon 2");
//
//            //Reset orientation
//            robotMovement.orient(RobotMovement.Orientation.FRONT);
//
//            // Move away from beacon back
//            if (teamColor == Robot.TeamColor.BLUE) {
//                robotMovement.move(RobotMovement.Direction.WEST, 6);
//            } else if (teamColor == Robot.TeamColor.RED) {
//                robotMovement.move(RobotMovement.Direction.EAST, 6);
//            }
//            robotMovement.move(RobotMovement.Direction.SOUTH, 96);
//            OpModeUtils.addToTelemetry(this, TAG, "Parked on corner vortex");

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
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }
}