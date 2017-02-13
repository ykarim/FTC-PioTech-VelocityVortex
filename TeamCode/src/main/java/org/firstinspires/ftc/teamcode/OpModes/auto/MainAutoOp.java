package org.firstinspires.ftc.teamcode.OpModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotConstants;
import org.firstinspires.ftc.teamcode.Robot.RobotMovement;
import org.firstinspires.ftc.teamcode.Robot.RobotUtilities;
import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;
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

    @Override
    public void runOpMode() throws InterruptedException{
        OpModeUtils.waitFor(this, OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();
        leo.initAutoOp(this, hardwareMap);

        Robot.TeamColor teamColor = OpModeUtils.getTeamColor();

        OpModeUtils.addToTelemetry(this, TAG, "READY on " + teamColor.getTeamColor());
        waitForStart();

        while (opModeIsActive()) {
            //Shoot Double Perfect at 13v
            robotMovement.move(RobotMovement.Direction.NORTH, 12);
            //TODO: Fix rotation here
            robotUtilities.shootDoubleBall(this, 10);
            OpModeUtils.addToTelemetry(this, "Shot Two Balls");

            //Get a bit ahead of the beacon
            robotMovement.move(RobotMovement.Direction.NORTH, 18);
            robotMovement.move(RobotMovement.Direction.EAST, 24);
            robotMovement.move(RobotMovement.Direction.NORTH, 36);
            robotMovement.move(RobotMovement.Direction.EAST, 12);

            //Change orientation
            robotMovement.orient(RobotMovement.Orientation.RIGHT);

            //Align with line now on right
            robotUtilities.alignWithLine(RobotMovement.Direction.EAST, 5);

            //TODO: Fix rotation here and maybe realign

            //Push beacon one
            robotUtilities.pushBeaconButton(robotMovement, beacon.getAnalysis(), teamColor);
            OpModeUtils.addToTelemetry(this, TAG, "Pushed beacon 1");

            //Align with line for beacon two
            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 7);

            //TODO: Fix rotation here and realign if necessary

            //Push beacon two
            robotUtilities.pushBeaconButton(robotMovement, beacon.getAnalysis(), teamColor);
            OpModeUtils.addToTelemetry(this, TAG, "Pushed beacon 2");

            //Reset orientation
            robotMovement.orient(RobotMovement.Orientation.FRONT);

            // Move away from beacon back
            if (teamColor == Robot.TeamColor.BLUE) {
                robotMovement.move(RobotMovement.Direction.WEST, 6);
            } else if (teamColor == Robot.TeamColor.RED) {
                robotMovement.move(RobotMovement.Direction.EAST, 6);
            }
            robotMovement.move(RobotMovement.Direction.SOUTH, 96);
            OpModeUtils.addToTelemetry(this, TAG, "Parked on corner vortex");

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