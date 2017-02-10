package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.auto.utils.Path1;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
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
            robotUtilities.shootDoubleBall(this, 10);
            OpModeUtils.addToTelemetry(this, "Shot Two Balls");

            robotMovement.move(RobotMovement.Direction.NORTH, 18);

            robotMovement.move(RobotMovement.Direction.EAST, 12);
            robotMovement.move(RobotMovement.Direction.NORTH, 36);

            robotMovement.move(RobotMovement.Direction.EAST, 12);

            robotMovement.orient(RobotMovement.Orientation.RIGHT);
            robotUtilities.alignWithLine(this, RobotMovement.Direction.WEST, 5);
            robotUtilities.pushBeaconButton(beacon.getAnalysis(), teamColor);
            OpModeUtils.addToTelemetry(this, TAG, "Pushed beacon 1");

//            path.goForBeaconTwo(this, beacon.getAnalysis(), OpModeUtils.getTeamColor());
            robotUtilities.alignWithLine(this, RobotMovement.Direction.WEST, 7);
            robotUtilities.pushBeaconButton(beacon.getAnalysis(), teamColor);
            OpModeUtils.addToTelemetry(this, TAG, "Pushed beacon 2");

//            path.goForVortex(OpModeUtils.getTeamColor());
            if (teamColor == Robot.TeamColor.BLUE) {
                robotMovement.move(RobotMovement.Direction.WEST, 6);
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