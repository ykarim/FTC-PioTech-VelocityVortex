package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
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
        waitFor(getDelay());
        waitForVisionStart();
        initVision();
        leo.initAutoOp(this, hardwareMap);

        Robot.TeamColor teamColor = getTeamColor();

        addToTelemetry("READY on " + teamColor.getTeamColor());
        waitForStart();

        while (opModeIsActive()) {
            robotMovement.move(RobotMovement.Direction.NORTH, 6);
            robotUtilities.shootDoubleBall(this);
            addToTelemetry("Shot Two Balls");

            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 180);
            robotMovement.invertDirection();

            robotMovement.move(RobotMovement.Direction.NORTH, 36);
            addToTelemetry("Pushed cap ball");

            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 90);
            robotMovement.move(RobotMovement.Direction.NORTH, 40);
            robotMovement.move(RobotMovement.Direction.WEST, 12);
            robotUtilities.alignWithLine(RobotMovement.Direction.EAST, 5);
            addToTelemetry("Aligned with line for beacon 1");

            robotUtilities.pushBeaconButton(beacon.getAnalysis(), teamColor);
            addToTelemetry("Pushed beacon 1");

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);
            addToTelemetry("Aligned with line for beacon 2");

            robotUtilities.pushBeaconButton(beacon.getAnalysis(), teamColor);
            addToTelemetry("Pushed beacon 2");

            robotMovement.move(RobotMovement.Direction.SOUTH, 6);
            robotMovement.move(RobotMovement.Direction.EAST, 96);
            addToTelemetry("Parked on corner vortex");

            addToTelemetry("DONE");
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

    /**
     * Reads Team Color from FtcRobotControllerActivity
     * @return Robot.TeamColor
     */
    private Robot.TeamColor getTeamColor() {
        boolean blueChecked = FtcRobotControllerActivity.blueTeamColor.isChecked();
        boolean redChecked = FtcRobotControllerActivity.redTeamColor.isChecked();

        if(blueChecked) {
            return Robot.TeamColor.BLUE;
        } else if(redChecked) {
            return Robot.TeamColor.RED;
        }
        return Robot.TeamColor.NONE;
    }

    /**
     * Returns int of sec to wait before running program
     * @return
     */
    private int getDelay() {
        try {
            return Integer.parseInt(FtcRobotControllerActivity.autoDelay.getText().toString());
        } catch (NumberFormatException nfe) {
            return 0;
        }
    }

    private void addToTelemetry (String... msg) {
        for (String text : msg) {
            telemetry.addData(TAG, text);
        }
        telemetry.update();
    }
    /**
     * Wait a period of time. This will be non-blocking, so Thread away!
     * @param sec time to wait in seconds.
     */
    private void waitFor(int sec) {
        long millis = sec * 1000;
        long stopTime = System.currentTimeMillis() + millis;
        while(opModeIsActive() && System.currentTimeMillis() < stopTime) {
            try {
                waitOneFullHardwareCycle();
            } catch(Exception ex) {}
        }
    }
}