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

@Autonomous(name = "Double Beacon Test", group = "AutoTest")
public class AutoBeaconTest extends LinearVisionOpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement();
    private RobotUtilities robotUtilities = new RobotUtilities();

    private final int RED_ALLIANCE = 1;
    private final int BLUE_ALLIANCE = 2;

    @Override
    public void runOpMode() throws InterruptedException{
        waitFor(getDelay());
        waitForVisionStart();
        initVision();

        robot.initAutoOp(this, hardwareMap);
        robotMovement.init(robot);
        robotUtilities.init(robot);

        int teamColor = getTeamColor();
        boolean blueLeft, blueRight, redLeft, redRight;

        waitForStart();

        while (opModeIsActive()) {
            robotMovement.move(RobotMovement.Direction.NORTH, 36);
            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 90);
            robotMovement.move(RobotMovement.Direction.NORTH, 48);
            robotUtilities.alignWithLine(RobotMovement.Direction.EAST);

            blueLeft = beacon.getAnalysis().isLeftBlue();
            blueRight = beacon.getAnalysis().isRightBlue();
            redLeft = beacon.getAnalysis().isLeftRed();
            redRight = beacon.getAnalysis().isRightRed();

            if (teamColor == BLUE_ALLIANCE) {
                if (blueLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon One on Left");
                } else if (blueRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon One on Right");
                }
            } else if (teamColor == RED_ALLIANCE) {
                if (redLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon One on Left");
                } else if (redRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon One on Right");
                }
            }

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST);

            blueLeft = beacon.getAnalysis().isLeftBlue();
            redLeft = beacon.getAnalysis().isLeftRed();

            if (teamColor == BLUE_ALLIANCE) {
                if (blueLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon Two on Left");
                } else if (blueRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon Two on Right");
                }
            } else if (teamColor == RED_ALLIANCE) {
                if (redLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon Two on Left");
                } else if (redRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon Two on Right");
                }
            }

            telemetry.update();
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
     * @return int representing team color (RED_ALLIANCE or BLUE_ALLIANCE)
     */
    private int getTeamColor() {
        boolean blueChecked = FtcRobotControllerActivity.blueTeamColor.isChecked();
        boolean redChecked = FtcRobotControllerActivity.redTeamColor.isChecked();

        if(blueChecked) {
            return BLUE_ALLIANCE;
        } else if(redChecked) {
            return RED_ALLIANCE;
        }
        return 0;
    }

    /**
     * Returns int of millis to wait before running program
     * @return
     */
    private int getDelay() {
        Integer delay = Integer.parseInt(FtcRobotControllerActivity.autoDelay.getText().toString());
        return (delay * 1000);
    }

    /**
     * Wait a period of time. This will be non-blocking, so Thread away!
     * @param millis time to wait in milliseconds.
     */
    private final void waitFor(long millis) {
        long stopTime = System.currentTimeMillis() + millis;
        while(opModeIsActive() && System.currentTimeMillis() < stopTime) {
            try {
                waitOneFullHardwareCycle();
            } catch(Exception ex) {}
        }
    }
}
